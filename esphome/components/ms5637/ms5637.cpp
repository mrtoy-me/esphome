#include "ms5637.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ms5637 {

static const char *const TAG = "ms5637";

// MS5637 device commands
static const uint8_t MS5637_RESET_COMMAND                    = 0x1E;
static const uint8_t MS5637_START_PRESSURE_ADC_CONVERSION    = 0x40;
static const uint8_t MS5637_START_TEMPERATURE_ADC_CONVERSION = 0x50;
static const uint8_t MS5637_READ_ADC                         = 0x00;

static const uint8_t MS5637_CONVERSION_OSR_MASK              = 0x0F;

// MS5637 first EEPROM read address
static const uint8_t MS5637_PROM_ADDRESS_READ_ADDRESS_0      = 0xA0;

// Coefficients indexes for temperature and pressure computation
static const uint8_t MS5637_COEFFICIENT_COUNT                        = 7;
static const uint8_t MS5637_CRC_INDEX                                = 0;
static const uint8_t MS5637_PRESSURE_SENSITIVITY_INDEX               = 1;
static const uint8_t MS5637_PRESSURE_OFFSET_INDEX                    = 2;
static const uint8_t MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX = 3;
static const uint8_t MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX      = 4;
static const uint8_t MS5637_REFERENCE_TEMPERATURE_INDEX              = 5;
static const uint8_t MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX          = 6;

void MS5637Component::setup() {
  this->conversion_time_osr_ = conversion_time_[this->resolution_osr_];

  ESP_LOGCONFIG(TAG, "Setting up MS5637...");

  if (this->write(nullptr, 0) != i2c::ERROR_OK) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  if (this->write(&MS5637_RESET_COMMAND,1) != i2c::ERROR_OK) {
    this->error_code_ = RESET_FAILED;
    this->mark_failed();
    return;
  }

  // read EEPROM Calibration Coefficients
  for (uint8_t i = 0; i < MS5637_COEFFICIENT_COUNT; i++) {
    if (!this->read_byte_16(MS5637_PROM_ADDRESS_READ_ADDRESS_0 + i * 2, this->eeprom_coeff_ + i)) {
      this->error_code_ = EEPROM_READ_FAILED;
      this->mark_failed();
      return;
    }
  }

  if (!this->crc_check(this->eeprom_coeff_, (this->eeprom_coeff_[MS5637_CRC_INDEX] & 0xF000) >> 12)) {
    this->error_code_ = EEPROM_CRC_FAILED;
    this->mark_failed();
    return;
  }
}

bool MS5637Component::crc_check(uint16_t *n_prom, uint8_t crc) {
  uint8_t cnt, n_bit;
  uint16_t n_rem, crc_read;

  n_rem = 0x00;
  crc_read = n_prom[0];
  n_prom[MS5637_COEFFICIENT_COUNT] = 0;
  n_prom[0] = (0x0FFF & (n_prom[0])); // Clear the CRC byte

  for (cnt = 0; cnt < (MS5637_COEFFICIENT_COUNT + 1) * 2; cnt++) {
    // Get next byte
    if (cnt % 2 == 1)
      n_rem ^= n_prom[cnt >> 1] & 0x00FF;
    else
      n_rem ^= n_prom[cnt >> 1] >> 8;

    for (n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000)
        n_rem = (n_rem << 1) ^ 0x3000;
      else
        n_rem <<= 1;
    }
  }
  n_rem >>= 12;
  n_prom[0] = crc_read;

  return (n_rem == crc);
}

void MS5637Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MS5637:");

  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGE(TAG, "  Communication startup failed");
      break;
    case RESET_FAILED:
      ESP_LOGE(TAG, "  Writing Reset Command failed");
      break;
    case EEPROM_READ_FAILED:
      ESP_LOGE(TAG, "  Reading EEPROM failed");
      break;
    case EEPROM_CRC_FAILED:
      ESP_LOGE(TAG, "  EEPROM Coefficients CRC check failed");
      break;
    case NONE:
      ESP_LOGI(TAG, "  Setup successful");
      break;
  }
  ESP_LOGD(TAG,"  Resolution: OSR %i",(256*(uint8_t)(1<<this->resolution_osr_)));
  ESP_LOGD(TAG,"  ADC conversion: %ims",this->conversion_time_osr_);
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
}

float MS5637Component::get_setup_priority() const { return setup_priority::DATA; }

void MS5637Component::update() {
  this->adc_temperature_ = 0;
  this->adc_pressure_ = 0;
  this->status_clear_warning();

  this->start_conversions();
}

void MS5637Component::start_conversions() {
  uint8_t cmd;
  // read temperature command
  cmd = this->resolution_osr_ * 2;
  cmd |= MS5637_START_TEMPERATURE_ADC_CONVERSION;

  if (this->write(&cmd,1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Error writing conversion command(temperature)");
    this->status_set_warning();
    return;
  }
  this->set_timeout("temperature", this->conversion_time_osr_, [this]() { this->read_temperature(); });
}

void MS5637Component::read_temperature() {
  uint8_t buffer[3];
  if (!this->read_bytes(MS5637_READ_ADC, buffer, 3)) {
    ESP_LOGW(TAG, "Error reading adc buffer(temperature)");
    this->status_set_warning();
    return;
  }
  this->adc_temperature_ = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
  this->do_pressure_conversion();
}

void MS5637Component::do_pressure_conversion() {
  uint8_t cmd;
  // read pressure
  cmd = this->resolution_osr_ * 2;
  cmd |= MS5637_START_PRESSURE_ADC_CONVERSION;

  if (this->write(&cmd,1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Error writing conversion command(pressure)");
    this->status_set_warning();
    return;
  }
  this->set_timeout("pressure", this->conversion_time_osr_, [this]() { this->read_pressure_and_publish(); });
}

void MS5637Component::read_pressure_and_publish() {
  uint8_t buffer[3];
  if (!this->read_bytes(MS5637_READ_ADC, buffer, 3)) {
    ESP_LOGW(TAG, "Error reading adc buffer(pressure)");
    this->status_set_warning();
    return;
  }
  this->adc_pressure_ = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

  if ( !this>calculate_temperature_and_pressure() ) return;

  if (this->temperature_sensor_!= nullptr) {
    ESP_LOGD(TAG, "'%s': new reading=%.2f°C", this->temperature_sensor_->get_name().c_str(), temperature_reading_);
    this->temperature_sensor_->publish_state(temperature_reading_);
  }
  if (this->pressure_sensor_ != nullptr) {
    ESP_LOGD(TAG, "'%s': new reading=%.1fhPa", this->pressure_sensor_->get_name().c_str(), pressure_reading_);
    this->pressure_sensor_->publish_state(pressure_reading_);
  }
}

bool MS5637Component::calculate_temperature_and_pressure() {
  int32_t dt, temp;
  int64_t off, sens, p, t2, off2, sens2;

  if (adc_temperature_ == 0 || adc_pressure_ == 0) {
    ESP_LOGW(TAG, "Error reading adc - zero read in either temperature or pressure");
    this->status_set_warning();
    return false;
  }

  // Difference between actual and reference temperature = D2 - Tref
  dt = (int32_t)adc_temperature_ - ((int32_t)this->eeprom_coeff_[MS5637_REFERENCE_TEMPERATURE_INDEX] << 8);

  // Actual temperature = 2000 + dT * TEMPSENS
  temp = 2000 + ((int64_t)dt *
                  (int64_t)this->eeprom_coeff_[MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23);

   // Second order temperature compensation
  if (temp < 2000) {
    t2 = (3 * ((int64_t)dt * (int64_t)dt)) >> 33;
    off2 = 61 * ((int64_t)temp - 2000) * ((int64_t)temp - 2000) / 16;
    sens2 = 29 * ((int64_t)temp - 2000) * ((int64_t)temp - 2000) / 16;

    if (temp < -1500) {
      off2 += 17 * ((int64_t)temp + 1500) * ((int64_t)temp + 1500);
      sens2 += 9 * ((int64_t)temp + 1500) * ((int64_t)temp + 1500);
    }
  }
  else {
    t2 = (5 * ((int64_t)dt * (int64_t)dt)) >> 38;
    off2 = 0;
    sens2 = 0;
  }

  // OFF = OFF_T1 + TCO * dT
  off = ((int64_t)(this->eeprom_coeff_[MS5637_PRESSURE_OFFSET_INDEX]) << 17) +
          (((int64_t)(this->eeprom_coeff_[MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dt) >> 6);
  off -= off2;

  // Sensitivity at actual temperature = SENS_T1 + TCS * dT
  sens = ((int64_t)this->eeprom_coeff_[MS5637_PRESSURE_SENSITIVITY_INDEX] << 16) +
           (((int64_t)this->eeprom_coeff_[MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dt) >> 7);
  sens -= sens2;

  // Temperature compensated pressure = D1 * SENS - OFF
  p = (((adc_pressure_ * sens) >> 21) - off) >> 15;

  temperature_reading_ = ((float)temp - (float)t2) / 100.0f;
  pressure_reading_= (float)p / 100.0f;
  return true;
}

} // namespace ms5637
} // namespace esphome
