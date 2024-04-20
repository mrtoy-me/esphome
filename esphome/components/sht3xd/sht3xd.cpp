#include "sht3xd.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace sht3xd {

static const char *const TAG = "sht3xd";

// https://sensirion.com/media/documents/E5762713/63D103C2/Sensirion_electronic_identification_code_SHT3x.pdf
// indicates two possible read serial number registers either with clock stretching enabled or disabled.
// Other SHT3XD_COMMAND registers use the clock stretching disabled register.
// To ensure compatibility, reading serial number using the register with clock stretching register enabled
// (used originally in this component) is tried first and if that fails the alternate register address
// with clock stretching disabled is read.

static const uint16_t SHT3XD_COMMAND_READ_SERIAL_NUMBER_CLOCK_STRETCHING = 0x3780;
static const uint16_t SHT3XD_COMMAND_READ_SERIAL_NUMBER = 0x3682;
static const uint16_t SHT3XD_COMMAND_BREAK =0x3093;
static const uint16_t SHT3XD_COMMAND_READ_STATUS = 0xF32D;
static const uint16_t SHT3XD_COMMAND_CLEAR_STATUS = 0x3041;
static const uint16_t SHT3XD_COMMAND_HEATER_ENABLE = 0x306D;
static const uint16_t SHT3XD_COMMAND_HEATER_DISABLE = 0x3066;
static const uint16_t SHT3XD_COMMAND_SOFT_RESET = 0x30A2;
static const uint16_t SHT3XD_COMMAND_POLLING_H = 0x2400;
static const uint16_t SHT3XD_COMMAND_FETCH_DATA = 0xE000;

void SHT3XDComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SHT3xD...");
  
  uint16_t raw_serial_number[2]{0,0};
  if (!this->get_register(SHT3XD_COMMAND_READ_SERIAL_NUMBER_CLOCK_STRETCHING, raw_serial_number, 2, 2)) {
    this->read_serial_error_code_ = READ_SERIAL_STRETCHED_FAILED;
    if (!this->get_register(SHT3XD_COMMAND_READ_SERIAL_NUMBER, raw_serial_number, 2, 2)) {
      this->read_serial_error_code_ = READ_SERIAL_FAILED;
      this->status_has_warning();
      return;
    }
  }
  
  this->serial_number_ = (uint32_t(raw_serial_number[0]) << 16) | uint32_t(raw_serial_number[1]);

  if (!this->write_command(SHT3XD_COMMAND_CLEAR_STATUS)) {
    this->heater_setup_error_code_ = WRITE_CLEAR_FAILED;
    this->mark_failed();
  }
  delay(2);

  if (!this->write_command(heater_enabled_ ? SHT3XD_COMMAND_HEATER_ENABLE : SHT3XD_COMMAND_HEATER_DISABLE)) {
    this->heater_setup_error_code_ = WRITE_HEATER_MODE_FAILED;
    this->mark_failed();
    return;
  }
  delay(2);
  if (!this->get_register(SHT3XD_COMMAND_READ_STATUS, &this->status_register_, 1, 2)) {
      this->heater_setup_error_code_ = READ_STATUS_FAILED;
      this->mark_failed();
      return;
  }
}

void SHT3XDComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "SHT3xD:");
  switch (this->read_serial_error_code_) {
    case READ_SERIAL_STRETCHED_FAILED:
      ESP_LOGW(TAG, "  Failed to read serial#(stretched)-trying alternate");
      break;
    case READ_SERIAL_FAILED:
      ESP_LOGW(TAG, "  Failed to read serial#");
      break; 
    default:
      break; 
  }
  switch (this->heater_setup_error_code_) {
    case WRITE_CLEAR_FAILED:
      ESP_LOGW(TAG, "  Failed write clear status");
      break;
    case WRITE_HEATER_MODE_FAILED:
      ESP_LOGW(TAG, "  Failed to write heater mode");
      break;
    case READ_STATUS_FAILED:
      ESP_LOGW(TAG, "  Failed to read heater mode");
      break;   
    default:
      break; 
  }

  if (this->read_serial_error_code_ == NO_READ_SERIAL_ERROR) {
    ESP_LOGD(TAG, "  Serial Number: 0x%08" PRIX32, this->serial_number_);
  }
  ESP_LOGD(TAG, "  Heater Enabled: %s", this->heater_enabled_ ? "true" : "false");
  if (this->heater_setup_error_code_ == NO_HEATER_SETUP_ERROR) {
    if ((this->status_register_ & 0x2000) == 0x2000) {
      ESP_LOGD(TAG, "  Heater Confirmed: ENABLED");
    } else {
      ESP_LOGD(TAG, "  Heater Confirmed: DISABLED");
    }
  } else {
    ESP_LOGD(TAG, "  Unable to confirm Heater mode");
  }

  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

float SHT3XDComponent::get_setup_priority() const { return setup_priority::DATA; }

void SHT3XDComponent::update() {
  if (this->status_has_warning()) {
    ESP_LOGD(TAG, "Trying to clear error with soft reset");
    if (!this->write_command(SHT3XD_COMMAND_SOFT_RESET)) {
      ESP_LOGD(TAG, "Failed to write soft reset");
    }
    delay(5);
  }
  if (!this->write_command(SHT3XD_COMMAND_POLLING_H)) {
    ESP_LOGD(TAG, "Failed to write polling mode");
    this->status_set_warning();
    return;
  }

  this->set_timeout(50, [this]() {
    uint16_t raw_data[2];
    if (!this->read_data(raw_data, 2)) {
      ESP_LOGD(TAG, "Failed to read sensor data");
      this->status_set_warning();
      return;
    }

    float temperature = 175.0f * float(raw_data[0]) / 65535.0f - 45.0f;
    float humidity = 100.0f * float(raw_data[1]) / 65535.0f;

    ESP_LOGD(TAG, "Got temperature=%.2f°C humidity=%.2f%%", temperature, humidity);
    if (this->temperature_sensor_ != nullptr)
      this->temperature_sensor_->publish_state(temperature);
    if (this->humidity_sensor_ != nullptr)
      this->humidity_sensor_->publish_state(humidity);
    this->status_clear_warning();
  });
}

}  // namespace sht3xd
}  // namespace esphome
