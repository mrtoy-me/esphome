#include "lps25hb.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace lps25hb {

static const char *const TAG = "lps25hb";

static const uint8_t LPS25HB_DEVID = 0xBD;                    // Factory set identifier

// Resolution Configuration
static const uint8_t LPS25HB_RES_CONF_DEFAULT        = 0x0F;  // Default
static const uint8_t LPS25HB_RES_CONF_T_8            = 0x00;  // Temperature 8 internal averages
static const uint8_t LPS25HB_RES_CONF_T_16           = 0x04;  // Temperature 16 internal averages
static const uint8_t LPS25HB_RES_CONF_T_32           = 0x08;  // Temperature 32 internal averages
static const uint8_t LPS25HB_RES_CONF_T_64           = 0x0C;  // Temperature 64 internal averages
static const uint8_t LPS25HB_RES_CONF_P_8            = 0x00;  // Pressure 8 internal averages
static const uint8_t LPS25HB_RES_CONF_P_32           = 0x01;  // Pressure 32 internal averages
static const uint8_t LPS25HB_RES_CONF_P_128          = 0x02;  // Pressure 128 internal averages
static const uint8_t LPS25HB_RES_CONF_P_512          = 0x03;  // Pressure 512 internal averages

// Control Register 1
static const uint8_t LPS25HB_CTRL_REG1_DEFAULT       = 0x00;  // Default
static const uint8_t LPS25HB_CTRL_REG1_PD_ACTIVE     = 0x80;  // Active mode - needed to turn on sensor
static const uint8_t LPS25HB_CTRL_REG1_ODR_OS        = 0x00;  // One shot mode
static const uint8_t LPS25HB_CTRL_REG1_ODR_1HZ       = 0x10;  // Both T and P at 1Hz
static const uint8_t LPS25HB_CTRL_REG1_ODR_7HZ       = 0x20;  // Both T and P at 7Hz
static const uint8_t LPS25HB_CTRL_REG1_ODR_12HZ5     = 0x30;  // Both T and P at 12.5Hz
static const uint8_t LPS25HB_CTRL_REG1_ODR_25HZ      = 0x40;  // Both T and P at 25Hz
static const uint8_t LPS25HB_CTRL_REG1_DIFF_EN       = 0x08;  // Enable diff interrupt generation (datasheet conflicts itself, also says enables computation of differential pressure output)
static const uint8_t LPS25HB_CTRL_REG1_BDU_EN        = 0x04;  // Sets Block Data Update so output registers not updated until MSB and LSB have been read (otherwise in continuous update)
static const uint8_t LPS25HB_CTRL_REG1_RESET_AZ      = 0x02;  // Reset Autozero function
static const uint8_t LPS25HB_CTRL_REG1_SIM_4W        = 0x00;  // Select Interface Method : 4 Wire
static const uint8_t LPS25HB_CTRL_REG1_SIM_3W        = 0x01;  // Select Interface Method : 3 Wire

// Control Register 2
static const uint8_t LPS25HB_CTRL_REG2_DEFAULT       = 0x00;  // Default
static const uint8_t LPS25HB_CTRL_REG2_REBOOT_MEM    = 0x80;  // Reboots memory content, bit is self-cleared when BOOT complete
static const uint8_t LPS25HB_CTRL_REG2_FIFO_EN       = 0x40;  // Enable FIFO
static const uint8_t LPS25HB_CTRL_REG2_STOP_ON_FTH   = 0x20;  // Enable the FTH_FIFO bit in FIFO_STATUS for monitoring of FIFO level
static const uint8_t LPS25HB_CTRL_REG2_FIFO_MEAN_DEC = 0x10;  // Enable to deimate the output pressure to 1Hz with FIFO Mean mode
static const uint8_t LPS25HB_CTRL_REG2_I2C_DIS       = 0x08;  // Disable I2C interface
static const uint8_t LPS25HB_CTRL_REG2_SWRESET       = 0x04;  // Perform software reset
static const uint8_t LPS25HB_CTRL_REG2_AUTOZERO      = 0x02;  // Enable autozero
static const uint8_t LPS25HB_CTRL_REG2_ONE_SHOT      = 0x01;  // Acquire a new dataset

// Control Register 3
static const uint8_t LPS25HB_CTRL_REG3_DEFAULT       = 0x00;	// Default
static const uint8_t LPS25HB_CTRL_REG3_INT_H         = 0x00;  // Interrupts are active high
static const uint8_t LPS25HB_CTRL_REG3_INT_L         = 0x80;  // Interrupts are active low
static const uint8_t LPS25HB_CTRL_REG3_PP            = 0x00;  // Interrupt pads are Push/Pull
static const uint8_t LPS25HB_CTRL_REG3_OD            = 0x40;  // Interrupt pads are Open Drain
static const uint8_t LPS25HB_CTRL_REG3_INT_S_DATA    = 0x00;  // Data signal on interrupt
static const uint8_t LPS25HB_CTRL_REG3_INT_S_P_H     = 0x01;  // Interrupt on pressure high
static const uint8_t LPS25HB_CTRL_REG3_INT_S_P_L     = 0x02;  // Interrupt on pressure low
static const uint8_t LPS25HB_CTRL_REG3_INT_S_P_E     = 0x03;  // Interrupt on pressure either high or low

// Control Register 4
static const uint8_t LPS25HB_CTRL_REG4_DEFAULT       = 0x00;  // Default
static const uint8_t LPS25HB_CTRL_REG4_F_EMPTY       = 0x08;  // Enable FIFO empty flag on INT_DRDY pin
static const uint8_t LPS25HB_CTRL_REG4_F_FTH         = 0x04;  // Enable FIFO threshold (watermark) status on INT_DRDY pin to indicate that FIFO is filled up to the threshold level
static const uint8_t LPS25HB_CTRL_REG4_F_OVR         = 0x02;  // Enable FIFO overrun interrupt on INT_DRDY pin to indicate that FIFO is full in FIFO mode or that an overrun occurred in Stream mode
static const uint8_t LPS25HB_CTRL_REG4_DRDY          = 0x01;	// Enable Data-ready signal on INT_DRDY pin

// Interrupt Configuration
static const uint8_t LPS25HB_INTERRUPT_CFG_DEFAULT   = 0x00;  // Default
static const uint8_t LPS25HB_INTERRUPT_CFG_LIR       = 0x04;	// Latch interrupt request
static const uint8_t LPS25HB_INTERRUPT_CFG_PL_E      = 0x02;	// Enable interrupt generation on differential pressure low event
static const uint8_t LPS25HB_INTERRUPT_CFG_PH_E      = 0x01;	// Enable interrupt generation on differential pressure high event

// FIFO Control
static const uint8_t LPS25HB_FIFO_CTRL_DEFAULT       = 0x00;  // Default
static const uint8_t LPS25HB_FIFO_CTRL_BYPASS        = 0x00;  // Bypass FIFO
static const uint8_t LPS25HB_FIFO_CTRL_FIFO          = 0x20;	// Use FIFO
static const uint8_t LPS25HB_FIFO_CTRL_STREAM        = 0x40;  // Stream mode
static const uint8_t LPS25HB_FIFO_CTRL_SF            = 0x60;  // Stream to FIFO mode
static const uint8_t LPS25HB_FIFO_CTRL_BS            = 0x80;  // Bypass to stream mode
static const uint8_t LPS25HB_FIFO_CTRL_MEAN          = 0xC0;	// FIFO Mean mode
static const uint8_t LPS25HB_FIFO_CTRL_BF            = 0xE0;  // Bypass to FIFO mode
static const uint8_t LPS25HB_FIFO_CTRL_M_2           = 0x01;  // 2 sample moving average
static const uint8_t LPS25HB_FIFO_CTRL_M_4           = 0x03;  // 4 sample moving average
static const uint8_t LPS25HB_FIFO_CTRL_M_8           = 0x07;  // 8 sample moving average
static const uint8_t LPS25HB_FIFO_CTRL_M_16          = 0x0F;	// 16 sample moving average
static const uint8_t LPS25HB_FIFO_CTRL_M_32          = 0x1F;	// 32 sample moving average

static const uint8_t TEMPERATURE_READY               = 0x01;
static const uint8_t PRESSURE_READY                  = 0x02;	

void LPS25HBComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LPS25HB...");

  uint8_t device_id = 0x00;
  if (this->read_register((LPS25HB_REG_WHO_AM_I | (1 << 7)), &device_id, 1, false) != i2c::ERROR_OK) {
    this->error_code_ = READ_REGISTER_FAILED;
    this->mark_failed();
    return;
  }
  if (device_id != LPS25HB_DEVID) {
    this->error_code_ = WRONG_DEVICE_ID;
    this->mark_failed();
    return;
  }

  uint8_t settings[5];

	settings[0] = LPS25HB_RES_CONF_DEFAULT; // configure resolution to default
	if (this->write_register((LPS25HB_REG_RES_CONF | (1 << 7)), settings, 1) != i2c::ERROR_OK) {
    this->error_code_ = WRITE_REGISTER_FAILED;
    this->mark_failed();
    return;
  }
  
	settings[0] = LPS25HB_CTRL_REG1_PD_ACTIVE | LPS25HB_CTRL_REG1_ODR_25HZ; // Turn the sensor ON and set output rate to 25 Hz
	settings[1] = LPS25HB_CTRL_REG2_DEFAULT;								                // Default
	settings[2] = LPS25HB_CTRL_REG3_INT_L | LPS25HB_CTRL_REG3_OD;			      // Set interrupts to output LOW and Open Drain function
	settings[3] = LPS25HB_CTRL_REG4_DEFAULT;								                // Default
	settings[4] = LPS25HB_INTERRUPT_CFG_DEFAULT;    					              // Set the Interrupt CFG register to default

  if (this->write_register((LPS25HB_REG_CTRL_REG1 | (1 << 7)), settings, 5) != i2c::ERROR_OK) {
    this->error_code_ = WRITE_CONTROLREG1_FAILED;
    this->mark_failed();
    return;
  }
}

void LPS25HBComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LPS25HB:");

  switch (this->error_code_) {
    case READ_REGISTER_FAILED:
      ESP_LOGE(TAG, "  Reading register failed");
      break;
    case WRONG_DEVICE_ID:
      ESP_LOGE(TAG, "  Invalid Device ID - not LSP25HB");
      break;
    case WRITE_REGISTER_FAILED:
      ESP_LOGE(TAG, "  Writing register failed");
      break;
    case WRITE_CONTROLREG1_FAILED:
      ESP_LOGE(TAG, "  Writing Control Register 1 failed");
      break;
    case NONE:
      ESP_LOGI(TAG, "  Setup successful");
      break;
  }
  
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
}

void LPS25HBComponent::update() {
  this->running_update_ = true;
  this->status_clear_warning();
  if (this->temperature_sensor_!= nullptr) {
    ESP_LOGD(TAG, "'%s': new reading=%.2f°C", this->temperature_sensor_->get_name().c_str(), temperature_reading_);
    this->temperature_sensor_->publish_state(temperature_reading_);
  }
  if (this->pressure_sensor_ != nullptr) {
    ESP_LOGD(TAG, "'%s': new reading=%.1fhPa", this->pressure_sensor_->get_name().c_str(), pressure_reading_);
    this->pressure_sensor_->publish_state(pressure_reading_);
  }
  this->running_update_ = false;
}

void LPS25HBComponent::loop() {
  // only run loop if not updating and not failled
  if (this->running_update_ || this->is_failed()) {
    return;
  }
  
  if (new_temperature_reading_ready()) {
    read_temperature();
  }

  if (new_pressure_reading_ready()) {
    read_pressure();
  }
}

float LPS25HBComponent::get_setup_priority() const { return setup_priority::DATA; }

uint8_t LPS25HBComponent::get_status() {
	uint8_t status = 0x00;
  if (this->read_register((LPS25HB_REG_STATUS_REG | (1 << 7)), &status, 1, false) != i2c::ERROR_OK) {
    this->error_code_ = READ_REGISTER_FAILED;
    this->mark_failed();
    return status;
  }
}

bool LPS25HBComponent::new_temperature_reading_ready() {
  return ((get_status() & TEMPERATURE_READY) == TEMPERATURE_READY);
}

bool LPS25HBComponent::new_pressure_reading_ready() {
  return ((get_status() & PRESSURE_READY) == PRESSURE_READY);
}

/*
	NOTE! 	
  The LPS25HB datasheet does not specify the 42.5 degrees C offset,
  however the LPS25H datasheet does and it works for 
	the LPS25HB as well. Thanks to the Pololu LPS25H library for
	illuminating this problem.
*/

void LPS25HBComponent::read_temperature() {
  uint8_t buffer[2];
  int16_t raw_temperature = 0;
  if (this->read_register((LPS25HB_REG_TEMP_OUT_L | (1 << 7)), buffer, 2, false) != i2c::ERROR_OK ) {
    ESP_LOGW(TAG, "Error reading temperature");
    this->status_set_warning();
    return;
  }
  raw_temperature = (buffer[1] << 8 | buffer[0]);
  this->temperature_reading_ = (float)(42.5 + (raw_temperature / 480.0));
}

void LPS25HBComponent::read_pressure() {
	uint8_t buffer[3];															   
  if (this->read_register((LPS25HB_REG_PRESS_OUT_XL | (1 << 7)), buffer, 3, false) != i2c::ERROR_OK ) {
    ESP_LOGW(TAG, "Error reading temperature");
    this->status_set_warning();
    return;
  }

  int32_t raw_pressure = 0;
	raw_pressure = ((int32_t)buffer[0] << 0) | ((int32_t)buffer[1] << 8) | ((int32_t)buffer[2] << 16); 
	if (buffer[2] & 0x80) {
		raw_pressure |= 0xFF000000;  // for proper 2's complement behaviour, conditionally set the highest byte
	} 
	this->pressure_reading_ = (float)(raw_pressure / 4096.0);
}

} // namespace lps25hb
} // namespace esphome
