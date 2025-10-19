#include "hdc1080.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace hdc1080 {

static const char *const TAG = "hdc1080";

static const uint8_t HDC1080_CMD_CONFIGURATION = 0x02;
static const uint8_t HDC1080_CMD_TEMPERATURE = 0x00;
static const uint8_t HDC1080_CMD_HUMIDITY = 0x01;

void HDC1080Component::setup() {
  

  // delay for sensor to be ready
  this->set_timeout(20, [this]() {
    //const uint8_t config[2] = {0x01,0x00};
    //write_register(HDC1080_CMD_CONFIGURATION, config, 2);

    // get boot config
    if (this->write(&HDC1080_CMD_CONFIGURATION, 1) != i2c::ERROR_OK) {
      this->status_set_warning();
      ESP_LOGW(TAG, "Error writing config register");
      this->setup_complete_= true;
      return;
    }
      
    //this->set_timeout(20, [this]() {
      if (this->read(this->boot_config_, 2) != i2c::ERROR_OK) {
        this->status_set_warning();
        ESP_LOGW(TAG, "Error reading configuration");
      }
      this->setup_complete_= true;
    //});
  });
}

void HDC1080Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HDC1080:");
  ESP_LOGCONFIG(TAG, "  Config_b: 0x%02X%02X", this->boot_config_[0],this->boot_config_[1]);
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this->temperature_);
  LOG_SENSOR("  ", "Humidity", this->humidity_);
}

void HDC1080Component::update() {
  // regardless of what sensor/s are defined in yaml configuration
  // the hdc1080 setup configuration used, requires both temperature and humidity to be read
  if(!this->setup_complete_) return;

  this->status_clear_warning();

  if (this->write(&HDC1080_CMD_TEMPERATURE, 1) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }

  this->set_timeout(20, [this]() {
    uint16_t raw_temperature;
    if (this->read(reinterpret_cast<uint8_t *>(&raw_temperature), 2) != i2c::ERROR_OK) {
      this->status_set_warning();
      return;
    }

    if (this->temperature_ != nullptr) {
      raw_temperature = i2c::i2ctohs(raw_temperature);
      float temperature = raw_temperature * 0.0025177f - 40.0f;  // raw * 2^-16 * 165 - 40
      this->temperature_->publish_state(temperature);
    }

    if (this->write(&HDC1080_CMD_HUMIDITY, 1) != i2c::ERROR_OK) {
      this->status_set_warning();
      return;
    }

    this->set_timeout(20, [this]() {
      uint16_t raw_humidity;
      if (this->read(reinterpret_cast<uint8_t *>(&raw_humidity), 2) != i2c::ERROR_OK) {
        this->status_set_warning();
        return;
      }

      if (this->humidity_ != nullptr) {
        raw_humidity = i2c::i2ctohs(raw_humidity);
        float humidity = raw_humidity * 0.001525879f;  // raw * 2^-16 * 100
        this->humidity_->publish_state(humidity);
      }
    });
  });
}

}  // namespace hdc1080
}  // namespace esphome
