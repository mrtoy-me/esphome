#include "channel_volume_number.h"

#include "esphome/core/log.h"

namespace esphome::tas58xx {

ESPHOME_LOG_TAG(TAG, "tas58xx.number");

void ChannelVolumeNumber::setup() {
  float value = 0.0f;
  this->pref_ = this->make_entity_preference<float>();
  if (!this->pref_.load(&value) || !(value >= CHANNEL_VOLUME_MIN_DB && value <= CHANNEL_VOLUME_MAX_DB))
    value = 0.0f;
  this->publish_state(value);
  this->parent_->set_channel_volume(this->channel_, value);
}

void ChannelVolumeNumber::dump_config() {
  // LOG_NUMBER needs a string literal for the type, which stays in flash on ESP8266
  if (this->channel_ == CHANNEL_LEFT) {
    LOG_NUMBER("", "TAS58xx Left Channel Volume", this);
  } else {
    LOG_NUMBER("", "TAS58xx Right Channel Volume", this);
  }
}

void ChannelVolumeNumber::control(float value) {
  this->publish_state(value);
  this->pref_.save(&value);
  this->parent_->set_channel_volume(this->channel_, value);
}

}  // namespace esphome::tas58xx
