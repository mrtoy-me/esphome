#include "mixer_mode_select.h"

#include "esphome/core/log.h"

namespace esphome::tas58xx {

ESPHOME_LOG_TAG(TAG, "tas58xx.select");

void MixerModeSelect::setup() {
  // Starts at the audio_dac mixer_mode (first_mode_ is 0 for BTL and 2 for PBTL)
  uint8_t mode = this->parent_->get_mixer_mode();
  this->publish_state(mode - this->first_mode_);
  this->parent_->apply_mixer_mode(static_cast<MixerMode>(mode));
}

void MixerModeSelect::control(size_t index) {
  const uint8_t mode = this->first_mode_ + index;
  this->publish_state(index);
  this->parent_->apply_mixer_mode(static_cast<MixerMode>(mode));
}

}  // namespace esphome::tas58xx
