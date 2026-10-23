#include "mixer_mode_select.h"

#include "esphome/core/log.h"

namespace esphome::tas58xx {

ESPHOME_LOG_TAG(TAG, "tas58xx.select");

void MixerModeSelect::setup() {
  // Starts at the audio_dac mixer_mode. With restore_value the saved mode is used instead, if there is one. The saved
  // mode is tied to the YAML dac_mode through the preference version (first_mode_ is 0 for BTL and 2 for PBTL), so
  // after dac_mode is changed in YAML a mode saved with the other dac_mode is not used.
  uint8_t mode = this->parent_->get_mixer_mode();
  if (this->restore_value_) {
    uint8_t saved{0};
    this->pref_ = this->make_entity_preference<uint8_t>(this->first_mode_);
    if (this->pref_.load(&saved) && saved >= this->first_mode_ && saved <= MIXER_MODE_RIGHT)
      mode = saved;
  }
  this->publish_state(mode - this->first_mode_);
  this->parent_->apply_mixer_mode(static_cast<MixerMode>(mode));
}

void MixerModeSelect::dump_config() { LOG_SELECT("", "TAS58xx Mixer Mode", this); }

void MixerModeSelect::control(size_t index) {
  const uint8_t mode = this->first_mode_ + index;
  this->publish_state(index);
  if (this->restore_value_)
    this->pref_.save(&mode);
  this->parent_->apply_mixer_mode(static_cast<MixerMode>(mode));
}

}  // namespace esphome::tas58xx
