#pragma once

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "../tas58xx.h"

namespace esphome::tas58xx {

/// Input mixer mode. The options are the mixer modes from first_mode to MIXER_MODE_RIGHT: all modes for BTL, mono,
/// left and right for PBTL. With restore_value the selected mode is restored after a reboot.
class MixerModeSelect final : public select::Select, public Component, public Parented<TAS58xx> {
 public:
  MixerModeSelect(MixerMode first_mode, bool restore_value) : first_mode_(first_mode) {}

  void setup() override;

 protected:
  void control(size_t index) override;

  MixerMode first_mode_;
};

}  // namespace esphome::tas58xx
