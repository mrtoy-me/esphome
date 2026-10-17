#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include "../tas58xx.h"

namespace esphome::tas58xx {

/// Volume of one channel in dB, restored after a reboot. 0 dB on first boot.
class ChannelVolumeNumber final : public number::Number, public Component, public Parented<TAS58xx> {
 public:
  explicit ChannelVolumeNumber(Channel channel) : channel_(channel) {}

  void setup() override;
  void dump_config() override;

 protected:
  void control(float value) override;

  ESPPreferenceObject pref_;
  Channel channel_;
};

}  // namespace esphome::tas58xx
