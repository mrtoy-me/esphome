/*
  ESPHome LPS25HB external component by @mrtoy-me, 2024
  Sparkfun LPS25HB Arduino library was referenced and adapted in 
  developing this component. 

  The Sparkfun LPS25HB Arduino library was released under the GPLv3 license.
*/


#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace lps25hb {

class LPS25HBComponent : public PollingComponent, public i2c::I2CDevice, public sensor::Sensor {
 public:
   void set_temperature_sensor(sensor::Sensor *temperature) { temperature_sensor_ = temperature; }
   void set_pressure_sensor(sensor::Sensor *pressure) { pressure_sensor_ = pressure; }
   void set_pressure_correction(float correction) { pressure_correction_ = correction; }

   void setup() override;
   void dump_config() override;
   void update() override;
   //void loop() override;
   float get_setup_priority() const override;

 protected:
   enum Registers {
	   LPS25HB_REG_REF_P_XL = 0x08,
	   LPS25HB_REG_REF_P_L,
	   LPS25HB_REG_REF_P_H,

	   LPS25HB_REG_WHO_AM_I = 0x0F,

	   LPS25HB_REG_RES_CONF = 0x10,

	   // Reserved 0x11-0x1F

	   LPS25HB_REG_CTRL_REG1 = 0x20,
	   LPS25HB_REG_CTRL_REG2,
	   LPS25HB_REG_CTRL_REG3,
	   LPS25HB_REG_CTRL_REG4,
	   LPS25HB_REG_INTERRUPT_CFG,
	   LPS25HB_REG_INT_SOURCE,

	   // Reserved 0x26

	   LPS25HB_REG_STATUS_REG = 0x27,
	   LPS25HB_REG_PRESS_OUT_XL,
	   LPS25HB_REG_PRESS_OUT_L,
	   LPS25HB_REG_PRESS_OUT_H,
	   LPS25HB_REG_TEMP_OUT_L,
	   LPS25HB_REG_TEMP_OUT_H,

	   // Reserved 0x2D

	   LPS25HB_REG_FIFO_CTRL = 0x2E,
	   LPS25HB_REG_FIFO_STATUS,
	   LPS25HB_REG_THS_P_L,
	   LPS25HB_REG_THS_P_H,

	   // Reserved 0x32-38

	   LPS25HB_REG_RPDS_L = 0x39,
     LPS25HB_REG_RPDS_H,
   };

   enum ErrorCode {
     NONE = 0,
     READ_REGISTER_FAILED,
     WRONG_DEVICE_ID,
     WRITE_REGISTER_FAILED,
     WRITE_CONTROLREG1_FAILED,
   } error_code_{NONE};

   uint8_t get_status();
   bool new_temperature_reading_ready();
   bool new_pressure_reading_ready();
   void read_temperature();
   void read_pressure();
   //bool set_pressure_offset(int16_t offset);
   //bool get_pressure_offset(int16_t *offset);

   bool running_update_{false};
   float temperature_reading_{0};
   float pressure_reading_{0};
   float pressure_correction_{0.0};

   // sensors for humidity and temperature
   sensor::Sensor *temperature_sensor_{nullptr};
   sensor::Sensor *pressure_sensor_{nullptr};
};

}  // namespace lps25hb
}  // namespace esphome
