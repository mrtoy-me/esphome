/*
  The MS5637 library originally written by TEConnectivity with MIT license.
  This library was updated by Nathan Seidle @ SparkFun Electronics, 2018.

  The MS5637 Sparkfun Arduino library has been adapted and modified 
  so it functions under the ESPHome component framework by @mrtoy-me, 2023 
  
  MIT License
  Copyright (c) 2016 TE Connectivity

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ms5637 {

// ms conversion time for each resolution
static const uint8_t CONVERSION_TIME_OSR_256  = 1;
static const uint8_t CONVERSION_TIME_OSR_512  = 2;
static const uint8_t CONVERSION_TIME_OSR_1024 = 3;
static const uint8_t CONVERSION_TIME_OSR_2048 = 5;
static const uint8_t CONVERSION_TIME_OSR_4096 = 9;
static const uint8_t CONVERSION_TIME_OSR_8192 = 17;

enum MS5637Resolution {
  OSR_256 = 0,
  OSR_512,
  OSR_1024,
  OSR_2048,
  OSR_4096,
  OSR_8192
};

class MS5637Component : public PollingComponent, public i2c::I2CDevice, public sensor::Sensor {
 public:
   void set_temperature_sensor(sensor::Sensor *temperature) { temperature_sensor_ = temperature; }
   void set_pressure_sensor(sensor::Sensor *pressure) { pressure_sensor_ = pressure; }
   void set_resolution(MS5637Resolution resolution) { resolution_osr_ = resolution; }

   void setup() override;
   void dump_config() override;
   void update() override;
   float get_setup_priority() const override;

 protected:
   bool crc_check(uint16_t *n_prom, uint8_t crc);
   void start_conversions();
   void read_temperature();
   void do_pressure_conversion();
   void read_pressure_and_publish();
   bool calculate_temperature_and_pressure();
	
   uint16_t eeprom_coeff_[8];

   uint8_t conversion_time_[6] = {
     CONVERSION_TIME_OSR_256,  CONVERSION_TIME_OSR_512,
     CONVERSION_TIME_OSR_1024, CONVERSION_TIME_OSR_2048,
     CONVERSION_TIME_OSR_4096, CONVERSION_TIME_OSR_8192 };

   MS5637Resolution resolution_osr_;
   uint8_t conversion_time_osr_;
   uint32_t adc_temperature_;
   uint32_t adc_pressure_;
	 float temperature_reading_;
   float pressure_reading_;

   // sensors for humidity and temperature 
   sensor::Sensor *temperature_sensor_{nullptr};
   sensor::Sensor *pressure_sensor_{nullptr}; 
    
   enum ErrorCode {
     NONE = 0,
     COMMUNICATION_FAILED,
     RESET_FAILED,
     EEPROM_READ_FAILED,
     EEPROM_CRC_FAILED,
   } error_code_{NONE};

};

}  // namespace ms5637
}  // namespace esphome
