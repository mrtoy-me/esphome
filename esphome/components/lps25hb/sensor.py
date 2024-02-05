import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
  CONF_ID,
  CONF_PRESSURE,
  CONF_TEMPERATURE,
  DEVICE_CLASS_PRESSURE,
  DEVICE_CLASS_TEMPERATURE,
  STATE_CLASS_MEASUREMENT,
  UNIT_HECTOPASCAL,
  UNIT_CELSIUS,
)

CONF_PRESSURE_CORRECTION = "pressure_correction"

CODEOWNERS = ["@mrtoy-me"]

lps25hb_ns = cg.esphome_ns.namespace("lps25hb")
LPS25HBComponent = lps25hb_ns.class_(
    "LPS25HBComponent", cg.PollingComponent, i2c.I2CDevice, sensor.Sensor
)

CONFIG_SCHEMA = (
  cv.Schema(
    {
      cv.GenerateID(): cv.declare_id(LPS25HBComponent),
      cv.Optional(CONF_PRESSURE): sensor.sensor_schema(
        unit_of_measurement=UNIT_HECTOPASCAL,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_PRESSURE,
        state_class=STATE_CLASS_MEASUREMENT,
      ),
      cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
      ),
      cv.Optional(CONF_PRESSURE_CORRECTION): cv.float_range(min=-10.0, max=10.0),
    }
  )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x5D))
)

async def to_code(config):
  var = cg.new_Pvariable(config[CONF_ID])
  await cg.register_component(var, config)
  await i2c.register_i2c_device(var, config)
    
  if CONF_PRESSURE in config:
    sens = await sensor.new_sensor(config[CONF_PRESSURE])
    cg.add(var.set_pressure_sensor(sens))

  if CONF_TEMPERATURE in config:
    sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
    cg.add(var.set_temperature_sensor(sens))

  if CONF_PRESSURE_OFFSET in config:
    cg.add(var.set_pressure_correction(config[CONF_PRESSURE_CORRECTION]))

