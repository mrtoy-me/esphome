import esphome.codegen as cg
from esphome.components import i2c,sensor

CODEOWNERS = ["@mrtoy-me"]
DEPENDENCIES = ["i2c","sensor"]

ms5637_ns = cg.esphome_ns.namespace("ms5637")
MS5637Resolution = ms5637_ns.enum("MS5637Resolution")
MS5637Component = ms5637_ns.class_(
    "MS5637Component", cg.PollingComponent, i2c.I2CDevice, sensor.Sensor
)