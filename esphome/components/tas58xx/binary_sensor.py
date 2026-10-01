import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_PROBLEM, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.types import ConfigType

from .audio_dac import CONF_TAS58XX_ID, TAS58xx, tas58xx_ns

CONF_HAVE_FAULT = "have_fault"

# FAULT SENSOR superset includes extra tas582x warning bitss
FAULT_SENSORS = (
    "left_channel_dc_fault",
    "right_channel_dc_fault",
    "left_channel_over_current",
    "right_channel_over_current",
    "otp_crc_check",
    "bq_write_failed",
    "load_eeprom_error",
    "clock_fault",
    "pvdd_over_voltage",
    "pvdd_under_voltage",
    "right_channel_cbc_over_current",
    "left_channel_cbc_over_current",
    "over_temp_shutdown",
    "left_channel_cbc_over_current_warning",
    "right_channel_cbc_over_current_warning",
    "over_temp_146c_warning",   # tas582x OTW Level 4
    "over_temp_warning",        # tas582x OTW Level 3 - keep tas5805 naming
    # "over_temp_122c_warning", # tas582x OTW Level 2 - not currently included
    # "over_temp_112c_warning", # tas582x OTW Level 1 - not currently included
)
FaultSensor = tas58xx_ns.enum("FaultSensor")

_FAULT_SCHEMA = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_PROBLEM,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TAS58XX_ID): cv.use_id(TAS58xx),
        cv.Optional(CONF_HAVE_FAULT): _FAULT_SCHEMA,
        **{cv.Optional(key): _FAULT_SCHEMA for key in FAULT_SENSORS},
    }
)


async def to_code(config: ConfigType) -> None:
    parent = await cg.get_variable(config[CONF_TAS58XX_ID])
    if sensor_config := config.get(CONF_HAVE_FAULT):
        sens = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(parent.set_have_fault_binary_sensor(sens))
    for key in FAULT_SENSORS:
        if sensor_config := config.get(key):
            sens = await binary_sensor.new_binary_sensor(sensor_config)
            fault = getattr(FaultSensor, f"FAULT_SENSOR_{key.upper()}")
            cg.add(parent.set_fault_binary_sensor(fault, sens))
