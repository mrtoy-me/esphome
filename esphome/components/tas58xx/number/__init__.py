import logging

import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG, UNIT_DECIBEL
import esphome.final_validate as fv
from esphome.types import ConfigType

from ..audio_dac import CONF_DAC_MODE, CONF_TAS58XX_ID, TAS58xx, tas58xx_ns

_LOGGER = logging.getLogger(__name__)

CONF_CHANNEL_VOLUME_LEFT = "channel_volume_left"
CONF_CHANNEL_VOLUME_RIGHT = "channel_volume_right"

ICON_VOLUME_SOURCE = "mdi:volume-source"

# Same as CHANNEL_VOLUME_MIN_DB and CHANNEL_VOLUME_MAX_DB in tas58xx.h
CHANNEL_VOLUME_MIN_DB = -24
CHANNEL_VOLUME_MAX_DB = 24
CHANNEL_VOLUME_STEP_DB = 1

ChannelVolumeNumber = tas58xx_ns.class_(
    "ChannelVolumeNumber", number.Number, cg.Component
)
Channel = tas58xx_ns.enum("Channel")
CHANNELS = {
    CONF_CHANNEL_VOLUME_LEFT: Channel.CHANNEL_LEFT,
    CONF_CHANNEL_VOLUME_RIGHT: Channel.CHANNEL_RIGHT,
}

_CHANNEL_VOLUME_SCHEMA = number.number_schema(
    ChannelVolumeNumber,
    entity_category=ENTITY_CATEGORY_CONFIG,
    icon=ICON_VOLUME_SOURCE,
    unit_of_measurement=UNIT_DECIBEL,
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TAS58XX_ID): cv.use_id(TAS58xx),
        cv.Optional(CONF_CHANNEL_VOLUME_LEFT): _CHANNEL_VOLUME_SCHEMA,
        cv.Optional(CONF_CHANNEL_VOLUME_RIGHT): _CHANNEL_VOLUME_SCHEMA,
    }
)


def _final_validate(config: ConfigType) -> ConfigType:
    fconf = fv.full_config.get()
    hub_path = fconf.get_path_for_id(config[CONF_TAS58XX_ID])
    hub_conf = fconf.get_config_for_path(hub_path[:-1])
    has_left = CONF_CHANNEL_VOLUME_LEFT in config
    has_right = CONF_CHANNEL_VOLUME_RIGHT in config

    if hub_conf[CONF_DAC_MODE] == "pbtl":
        # PBTL drives a single speaker from the left channel
        if has_right:
            _LOGGER.warning(
                "%s is not used when %s is 'pbtl' - it can be removed",
                CONF_CHANNEL_VOLUME_RIGHT,
                CONF_DAC_MODE,
            )
    elif has_left != has_right:
        raise cv.Invalid(
            f"{CONF_CHANNEL_VOLUME_LEFT} and {CONF_CHANNEL_VOLUME_RIGHT} must be used together "
            f"when {CONF_DAC_MODE} is 'btl'"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_TAS58XX_ID])
    for key, channel in CHANNELS.items():
        if volume_config := config.get(key):
            num = await number.new_number(
                volume_config,
                channel,
                min_value=CHANNEL_VOLUME_MIN_DB,
                max_value=CHANNEL_VOLUME_MAX_DB,
                step=CHANNEL_VOLUME_STEP_DB,
            )
            await cg.register_component(num, volume_config)
            await cg.register_parented(num, hub)
