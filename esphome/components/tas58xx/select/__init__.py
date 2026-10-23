import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_RESTORE_VALUE, ENTITY_CATEGORY_CONFIG
from esphome.core import CORE
import esphome.final_validate as fv
from esphome.types import ConfigType

from .. import DOMAIN
from ..audio_dac import (
    CONF_DAC_MODE,
    CONF_MIXER_MODE,
    CONF_TAS58XX_ID,
    MIXER_MODES,
    TAS58xx,
    tas58xx_ns,
)

ICON_SPEAKER_MULTIPLE = "mdi:speaker-multiple"

MixerModeSelect = tas58xx_ns.class_("MixerModeSelect", select.Select, cg.Component)

# Option text of each mixer mode, in MixerMode order
MIXER_MODE_OPTIONS = {
    "stereo": "Stereo",
    "stereo_inverse": "Stereo inverse",
    "mono": "Mono",
    "left": "Left",
    "right": "Right",
}
# PBTL drives a single speaker, so the stereo modes are not offered
PBTL_MIXER_MODES = ("mono", "left", "right")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TAS58XX_ID): cv.use_id(TAS58xx),
        cv.Optional(CONF_MIXER_MODE): select.select_schema(
            MixerModeSelect,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_SPEAKER_MULTIPLE,
        )
        .extend(
            {
                # Off: every boot starts at the audio_dac mixer_mode. On: the last selected mode is restored
                cv.Optional(CONF_RESTORE_VALUE, default=False): cv.boolean,
            }
        )
        .extend(cv.COMPONENT_SCHEMA),
    }
)


def _final_validate(config: ConfigType) -> ConfigType:
    if mixer_config := config.get(CONF_MIXER_MODE):
        hub_conf = fv.get_declaring_config(config[CONF_TAS58XX_ID])
        # The options depend on the dac_mode of the audio_dac, saved for to_code
        modes = (
            PBTL_MIXER_MODES
            if hub_conf[CONF_DAC_MODE] == "pbtl"
            else tuple(MIXER_MODE_OPTIONS)
        )
        CORE.data.setdefault(DOMAIN, {})[str(mixer_config[CONF_ID])] = modes
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_TAS58XX_ID])
    selects = select.sub_selects(config, parent=hub)
    if mixer_config := config.get(CONF_MIXER_MODE):
        modes = CORE.data[DOMAIN][str(mixer_config[CONF_ID])]
        # The hub keeps no pointer: the select restores and applies the mode itself
        sel = await selects(
            CONF_MIXER_MODE,
            None,
            MIXER_MODES[modes[0]],
            mixer_config[CONF_RESTORE_VALUE],
            options=[MIXER_MODE_OPTIONS[mode] for mode in modes],
        )
        await cg.register_component(sel, mixer_config)
