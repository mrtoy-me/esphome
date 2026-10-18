# ESPHome dev changes affecting tas58xx

Working notes for branch `dev_eq_test` only. Do not include this file in any PR branch.

Reviews of `esphome/esphome` `dev` merges that affect ESPHome's own tas58xx, the planned EQ work (`dev-notes.md`,
`TAS58xx_EQ_Control_Proposal_v1.docx`) and the waiting branches `add_fault_output`, `add_modulation_option`,
`add_mixer_select` and `add_channel_volume` (all in `mrtoy-me/esphome`). Updated daily by a scheduled routine and in
interactive sessions; newest section at the bottom.

Last reviewed dev commit: 0bb78186f2

## Watching (open PRs that change tas58xx)
- #20864 (uses `status_comm_result` in tas58xx `update()`), stacked on #20863 (merged).
- #20912 adds `final_validate.get_declaring_config()`; #20913 uses it in tas58xx `binary_sensor.py`. Our EQ
  final validation should use it instead of the hand-written parent config lookup once merged.

## Up to b3f5789dc6 (since the 2026.10 beta, reviewed 2026-10-17)
- #20319 [tas58xx] TAS5805M `0x53=0x60`, TAS5825M GPIO1 as FAULTZ, startup comments. Baseline for the model files.
- #20889 [core] `cv.float_with_unit` parses "1.001kHz" as one decimal, so whole-Hz values written in kHz are exact.
- #20395 / #20419 [core] `DOMAIN` constant in every component; #20291 log tags via `ESPHOME_LOG_TAG`. New code must
  follow.
- #19609 [select], #19606 / #20433 triggers built with `build_callback_automation` / `build_trigger_callback`.
- #20362 / #20369 nameless `status_momentary_*` overloads; #20389 `alloc_helpers.h` no longer included by
  `helpers.h`; #20507 `sum_bytes()`.

## b3f5789dc6..d2a1f7ee31 (reviewed 2026-10-17)
- #20910 / #20911 `MockObjEnum.options(*names, prefix=...)`; upstream tas58xx uses it for `DAC_MODES` and
  `MIXER_MODES`. EQ filter maps should be built the same way, eg
  `FilterType.options("equalizer", ..., prefix="FILTER_TYPE_")`.
- #20941 [i2c] `I2CDevice::check_chip_id(component, reg, expected, mask)`. Possible later tas58xx PR (check the
  datasheets for an ID register first).
- #20931 `ESP_LOGV_HEX` / `ESP_LOGVV_HEX` / `ESP_LOGD_HEX`; #20907, #20921, #20933, #20961 not relevant.

## d2a1f7ee31..0bb78186f2 (reviewed 2026-10-18; dev merged into dev_eq_test at 0bb78186f2, clean)
- #20863 MERGED: `Component::status_comm_result(ok[, msg])`. Use it in new tas58xx code for I2C failures.
- #20909 changed upstream tas58xx `audio_dac.py` `to_code`: `var = await i2c.new_i2c_component(config,
  cg.RawExpression(f"&{model.model_info}"))` (helper from #20908). The four `add_*` branches need rebasing over it.
- New I2C helpers: #20945 `write_registers(const I2CRegValue (&table)[N])` for PROGMEM {reg, value} tables (could
  replace the startup sequence loop); #20854 `update_register(reg, mask, value)` read-modify-write (DEVICE_CTRL_1
  modulation and PBTL, AGAIN); #20938 little-endian and signed decode helpers.
- Conventions: #21014 do not override `get_setup_priority()` when it returns the default; #20874 core reports
  communication failures in `dump_config`; #20943 components use `check_chip_id`; #20932 `ESP_LOG*_HEX` for hex dumps.
