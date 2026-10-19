# ESPHome dev changes affecting tas58xx

Working notes for branch `dev_eq_test` only. Do not include this file in any PR branch.

Reviews of `esphome/esphome` `dev` merges that affect ESPHome's own tas58xx, the planned EQ work (`dev-notes.md`,
`TAS58xx_EQ_Control_Proposal_v1.docx`) and the waiting branches `add_fault_output`, `add_modulation_option`,
`add_mixer_select` and `add_channel_volume` (all in `mrtoy-me/esphome`). Updated daily by a scheduled routine and in
interactive sessions; newest section at the bottom.

Last reviewed dev commit: dd456ea0b2

## Watching (open PRs that change tas58xx)
- #21083 (bdraco, open 2026-10-19): the startup sequences in `model_tas5805m.cpp` / `model_tas5825m.cpp` become
  `static constexpr i2c::I2CRegValue STARTUP_SEQUENCE[] PROGMEM`, `ModelInfo::startup_sequence` becomes
  `const i2c::I2CRegValue *`, and `init_()` writes it with one `write_registers()` call (uses #20945). Conflicts
  expected with `add_fault_output` (it edits both startup sequences and `ModelInfo`); `add_channel_volume` adds
  `ModelInfo` fields nearby. New startup or register-table code should use `I2CRegValue` / `write_registers()`.
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

## 0bb78186f2..dd456ea0b2 (reviewed 2026-10-18, routine)
Watched PRs #20864, #20912, #20913 have not merged yet; they stay under 'Watching'.
- #21018 [core] new `cg.add_pin(setter, pin_config)` and `cg.add_variable(setter, id)` (in `cpp_helpers`, exported by
  `codegen`). `add_pin` does nothing when the pin is not configured; `add_variable` looks up the id and calls the setter.
- #21019 [components] ESPHome's tas58xx `audio_dac.py` now ends with `await cg.add_pin(var.set_enable_pin,
  config.get(CONF_ENABLE_PIN))`, replacing the `if enable_pin_config := ...` block with `gpio_pin_expression`.
  - (a) Only change to tas58xx files in this range. No behaviour change.
  - (c) Likely rebase conflict at the end of `to_code()` in `audio_dac.py` for any waiting branch that edits the
    lines around `set_enable_pin` (add_fault_output most likely, as it adds a pin). Take the `cg.add_pin` form.
  - (d) New tas58xx code that sets an optional pin (eg a fault output pin) should use `cg.add_pin`, and one that
    sets a referenced component by id should use `cg.add_variable`.
- #21021 / #21023 [components] reference components are set with `cg.add_variable` / `register_parented`. The only
  file of interest is `i2c/__init__.py`: `register_i2c_device()` now uses `cg.add_variable(var.set_i2c_bus, ...)`.
  No change for tas58xx; no conflict expected.
- #21017 [core] `EntityBase::make_entity_preference<T>(version, in_flash)` overload: an entity can choose flash or RTC
  storage. Not used by tas58xx; relevant only if EQ settings are ever stored per entity (the plan does not).
- #21003 [core] AGENTS.md: a hub's entities go in platform files (`sensor.py`, ...) taking the hub's id, not under the
  hub's own key. New EQ entities (number/select/switch) already follow this as platforms; no action.
- Checked, not relevant: #21077 api, #21026 / #21025 BLE and modbus helpers, #21029 / #21030 / #21071 / #21035
  uart `discard_available`, #20994 bme68x_bsec2, #15348 / #20787 / #11354 light and homeassistant, #20790
  `progmem_string_table_find()` (core helper, no tas58xx use today), #21016 `encode_uint24`, #20939 little-endian I2C
  helpers, #21015 `cv.uint8_t` / `cv.uint16_t` (could replace `cv.int_range` over a full byte range in new
  validators; optional).
