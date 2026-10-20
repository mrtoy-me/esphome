# ESPHome dev changes affecting tas58xx

Working notes for branch `dev_eq_test` only. Do not include this file in any PR branch.

Reviews of `esphome/esphome` `dev` merges that affect ESPHome's own tas58xx, the planned EQ work (`dev-notes.md`,
`TAS58xx_EQ_Control_Proposal_v1.docx`) and the waiting branches `add_fault_output`, `add_modulation_option`,
`add_mixer_select` and `add_channel_volume` (all in `mrtoy-me/esphome`). Updated daily by a scheduled routine and in
interactive sessions; newest section at the bottom.

Last reviewed dev commit: bac4d2aa2c

## Watching (open PRs that change tas58xx)
- #21083 (bdraco, open 2026-10-19): the startup sequences in `model_tas5805m.cpp` / `model_tas5825m.cpp` become
  `static constexpr i2c::I2CRegValue STARTUP_SEQUENCE[] PROGMEM`, `ModelInfo::startup_sequence` becomes
  `const i2c::I2CRegValue *`, and `init_()` writes it with one `write_registers()` call (uses #20945). Conflicts
  expected with `add_fault_output` (it edits both startup sequences and `ModelInfo`); `add_channel_volume` adds
  `ModelInfo` fields nearby. New startup or register-table code should use `I2CRegValue` / `write_registers()`.
- #20913 uses `get_declaring_config()` (#20912, merged) in tas58xx `binary_sensor.py`; not merged yet. Our EQ final
  validation can already use `get_declaring_config()` instead of the hand-written parent config lookup.

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

## dd456ea0b2..65ba64b9a6 (reviewed 2026-10-19, routine)
Merged since the last review: #20864 and #20912, so both leave 'Watching'. #21083 and #20913 are still open.
- #20864 [components] `TAS58xx::update()` now uses `status_comm_result(this->read_faults_() &&
  this->read_byte(TAS58XX_POWER_STATE, &power_state), LOG_STR("Failed to read status"))`; `power_state` starts at 0.
  - (a) No behaviour change. (c) Conflict only if a waiting branch edits `update()`: `add_fault_output` may, when it
    reads the fault pin in `update()`. Take the `status_comm_result` form. (d) Use it for any new I2C failure path.
- #20912 [core] `final_validate.get_declaring_config(id_, config=None)` returns the config block that declares an id
  (KeyError if undeclared, TypeError if not a dict). (b) EQ final validation should use it in place of
  `get_path_for_id()` plus `get_config_for_path()`. (c) Our `binary_sensor.py` still has the hand-written lookup
  until #20913 merges; it is the same code the branches touch, so expect a small conflict there.
- #20935 [core] Apply entities: `sub_numbers` / `sub_switches` / `sub_selects` / `sub_buttons` accept `apply=` (a C++
  statement run on the hub, with `entity` plus `value` / `state` / `index` / `option` available) and `publish=`
  ("before" or "after"), and then use a class-free `ApplyNumber` / `ApplySwitch` / `ApplySelect` / `ApplyButton`.
  The entity needs `parent=` set (a hub). (a) tas58xx has no number, select or switch today, so nothing changes.
  (b) The planned EQ entities (gain numbers, mode select, EQ switch) fit this shape when they only forward to the
  hub; this would remove the per-entity C++ classes. (c) `add_mixer_select` and `add_channel_volume` could use it
  too, but it is optional. (d) Prefer `apply=` over a new entity class when the entity only calls one hub method.
- #21129 [const] `CONF_SOURCES` moved to `components/const`. No effect.
- Checked, not relevant: #20866 shared expander pin class and pin helpers, #20925 `SensorWithDedup`, #21052 Python
  3.15, #21137 and other dependency bumps, and the display, ld24xx, usb_audio, globals, web_server and api merges.

## 65ba64b9a6..bac4d2aa2c (reviewed 2026-10-20, routine)
No tas58xx files changed. #21083 and #20913 are not merged yet; they stay under 'Watching'.
- #21180 [core] new `cg.add_optional(setter, value)`: calls the setter with the value and returns it; does nothing for
  `None`. #21181 / #21183 apply it across components (the only file of interest is `i2c/__init__.py`, pullup options;
  tas58xx is not touched). (a) No change. (c) No conflict expected. (d) New tas58xx code that sets an optional plain
  config value should use `cg.add_optional(var.set_x, config.get(CONF_X))` instead of an `if CONF_X in config` block
  (for example the fault output, modulation or channel volume options). Pins still use `cg.add_pin`.
- #21130 [core] AGENTS.md now lists the codegen and bus helpers (`update_register`, `write_registers`, `check_chip_id`,
  `status_comm_result`, `get_declaring_config`, `MyEnum.options`, `new_i2c_component`) and the Apply entities
  (`apply=`, `publish="before"` or `"after"`). (b) This supports the EQ plan: use Apply entities for EQ numbers,
  selects and switches that only call the hub. No new action beyond the existing to-do items.
- #21146, #21147, #21148, #21162, #21163 (ld2420, ld2450, dfrobot_sen0609, airton, haier_tundra, tm1638, bl0940,
  micronova, atm90e32, copy) now use Apply numbers, selects, switches and buttons. They are working in-tree examples
  of the `apply=` form for the planned EQ entities.
- Checked, not relevant: #21176 flake8 replaced by ruff docstring rules (Python docstrings in new tas58xx Python may
  be checked by ruff), #21117 / #21118 core logging from flash, #21152 / #21150 and other test helpers, IR frame byte
  helpers (remote_base and climate IR merges), #21202 sx126x, #21174 sx126x / sx127x constant, #17744 nextion,
  #21145 ld2410, #15608 st25r, #21172 st7305, #21178 ota, dependency bumps.
