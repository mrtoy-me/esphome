# tas58xx working branch to-do (dev_eq_test)

Working notes for this branch only. Do not include this file in any PR branch.

The EQ control proposal is `TAS58xx_EQ_Control_Proposal_v1.docx` and the notes and upstream PR plan are in
`dev-notes.md`, both in this folder. The proof of concept is `mrtoy-me/esphome-tas58xx` branch `dev_update_eq_proto`.

## To-do
- `volume_follows: <audio_dac id>` option: a DAC that no speaker drives (eg a second DAC on the same I2S output)
  copies volume and mute from the given DAC, at boot and on every change, so no media player `on_volume` /
  `on_mute` / `on_unmute` lambdas are needed. Until then the YAML workaround is those three triggers copying
  `id(dac1).volume()` and `is_muted()` to the second DAC.
  - Done in the proof of concept (`dev_update_eq_proto`, 2026-10-18): port that design (leader/follower list,
    volume, mute and audio start passed on, follower checks Play, guards in `add_volume_follower()`).
  - Logs: there a follower logs "Volume change / Mute / Unmute followed by 0x.." at DEBUG. When porting, decide
    whether those should be VERBOSE (the media player already logs each change at DEBUG).
- Skip unchanged volume writes in `set_volume()`: a `speaker_source` media player calls `set_volume()` on every
  pipeline's speaker, so with media + announcement pipelines on one I2S speaker `DIG_VOL` is written twice per
  change. Done in the proof of concept (`dev_update_eq_proto`, 2026-10-18: cached last DIG_VOL value). Could be its
  own small PR. Agreed form for ESPHome's tas58xx (uses the existing `previous`, no new member or helper):
  ```cpp
  bool TAS58xx::set_volume(float volume) {
    const float previous = this->volume_;
    this->volume_ = clamp(volume, 0.0f, 1.0f);
    // a speaker_source media player sets the volume on every pipeline's speaker, so the same volume can arrive
    // more than once; only write when it changes
    if (this->volume_ == previous)
      return true;
    if (!this->write_volume_()) {
      this->volume_ = previous;
      return false;
    }
    return true;
  }
  ```
  Exact float `==` is safe: the duplicates are the same `bounded_volume` value passed to each pipeline's speaker.
  A failed write restores `previous`, so the next call retries; `init_()` still writes DIG_VOL after reset.
  Changes too small to move the 0.5 dB step are still written (harmless).
  To look at when doing it: the proof of concept avoids those sub-step writes too, because it keeps the last raw
  DIG_VOL value written (`tas58xx_raw_volume_`, one byte) and compares register values instead of volumes. Decide
  between that (one byte, skips every unchanged register write) and the `previous` form above (no new member,
  simplest for review).
- Chebyshev low pass / high pass (2nd order, ripple in dB) as in TI PurePath Console 3: write an efficient C++
  version in the style of the biquad helpers, compare its complexity and cost with Butterworth 2, then either add it
  as another filter type or shelve it. A one-off cloud run on 2026-10-19 (midnight Brisbane) writes the study to
  `chebyshev-study.md` in this folder (added 2026-10-18).
  - (2026-10-19) Added to `dev_update_eq_proto` as `FILTER_TYPE_LOW_PASS_CHEBYSHEV` / `FILTER_TYPE_HIGH_PASS_CHEBYSHEV`,
    not yet called. `chebyshev_filter()` takes the ripple as an argument; `make_biquad()` passes the constant
    `CHEBYSHEV_RIPPLE_DB` (0.5), so Q is fixed as for low and high pass. A YAML `ripple:` option later would pass a
    value instead of the constant.
  - Gain limits are the same as the Butterworth filters: low pass -24 to +24 dB, high pass -24 to +18 dB (largest
    coefficient 14.99 at 10 Hz, +18 dB). `chebyshev_filter()` guards its own gain (as every filter does since
    2026-10-19, with a warning when it limits). The YAML band range is -24 to +18 dB for every filter, so adding the
    YAML names needs no gain-limit changes in Python.
- EQ band gain limits (`add_eq_gain_limits`, merged into `dev_update_eq_proto` 2026-10-19): `gain_min` / `gain_max` at
  platform and band level, -24 to +18 dB for every filter (a design choice; each C++ filter guards its own technical
  limit, -24 to +24 dB, high shelf / high pass +18 dB). A saved gain outside the range is moved to the nearest limit
  at boot, with a warning, and saved. `dump_config` shows each band's gain range. Carry into the upstream EQ PRs.
  - (routine 2026-10-19) Study done: `chebyshev-study.md`. Recommendation: include (similar cost to Butterworth 2; C++ matches
    the reference with 0 LSB and the main branch presets are Chebyshev 0.5 dB).
- `FS_MON` (0x37) check: once audio starts, read the detected input sample rate and log a warning if it is not
  48 kHz or 96 kHz (eg an unexpected `sample_rate:` of 16k or 32k). It shows the I2S input rate, not the DSP
  processing rate, and cannot tell 44.1 kHz from 48 kHz (44.1k has no code and reads back as 48k), so 44.1k input
  still needs the acoustic check (narrow +12 dB peak at 1 kHz, measured). Codes: 48k `1001`, 96k `1011`.
- When rebasing add_fault_output (and any branch editing the end of `to_code()` in `audio_dac.py`), use
  `await cg.add_pin(var.set_enable_pin, config.get(CONF_ENABLE_PIN))` as in dev (#21019), and use `cg.add_pin` for the
  new fault pin too (routine 2026-10-18).
- TAS5805M DSP_MISC (0x66), decided 2026-10-19: do what PPC3 does, as for register 0x46. The EQ code writes
  0x86 (reserved bit 7, DRC and FIR bypassed, EQ on, ganged) with bit 0 for EQ bypass and bit 3 for unique left and
  right coefficients, ie `0x86 | bypass | unique`. `dev_update_eq_proto` writes 0x06 (no bit 7); it stays as is (frozen).
  Never write 0x66 on the TAS5825M (it is GPIO_OUT_INV there). Details in dev-notes.md, "PPC3 audio modes".
- Use `final_validate.get_declaring_config()` (#20912, merged) in the EQ final validation instead of
  `get_path_for_id()` (routine 2026-10-19).
- Check whether the EQ number, select and switch entities can use the Apply entities (#20935: `apply=` / `publish=`
  on `sub_numbers` etc., no per-entity class) before writing entity classes (routine 2026-10-19).
