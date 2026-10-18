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
- `FS_MON` (0x37) check: once audio starts, read the detected input sample rate and log a warning if it is not
  48 kHz or 96 kHz (eg an unexpected `sample_rate:` of 16k or 32k). It shows the I2S input rate, not the DSP
  processing rate, and cannot tell 44.1 kHz from 48 kHz (44.1k has no code and reads back as 48k), so 44.1k input
  still needs the acoustic check (narrow +12 dB peak at 1 kHz, measured). Codes: 48k `1001`, 96k `1011`.
