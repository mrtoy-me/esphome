# tas58xx working branch to-do (dev_eq_test)

Working notes for this branch only. Do not include this file in any PR branch.

The EQ control proposal is `TAS58xx_EQ_Control_Proposal_v1.docx` and the notes and upstream PR plan are in
`dev-notes.md`, both in this folder. The proof of concept is `mrtoy-me/esphome-tas58xx` branch `dev_update_eq_proto`.

## To-do
- `volume_follows: <audio_dac id>` option: a DAC that no speaker drives (eg a second DAC on the same I2S output)
  copies volume and mute from the given DAC, at boot and on every change, so no media player `on_volume` /
  `on_mute` / `on_unmute` lambdas are needed. Until then the YAML workaround is those three triggers copying
  `id(dac1).volume()` and `is_muted()` to the second DAC.
- `FS_MON` (0x37) check: once audio starts, read the detected input sample rate and log a warning if it is not
  48 kHz or 96 kHz (eg an unexpected `sample_rate:` of 16k or 32k). It shows the I2S input rate, not the DSP
  processing rate, and cannot tell 44.1 kHz from 48 kHz (44.1k has no code and reads back as 48k), so 44.1k input
  still needs the acoustic check (narrow +12 dB peak at 1 kHz, measured). Codes: 48k `1001`, 96k `1011`.
