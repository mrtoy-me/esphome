"""Tests for the TAS58xx mixer mode select."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path


def test_mixer_select_options_follow_dac_mode(
    generate_main: Callable[[str | Path], str],
) -> None:
    """BTL offers every mixer mode, PBTL only mono, left and right, starting at its first mode.

    restore_value is off by default (mixer_btl) and passed on when set (mixer_pbtl).
    """
    main_cpp = generate_main(
        "tests/component_tests/tas58xx/test_tas58xx_mixer_select.yaml"
    )

    assert (
        'select_options[] PROGMEM = {"Stereo", "Stereo inverse", "Mono", "Left", "Right"};'
        in main_cpp
    )
    assert (
        "new(mixer_btl) tas58xx::MixerModeSelect(tas58xx::MIXER_MODE_STEREO, false);"
        in main_cpp
    )
    assert "mixer_btl->traits.set_options_static(select_options, 5);" in main_cpp
    assert 'select_options_2[] PROGMEM = {"Mono", "Left", "Right"};' in main_cpp
    assert (
        "new(mixer_pbtl) tas58xx::MixerModeSelect(tas58xx::MIXER_MODE_MONO, true);"
        in main_cpp
    )
    assert "mixer_pbtl->traits.set_options_static(select_options_2, 3);" in main_cpp


def test_mixer_select_is_given_its_amplifier(
    generate_main: Callable[[str | Path], str],
) -> None:
    """Each select restores and applies the mode through its own amplifier, as a component."""
    main_cpp = generate_main(
        "tests/component_tests/tas58xx/test_tas58xx_mixer_select.yaml"
    )

    assert "mixer_btl->set_parent(amp_btl);" in main_cpp
    assert "mixer_pbtl->set_parent(amp_pbtl);" in main_cpp
    assert "App.register_component_(mixer_btl" in main_cpp
    assert "App.register_component_(mixer_pbtl" in main_cpp
