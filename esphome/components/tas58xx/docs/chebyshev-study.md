# Chebyshev 2 low pass / high pass study (routine 2026-10-19)

One-off study for the tas58xx component. Question: add a 2nd order Chebyshev low pass / high pass (as in TI PurePath
Console 3, PPC3) to the biquad helpers, or shelve it?

## Summary and recommendation

**Recommendation: include it.** It is not a lot more complex or expensive than Butterworth 2.

- Cost: 1 tan, 1 exp, 2 sqrt, 3 divides per call (Butterworth 2: 1 sin, 1 cos, 2 divides). Host run time is
  about 1.1x to 1.4x Butterworth at -O2 (up to about 2x at -Os); flash is about +0.7 kB (x86-64 -Os, estimate for
  the ESP32 below). Coefficients are only calculated when a setting changes, so this is not significant.
- Maths simplify well. No asinh / sinh / cosh and no wc is needed: with u = 10^(|R|/20) and epsilon = sqrt(u^2 - 1),
  b = u / (2 epsilon) and p = sqrt(2 b - 1). The function is about 40 lines.
- Correctness: the C++ matches the Python reference with **0 LSB difference** in 5.27 over 34,560 cases (96 kHz,
  20 Hz to 16 kHz, ripple 0.1 to 3 dB both signs, low and high pass, gains -12 to +24 dB).
- The reference maths also reproduce the PPC3 coefficients in the **main branch presets** (see
  "Which presets are Chebyshev"): all 20 LF / HF presets are Chebyshev with **0.5 dB ripple**; the C++ matches them
  within 1 LSB (17 exact, 3 off by 1).
- Stable and no overflow: largest pole radius 0.99958 (20 Hz). All coefficients are below 16 up to +24 dB for low
  pass; high pass is limited to +18 dB, the same as the existing `high_pass`.
- Proposed YAML: `eq_filter: low_pass_chebyshev` / `high_pass_chebyshev` with `ripple:` 0.1 to 3.0 dB (default 0.5),
  gain like `low_pass` / `high_pass`. Details in "Proposal".
- **README vs code:** the README (main, and still on `dev_update_eq_proto`) says the presets are 2nd order
  Chebyshev. That is true for the main branch (PPC3 tables, 0.5 dB ripple). The presets code on
  `dev_update_eq_proto` uses **Butterworth** `FILTER_TYPE_LOW_PASS` / `HIGH_PASS`, so those presets differ from main
  (and the +1 dB HF shelf, which offsets the Chebyshev passband loss, is now not needed). Adding Chebyshev lets the
  presets match main again.

## Method

1. `cheb_ref.py`: literal Python version of the PPC3 description (asinh as ln, pole angles 3pi/4 and 5pi/4, bilinear
   transform with pre-warping).
2. `cheb_snippet.cpp`: efficient C++ in the helpers' style. It was spliced into a copy of
   `tas58xx_helpers.cpp` (repository unchanged) and built on the host with g++ -std=gnu++20 and the
   `docs/tools/biquad_check/stub` headers.
3. `compare.py` compares both in 5.27 (the Python value is rounded and saturated like `double_to_fixed_5_27`).
4. `response.py` checks the frequency response of the reference and the gain limits.
5. `timing.cpp` times Butterworth and Chebyshev calls.

Simplification used (checked numerically against the long form):

- a = -sqrt(2) sinh(alpha) and b = cosh(2 alpha) / 2 (the brief's simplification is right).
- e^(2 alpha) = sqrt((u + 1) / (u - 1)), so cosh(2 alpha) = u / epsilon and **b = u / (2 epsilon)**.
- cosh(2 alpha) - 1 = 2 sinh(alpha)^2, so **p = -a = sqrt(2 b - 1) = sqrt(u / epsilon - 1)**.
- c = b / sqrt(1 + epsilon^2) = b / u = 1 / (2 epsilon) for R > 0.
- With T = tan(pi f / Fs) and k = wc / T, dividing every term by wc^2 and multiplying by T^2 removes wc completely.
  Low pass: D = 1 + p T + b T^2. High pass: D = b + p T + T^2 (details in the code comment).
- A negative ripple is the same filter as the positive one with b0, b1, b2 multiplied by 10^(|R|/20) (checked,
  difference 2e-16). So the sign only moves the level; in the YAML a positive ripple plus a gain is enough.

## Results

### C++ against Python reference (5.27, fs 96000)

Frequencies: 48 log spaced values 20 Hz to 16 kHz (rounded to whole Hz as the component does). Ripple: +-0.1 to
+-3.0 dB in 0.1 steps (60 values). Low and high pass. Gains 0, -12, -1, 3, 12, 24 dB. 34,560 cases.

| Check | Result |
|---|---|
| Maximum difference C++ vs Python | **0 LSB** (every gain, low and high) |
| Largest pole radius (exact) | 0.99957800 (low pass, 20 Hz, ripple -3 dB) |
| Largest pole radius after 5.27 rounding | 0.99957800, inside the unit circle everywhere |
| Largest coefficient, gain 0 | 1.9992 (limit 16) |
| Largest coefficient, low pass +12 / +24 dB | 2.53 / 10.08 |
| Largest coefficient, high pass +3 / +12 / +24 dB | 2.82 / 7.96 / 31.68 (overflows) |

Highest gain with every coefficient below 16 (20 Hz to 16 kHz): low pass 24 dB for all ripples; high pass 18 dB
for all ripples (21 dB for some positive ripples). The high pass overflow is b1 = -2 x b0, as for `high_pass`.

### Frequency response (reference, gain 0)

| Check | Result |
|---|---|
| Low pass DC gain; high pass Nyquist gain | -|R| dB for R > 0, 0 dB for R < 0; error under 3e-8 dB |
| Gain at f | equals the ripple band edge level (-|R| dB for R > 0, 0 dB for R < 0) to 1e-9 dB |
| Pass band max - min | |R| dB to within 0.002 dB |
| Low pass gain at Nyquist | 0 (zeros at z = -1) |

So f is the **ripple band edge**, not the -3 dB point. The -3 dB point, relative to the 0 dB ripple peaks (R > 0):

| Ripple dB | 0.1 | 0.5 | 1.0 | 2.0 | 3.0 | 3.0103 |
|---|---|---|---|---|---|---|
| low pass -3 dB at (x f) | 1.94 | 1.39 | 1.22 | 1.07 | 1.001 | 1.000 |
| high pass -3 dB at (x f) | 0.515 | 0.720 | 0.821 | 0.931 | 0.999 | 1.000 |

A ripple of about 0 does **not** behave like Butterworth. As R goes to 0 the -3 dB point moves away from f (about 2x
at 0.1 dB and further away for smaller ripple), and R = 0 itself is a pass-through (b = infinite). The -3 dB point equals f
only at R = 3.0103 dB (epsilon = 1), but that filter is still not Butterworth (low pass, 100 Hz at 96 kHz:
b0 5.34e-6 and a1 1.99577, Butterworth 1.07e-5 and 1.99074; it has a 3 dB ripple). The
Butterworth 2 curve has no equivalent ripple value. This is why 0.1 dB is proposed as the lower limit.

### Cost against Butterworth 2 (`pass_filter`), per call, gain 0

| | Butterworth 2 | Chebyshev 2 |
|---|---|---|
| sin / cos / tan | 1 / 1 / 0 | 0 / 0 / 1 |
| exp (ripple) | 0 | 1 |
| exp (gain, only if gain is not 0) | 1 | 1 |
| sqrt | 0 | 2 |
| log | 0 | 0 |
| multiplies (approx, incl. 5 x 5.27 scaling) | 16 | 21 (high) to 23 (low) |
| divides | 2 | 3 |
| host time -O2, ns per call (gain 0) | about 50 to 60 | about 66 to 77 (ratio 1.1 to 1.4) |
| host time -O2, gain 6 dB | about 61 to 75 | about 69 to 84 (ratio 0.9 to 1.3) |
| host time -Os, gain 0 | about 49 to 64 | about 93 to 104 (ratio 1.5 to 2.0) |
| code size x86-64 -Os | pass_filter 327 bytes | chebyshev_filter 686 bytes (+0.36 kB over pass_filter) |

Host timing is noisy (3 runs each, 4,000,000 calls, frequency changing every call). Total added to the helpers
object file: 880 bytes, which includes two small test wrappers (about 90 bytes), so about 0.7 to 0.8 kB.
**Estimate for the ESP32 (not measured):** double maths is in software there, so time is mostly the sin / cos / tan /
exp / sqrt / divide calls. Chebyshev has 4 library calls (tan, exp, 2 sqrt) and 3 divides; Butterworth has 2
(sin, cos) and 2 divides. Expect roughly 1.3 to 2x Butterworth per call, tens of
microseconds in absolute terms, and about 0.7 to 1.5 kB of flash. A band is only recalculated when its setting
changes (preset: one call for the two identical biquads), so this does not matter.

### Which presets are Chebyshev

- **main** (`tas58xx_eq_profiles.h`): PPC3 exports, comments say "Low Pass-Chebyshev" / "High Pass-Chebyshev". I
  decoded LF and HF 60, 100 and 150 Hz and fitted the ripple: best fit is **+0.5 dB** (error about 1e-8, which is
  the 5.27 rounding). Butterworth is clearly different (eg LF 60 Hz b0 5.51e-6 against Butterworth 3.84e-6; HF 60 Hz
  b0 0.9423 against 0.9972). All 20 presets: `chebyshev_filter(..., 0.5)` gives 17 identical and 3 off by 1 LSB.
  This is an independent check of the PPC3 maths in the brief. The 3rd biquad is a +1 dB high shelf, Q 0.5, 200 to
  300 Hz, which makes up the 2 x 0.5 dB Chebyshev loss of the HF presets.
- **dev_update_eq_proto**: `tas58xx_eq.h` uses `FILTER_TYPE_LOW_PASS` / `HIGH_PASS`, which are Butterworth 2.
  The README text "2nd order Chebyshev" is therefore only true for main. The +1 dB shelf (`EQ_PRESET_SHELF_GAIN`) is
  kept, but with Butterworth there is no loss to make up, so it would lift the treble above the shelf frequency by about 1 dB (not measured). Decide: either
  switch the presets to Chebyshev 0.5 dB (matches main and the README), or keep Butterworth and fix the README and
  the shelf gain.

### Against PPC3 exports (2026-10-19)

Exports from the startup sequence modes (see dev-notes.md, "PPC3 audio modes"), checked with the PPC3 maths and the
C++ form at 0.5 dB ripple, fs 96000:
- TAS5805M, 2.0 96k 3 Band DRC: 14 biquads, high and low pass at 30, 350, 700, 1000, 3000, 6000 and 12000 Hz,
  gains -24 to +18 dB. All fit 0.5 dB best; 10 exact, 4 within 1 LSB.
- TAS5825M, 2.0 96k 2 Band DRC & AGL: high pass 10 Hz +18 dB (largest coefficient 14.99, within 1 LSB), low pass
  20 Hz +18 dB, high and low pass 20 Hz -24 dB (exact). Same coefficients as the TAS5805M would give: same 96 kHz
  flow and 5.27 format.
- Gain limit: the high pass is the limit and gets worse as the frequency drops (14.99 at 10 Hz, +18 dB); +19 dB
  overflows below about 1 kHz. A low pass only overflows above about 44 kHz at +19 dB. A cut never matters (largest
  coefficient 2). PPC3 does not let a coefficient wrap: at 20 Hz +20 dB it exported b0 = b2 = 8.0, b1 = -16 (0x80000001),
  the same 1 : -2 : 1 shape scaled to fit, so the gain becomes about +18.5 dB, not +20. Cap Chebyshev high pass at +18 dB.

### Effect of the ripple, and why 0.5 dB (2026-10-19)

A 2nd order Chebyshev is one pole pair, so the ripple acts like a Q setting (analog prototype; the frequency is the
ripple band edge):

| Ripple | Equivalent Q | Near the cutoff | -3 dB point (low pass; high pass f / x) |
|---|---|---|---|
| 0 (Butterworth) | 0.707 | flat | 1.00 x f |
| 0.1 dB | 0.77 | almost Butterworth | 1.94 x f |
| **0.5 dB** | **0.86** | 0.5 dB bump, slightly sharper knee | **1.39 x f** |
| 1 dB | 0.96 | 1 dB bump | 1.22 x f |
| 3 dB | 1.31 | 3 dB hump, boomy / ringing | 1.00 x f |

- Far from the cutoff every ripple gives 12 dB per octave; more ripple only sharpens the knee, with a bigger bump,
  more overshoot and ringing.
- **PPC3's default ripple is 1.0 dB**, not 0.5 (each Chebyshev filter has to be changed to 0.5 by hand when exporting).
  So sonocotta chose 0.5 dB for the main presets on purpose, and the +1 dB shelf makes up the 2 x 0.5 dB loss.
- Guidance found: MathWorks' standard Chebyshev I example uses 0.5 dB; TI filter notes use 0.2 to 0.5 dB examples;
  all say more ripple means a steeper transition but more overshoot and ringing, and TI notes crossovers favour low
  overshoot. (The DSP Guide's "0.5%" is per cent, about 0.04 dB.)
- Decision: fixed at 0.5 dB for now (`CHEBYSHEV_RIPPLE_DB` in `dev_update_eq_proto`). If a `ripple:` option is ever
  added, 0.1 to 1.0 dB is the sensible range (this replaces the 0.1 to 3.0 suggested above); above 1 dB the bump
  sounds like a deliberate boost.

### sqrt cost, and constants if the ripple stays fixed (2026-10-19, for later)

- The ESP32 FPU is single precision only, so every double operation is software. Rough order (not measured):
  add / multiply cheapest, divide and sqrt a few times more (similar to each other), sin / cos / tan / exp several
  times more again (series of multiplies). sqrt is not the expensive part.
- sqrt use in the helpers: only band pass (1, and it replaces a cos: 1 / cos(x) = sqrt(1 + tan(x)^2)) and Chebyshev
  (2, plus 1 exp). Shelf avoids one (sqrt(A) = 10^(gain / 80)); Butterworth uses the constant `INVERSE_SQRT2`.
- No useful sqrt speed-up: `sqrtf` or fast inverse sqrt tricks are float accuracy (24 bits) and would lose the 1 LSB
  match with PPC3. The only real saving is not calculating it.
- If the ripple is never made configurable, `b`, `c` and `p` depend only on the ripple and can be constants. For
  0.5 dB (from `chebyshev_filter()`'s own maths, check p^2 = 2b - 1):
  `b = 1.5162026269459308`, `c = 1.4313875806215943`, `p = 1.4256245136402017`.
  Then the filter is one `tan`, one divide and an exp only when the gain is not 0 (`linear_gain`), cheaper than
  Butterworth (sin and cos). Cost: the `ripple` argument goes, so a later `ripple:` option would need it back.
- Not worth it for speed now: a band is recalculated only when its setting changes, not per sample. Keep the
  ripple argument (`CHEBYSHEV_RIPPLE_DB`) unless the ripple is decided to be fixed for good.

## Proposal

- **YAML** (EQ band, as the other `eq_filter` values): `low_pass_chebyshev`, `high_pass_chebyshev`.
  - `ripple:` in dB, 0.1 to 3.0, default 0.5 (the value in the main presets). Positive only: negative ripple is the
    same filter with extra gain of |R| dB, which the `gain` already provides. The C++ still accepts a negative value.
    Below 0.1 dB the -3 dB point moves far above f, and above 3.0 dB it is below f.
  - `frequency:` 20 Hz to 16 kHz as now; the cutoff is the ripple band edge (document this).
  - `gain:` like `low_pass` / `high_pass`: low pass -24 to +24 dB; high pass limited to +18 dB (add to
    `EQ_GAIN_LIMITED_FILTERS`); both fit with no coefficient overflow at any ripple, 20 Hz to 16 kHz.
  - No `q_factor` (add to the "fixed Q" group, `EQ_FIXED_Q_FILTERS`). Ripple can use the `q_factor` slot of
    `make_biquad()`, so the signature does not change, or the number platform can pass a separate value.
- **Code:** two `FilterType` values (`FILTER_TYPE_LOW_PASS_CHEBYSHEV`, `FILTER_TYPE_HIGH_PASS_CHEBYSHEV`) added at the
  end of the enum (so existing values keep their numbers, which `ppc_check/compare.py` relies on), two `case`
  lines in `make_biquad()` that call `chebyshev_filter(high, ..., q_factor)`, and in `audio_dac.py` the two names in
  `EQ_BAND_FILTERS`, `EQ_FILTER_ORDER` (high pass 0, low pass 1) and the limited gain list.
- **Fit with the EQ proposal:** number bands only (frequency, gain, and ripple as the band's third value), the same
  as `low_pass` / `high_pass`. Presets can use it as above. A cascade of two Chebyshev biquads (4th order) has
  twice the ripple and loss in dB.
- To check before merging: a PPC3 reference for Chebyshev (add rows to `ppc_check/ppc_reference.cfg`; the main
  preset tables already agree to 1 LSB), and a hardware sweep of one LF and one HF preset.

## Proposed C++ (complete, compilable)

Insert after `pass_filter()` in `tas58xx_helpers.cpp` (it uses `ALL_PASS`, `LN10_DIV_20`, `angular_frequency`,
`linear_gain` and `double_to_fixed_5_27` from the same file). `run.sh` below compiles it with no warnings
(`-Wall -Wextra`) in a copy of the file.

```cpp
// 2nd order Chebyshev low pass / high pass as in TI Pure Path Console 3. The frequency is the edge of the ripple
// band, not the -3 dB point. ripple > 0: the ripple peaks are 0 dB and the DC (low pass) or Nyquist (high pass) gain
// is -ripple dB. ripple < 0: that gain is 0 dB and the peaks are +|ripple| dB. ripple = 0 gives all pass.
//
// TI's maths simplified with u = 10^(|ripple| / 20), epsilon = sqrt(u^2 - 1), e^(2 alpha) = sqrt((u + 1) / (u - 1)):
//   b = cosh(2 alpha) / 2 = u / (2 epsilon)           pole product in units of wc^2
//   p = -a = sqrt(2) sinh(alpha) = sqrt(2 b - 1)      pole sum in units of wc
//   c = b / u = 1 / (2 epsilon) for ripple > 0, else c = b
// With T = tan(pi f / Fs), k = wc / T, dividing every term by wc^2 and multiplying by T^2 removes wc:
//   low pass:  D = 1 + p T + b T^2, b0 = c T^2 / D, a1 = (2 - 2 b T^2) / D, a2 = (p T - 1 - b T^2) / D
//   high pass: D = b + p T + T^2,   b0 = c / D,     a1 = (2 b - 2 T^2) / D, a2 = (p T - b - T^2) / D
// b1 = +/-2 b0 and b2 = b0; a1 and a2 are already in the sign the DSP expects.
static BiquadCoefficients chebyshev_filter(bool high, uint32_t sample_rate, uint16_t frequency, int8_t gain,
                                           double ripple) {
  if (ripple == 0.0) return ALL_PASS;

  const double u = std::exp(std::fabs(ripple) * LN10_DIV_20);
  const double inverse_epsilon = 1.0 / std::sqrt((u - 1.0) * (u + 1.0));
  const double b = 0.5 * u * inverse_epsilon;
  const double c = ripple > 0.0 ? 0.5 * inverse_epsilon : b;
  const double p = std::sqrt((u * inverse_epsilon) - 1.0);

  const double t = std::tan(0.5 * angular_frequency(sample_rate, frequency));
  const double t2 = t * t;
  const double pt = p * t;
  const double ag = linear_gain(gain, LN10_DIV_20);

  BiquadCoefficients result{};
  double b0;
  if (high) {
    const double inverse_d = 1.0 / (b + pt + t2);
    b0 = c * inverse_d * ag;
    result.b1 = double_to_fixed_5_27(-2.0 * b0);
    result.a1 = double_to_fixed_5_27((2.0 * (b - t2)) * inverse_d);
    result.a2 = double_to_fixed_5_27((pt - b - t2) * inverse_d);
  } else {
    const double bt2 = b * t2;
    const double inverse_d = 1.0 / (1.0 + pt + bt2);
    b0 = c * t2 * inverse_d * ag;
    result.b1 = double_to_fixed_5_27(2.0 * b0);
    result.a1 = double_to_fixed_5_27((2.0 - (2.0 * bt2)) * inverse_d);
    result.a2 = double_to_fixed_5_27((pt - 1.0 - bt2) * inverse_d);
  }
  result.b0 = double_to_fixed_5_27(b0);
  result.b2 = result.b0;
  return result;
}
```

Matching changes (not made, shown for the follow-up):

```cpp
// tas58xx_helpers.h, end of enum FilterType
  FILTER_TYPE_LOW_PASS_CHEBYSHEV,   // 2nd order, ripple (dB) in q_factor, frequency is the ripple band edge
  FILTER_TYPE_HIGH_PASS_CHEBYSHEV,  // 2nd order, ripple (dB) in q_factor, frequency is the ripple band edge

// tas58xx_helpers.cpp, make_biquad()
    case FILTER_TYPE_LOW_PASS_CHEBYSHEV:
      return chebyshev_filter(false, sample_rate, frequency, gain, q_factor);
    case FILTER_TYPE_HIGH_PASS_CHEBYSHEV:
      return chebyshev_filter(true, sample_rate, frequency, gain, q_factor);
```

## Python reference (`cheb_ref.py`)

```python
"""Literal Python (double precision) reference of the PPC3 Chebyshev 2 low / high pass, as described in the brief."""
import math


def asinh(x):
    return math.log(x + math.sqrt(x * x + 1.0))


def chebyshev_reference(high, fs, f, gain_db, ripple_db):
    """Returns (b0, b1, b2, a1, a2) as floats, a1 and a2 with the sign the DSP expects."""
    if ripple_db == 0:
        return (1.0, 0.0, 0.0, 0.0, 0.0)
    eps = math.sqrt(10 ** (abs(ripple_db) / 10) - 1)
    alpha = asinh(1 / eps) / 2
    beta1, beta2 = 3 * math.pi / 4, 5 * math.pi / 4
    a = math.sinh(alpha) * (math.cos(beta1) + math.cos(beta2))
    b = (math.sinh(alpha) ** 2 * math.cos(beta1) * math.cos(beta2)
         - math.cosh(alpha) ** 2 * math.sin(beta1) * math.sin(beta2))
    c = b
    if ripple_db > 0:
        c = b / math.sqrt(1 + eps * eps)
    wc = 2 * math.pi * f
    k = wc / math.tan(math.pi * f / fs)
    if not high:
        n = c * wc ** 2
        a1_, a2_ = -wc * a, b * wc ** 2
        d = k * k + a2_ + a1_ * k
        co = [n / d, 2 * n / d, n / d]
    else:
        g = c / b
        a1_, a2_ = -wc * a / b, wc ** 2 / b
        d = k * k + a2_ + a1_ * k
        co = [g * k * k / d, -2 * g * k * k / d, g * k * k / d]
    out_a1 = (2 * k * k - 2 * a2_) / d
    out_a2 = (a1_ * k - k * k - a2_) / d
    lin = 10 ** (gain_db / 20)
    return (co[0] * lin, co[1] * lin, co[2] * lin, out_a1, out_a2)
```

## Test scripts (kept short; copies are in the run's scratch directory, not in the repository)

`run.sh <esphome-tas58xx checkout> <work dir>` (needs g++ and python3; the work dir holds the files below):

```bash
#!/usr/bin/env bash
# Splices cheb_snippet.cpp into a copy of tas58xx_helpers.cpp (nothing in the repository changes), builds with the
# biquad_check stub headers and compares the C++ with cheb_ref.py.  usage: run.sh <esphome-tas58xx checkout> <work dir>
set -eu
REPO="$1"; D="$2"; SRC="$REPO/components/tas58xx"; STUB="$REPO/docs/tools/biquad_check/stub"
python3 - "$SRC/tas58xx_helpers.cpp" "$D" <<'PY'
import sys
src, d = sys.argv[1], sys.argv[2]
s = open(src).read(); snip = open(d + "/cheb_snippet.cpp").read()
s = s.replace("\nint32_t gain_to_fixed_9_23", "\n" + snip + "\nint32_t gain_to_fixed_9_23", 1)
s = s.replace("}  // namespace esphome::tas58xx",
  "BiquadCoefficients test_chebyshev(bool h, uint32_t fs, uint16_t f, int8_t g, double r) {\n"
  "  return chebyshev_filter(h, fs, f, g, r);\n}\n"
  "BiquadCoefficients test_butter(bool h, uint32_t fs, uint16_t f, int8_t g) { return pass_filter(h, fs, f, g); }\n"
  "}  // namespace esphome::tas58xx")
open(d + "/helpers_with_cheb.cpp", "w").write(s)
PY
g++ -std=gnu++20 -O2 -Wall -Wextra -I"$STUB" -I"$SRC" "$D/check.cpp" "$D/helpers_with_cheb.cpp" -o "$D/check"
"$D/check" > "$D/cpp_out.txt"
python3 -I "$D/compare.py" "$D/cpp_out.txt"
python3 -I "$D/response.py"
```

`check.cpp` (dumps the C++ coefficients):

```cpp
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include "tas58xx_helpers.h"
namespace esphome::tas58xx {
BiquadCoefficients test_chebyshev(bool,uint32_t,uint16_t,int8_t,double);
}
using namespace esphome::tas58xx;
int main(){
  uint32_t fs=96000;
  const int gains[]={0,-12,-1,3,12,24};
  for(int h=0;h<2;h++) for(int i=0;i<48;i++){
    double f=20*pow(800.0,i/47.0); uint16_t fi=(uint16_t)lround(f);
    for(int ri=-30;ri<=30;ri++){ if(ri==0) continue; double r=ri/10.0;
      for(int g:gains){
        BiquadCoefficients b=test_chebyshev(h,fs,fi,g,r);
        printf("%d %u %d %.1f %08x %08x %08x %08x %08x\n",h,fi,g,r,(uint32_t)b.b0,(uint32_t)b.b1,(uint32_t)b.b2,(uint32_t)b.a1,(uint32_t)b.a2);
      }}}
}
```

`compare.py` (5.27 comparison, pole radius, overflow):

```python
"""Compare the C++ dump (hex, byte swapped 5.27) with cheb_ref.py in 5.27; also stability and overflow."""
import math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cheb_ref import chebyshev_reference

def to_5_27(x):
    v = max(-2**31, min(2**31 - 1, math.floor(x * 2**27 + 0.5)))  # std::round: half away from zero, ties are rare
    return v

def from_hex(h):
    v = int.from_bytes(bytes.fromhex(h), "little", signed=True)  # undo the byte swap
    return v

fs = 96000
max_diff = 0; worst = None; n = 0
max_abs = 0.0; max_abs_case = None; max_pole = 0.0; max_pole_case = None
stats = {}
for line in open(sys.argv[1]):
    t = line.split()
    high, f, g, r = int(t[0]), int(t[1]), int(t[2]), float(t[3])
    cpp = [from_hex(x) for x in t[4:9]]
    ref = chebyshev_reference(bool(high), fs, f, g, r)
    refq = [to_5_27(x) for x in ref]
    d = max(abs(a - b) for a, b in zip(cpp, refq))
    n += 1
    key = (high, g)
    if d > max_diff: max_diff, worst = d, (t[:4], cpp, refq)
    qa1, qa2 = cpp[3] / 2**27, cpp[4] / 2**27   # poles of the rounded 5.27 coefficients
    qd = qa1 * qa1 + 4 * qa2
    qrad = math.sqrt(-qa2) if qd < 0 else max(abs((qa1 + math.sqrt(qd)) / 2), abs((qa1 - math.sqrt(qd)) / 2))
    max_qpole = max(globals().get('max_qpole', 0.0), qrad)
    # overflow and stability from the exact reference
    m = max(abs(x) for x in ref)
    if m > max_abs: max_abs, max_abs_case = m, (high, f, g, r)
    a1, a2 = ref[3], ref[4]   # y = ... + a1 y1 + a2 y2  -> poles z^2 - a1 z - a2
    disc = a1 * a1 + 4 * a2
    rad = math.sqrt(-a2) if disc < 0 else max(abs((a1 + math.sqrt(disc)) / 2), abs((a1 - math.sqrt(disc)) / 2))
    if rad > max_pole: max_pole, max_pole_case = rad, (high, f, g, r)
    s = stats.setdefault(key, [0, 0.0])
    s[0] = max(s[0], d); s[1] = max(s[1], m)
print(f"cases {n}, max LSB difference {max_diff}, worst {worst}")
print(f"max |coefficient| {max_abs:.4f} at {max_abs_case} (limit 16)")
print(f"max pole radius of the rounded 5.27 coefficients {max_qpole:.8f}")
print(f"max pole radius {max_pole:.8f} at {max_pole_case}")
for (h, g), (d, m) in sorted(stats.items()):
    print(f"  {'high' if h else 'low '} gain {g:+3d}: max LSB diff {d}, max |coef| {m:.4f}")
```

`response.py` (frequency response, -3 dB points, gain limits):

```python
"""Frequency response and gain-limit checks of the reference (fs 96000, 20 Hz - 16 kHz, ripple 0.1 - 3 dB, both signs)."""
import math, cmath, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cheb_ref import chebyshev_reference
fs = 96000
def H(co, f):
    b0, b1, b2, a1, a2 = co
    z1 = cmath.exp(-2j * math.pi * f / fs)
    return (b0 + b1 * z1 + b2 * z1 * z1) / (1 - a1 * z1 - a2 * z1 * z1)
db = lambda x: 20 * math.log10(max(abs(x), 1e-12))
freqs = [20 * 800 ** (i / 47) for i in range(48)]
rip = [r / 10 * s for r in range(1, 31) for s in (1, -1)]
# band ripple, DC, Nyquist, level at f
worst_dc = worst_edge = worst_ripple = worst_ny = 0
n3 = {}
for high in (False, True):
    for r in rip:
        for f in freqs:
            fi = round(f); co = chebyshev_reference(high, fs, fi, 0, r)
            if not high:
                dc = db(H(co, 0.001)); band = [db(H(co, fi * 10 ** (-3 * i / 400))) for i in range(0, 401)]
                ny = db(H(co, fs / 2))
                exp_dc = -abs(r) if r > 0 else 0.0
                worst_dc = max(worst_dc, abs(dc - exp_dc))
                worst_ripple = max(worst_ripple, abs((max(band) - min(band)) - abs(r)))
                worst_ny = min(worst_ny, ny)
            else:
                ny = db(H(co, fs / 2)); band = [db(H(co, fi * (fs / 2 / fi) ** (i / 800) * 0.9999)) for i in range(0, 801)]
                exp_ny = -abs(r) if r > 0 else 0.0
                worst_dc = max(worst_dc, abs(ny - exp_ny))
                worst_ripple = max(worst_ripple, abs((max(band) - min(band)) - abs(r)))
            edge = db(H(co, fi))
            # at f: low pass ripple edge level; for r>0 it is -|r|, for r<0 it is 0 dB
            expected_edge = -abs(r) if r > 0 else 0.0
            if abs(edge - expected_edge) > worst_edge: worst_edge = abs(edge - expected_edge)
print(f"DC (low) / Nyquist (high) gain error vs -|R| (R>0) or 0 dB (R<0): max {worst_dc:.2e} dB")
print(f"level at f (ripple band edge) vs the ripple band edge level: max error {worst_edge:.2e} dB")
print(f"max-min of pass band response vs |R|: max error {worst_ripple:.3f} dB (log grid, 400/800 points)")
# -3 dB frequency relative to f  (R>0 low pass, normalised: gain 0 dB peaks)
print("\n-3 dB point / f (low pass, response relative to 0 dB band peaks), ratio is independent of f well below fs/2")
print("ripple dB | -3 dB point / f (low pass LP, high pass HP)")
def f3(high, r, f=100):
    co = chebyshev_reference(high, fs, f, 0, r)
    lo, hi = (f, fs / 2 - 1) if not high else (1.0, f)
    ref_level = 0.0 if r > 0 else abs(r)  # band peak level
    t = ref_level - 3.0103
    for _ in range(100):
        m = (lo + hi) / 2
        v = db(H(co, m))
        if not high: (lo, hi) = (m, hi) if v > t else (lo, m)
        else: (lo, hi) = (lo, m) if v > t else (m, hi)
    return (lo + hi) / 2
for r in (0.1, 0.5, 1.0, 2.0, 3.0, 3.0103):
    print(f"{r:8.4f}  | {f3(False, r) / 100:8.3f} (LP)  {f3(True, r) / 100:8.3f} (HP)")
# gain limits: largest integer gain (dB) with |coef| < 16 for all freqs, per filter and ripple
print("\nlargest gain (dB) with all coefficients < 16, 20 Hz - 16 kHz at 96 kHz; (worst ripple)")
for high in (False, True):
    for sign in (1, -1):
        res = {}
        for r in [x / 10 for x in range(1, 31)]:
            g = None
            for gain in range(0, 25):
                ok = all(max(abs(c) for c in chebyshev_reference(high, fs, round(f), gain, sign * r)) < 16 for f in freqs)
                if ok: g = gain
                else: break
            res[r] = g
        print(f"{'high' if high else 'low '} pass, ripple {'>0' if sign > 0 else '<0'}: min over ripple {min(res.values())} dB, max {max(res.values())} dB")
```

`timing.cpp` (relative run time; build it like `run.sh` does for `check.cpp`, with `timing.cpp` in place of
`check.cpp` and `-O2` or `-Os`):

```cpp
// relative host run time of Butterworth 2 (pass_filter) and Chebyshev 2 (chebyshev_filter), one call = one band recalculation
#include <chrono>
#include <cstdio>
#include "tas58xx_helpers.h"
namespace esphome::tas58xx {
BiquadCoefficients test_chebyshev(bool, uint32_t, uint16_t, int8_t, double);
BiquadCoefficients test_butter(bool, uint32_t, uint16_t, int8_t);
}
using namespace esphome::tas58xx;
template <typename F> static double run(F f, int n) {
  volatile int32_t sink = 0;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < n; i++) {
    BiquadCoefficients b = f(i);
    sink = sink + b.b0 + b.a2;
  }
  return std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - t0).count() / n;
}
int main() {
  const int N = 4000000;
  for (int gain : {0, 6}) {
    for (int rep = 0; rep < 3; rep++) {
      double bl = run([&](int i) { return test_butter(false, 96000, 20 + (i & 8191), gain); }, N);
      double bh = run([&](int i) { return test_butter(true, 96000, 20 + (i & 8191), gain); }, N);
      double cl = run([&](int i) { return test_chebyshev(false, 96000, 20 + (i & 8191), gain, 0.5 + 0.1 * (i & 15)); }, N);
      double ch = run([&](int i) { return test_chebyshev(true, 96000, 20 + (i & 8191), gain, 0.5 + 0.1 * (i & 15)); }, N);
      printf("gain %d: butter low %.1f ns high %.1f ns | cheb low %.1f ns high %.1f ns | ratio %.2f %.2f\n", gain, bl, bh, cl, ch, cl / bl, ch / bh);
    }
  }
}
```

Output of `run.sh`:

```text
cases 34560, max LSB difference 0, worst None
max |coefficient| 31.6830 at (1, 20, 24, -0.1) (limit 16)
max pole radius of the rounded 5.27 coefficients 0.99957800
max pole radius 0.99957800 at (0, 20, 0, -3.0)
  low  gain -12: max LSB diff 0, max |coef| 1.9992
  low  gain  -1: max LSB diff 0, max |coef| 1.9992
  low  gain  +0: max LSB diff 0, max |coef| 1.9992
  low  gain  +3: max LSB diff 0, max |coef| 1.9992
  low  gain +12: max LSB diff 0, max |coef| 2.5316
  low  gain +24: max LSB diff 0, max |coef| 10.0784
  high gain -12: max LSB diff 0, max |coef| 1.9991
  high gain  -1: max LSB diff 0, max |coef| 1.9991
  high gain  +0: max LSB diff 0, max |coef| 1.9991
  high gain  +3: max LSB diff 0, max |coef| 2.8238
  high gain +12: max LSB diff 0, max |coef| 7.9584
  high gain +24: max LSB diff 0, max |coef| 31.6830
DC (low) / Nyquist (high) gain error vs -|R| (R>0) or 0 dB (R<0): max 2.27e-08 dB
level at f (ripple band edge) vs the ripple band edge level: max error 1.36e-09 dB
max-min of pass band response vs |R|: max error 0.002 dB (log grid, 400/800 points)

-3 dB point / f (low pass, response relative to 0 dB band peaks), ratio is independent of f well below fs/2
ripple dB | -3 dB point / f (low pass LP, high pass HP)
  0.1000  |    1.943 (LP)     0.515 (HP)
  0.5000  |    1.390 (LP)     0.720 (HP)
  1.0000  |    1.218 (LP)     0.821 (HP)
  2.0000  |    1.074 (LP)     0.931 (HP)
  3.0000  |    1.001 (LP)     0.999 (HP)
  3.0103  |    1.000 (LP)     1.000 (HP)

largest gain (dB) with all coefficients < 16, 20 Hz - 16 kHz at 96 kHz; (worst ripple)
low  pass, ripple >0: min over ripple 24 dB, max 24 dB
low  pass, ripple <0: min over ripple 24 dB, max 24 dB
high pass, ripple >0: min over ripple 18 dB, max 21 dB
high pass, ripple <0: min over ripple 18 dB, max 18 dB
```
