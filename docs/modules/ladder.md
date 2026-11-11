# Ladder filter (4-pole, Moog-style)

Files: `include/pcl/pcl_ladder.h`, `src/filter/pcl_ladder.c`, `tests/test_ladder.c`, the shared internal
header `src/core/pcl_cutoff.h` (moved out of the SVF), the `ladder` subcommand of `tools/render.c`,
and the `ladder_*` cases of `platform/bench/pcl_bench.c`.

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds** (Clang with ASan +
UBSan, GCC `-O2`, and the `cm0plus-*` / `cm33-*` compile checks including the no-soft-float check;
the `mcxn947-*` firmware builds). Response, tuning, sizes and host timings are **measured**. **Cortex-M33
cycle counts are not:** the bench cases exist, but the board was not connected. Cortex-M0+ is
unmeasured, as for every module.

## Summary

The classic four-pole transistor-ladder low-pass: four identical one-pole low-passes in series, with
the output fed back negatively to the input. Resonance rises to self-oscillation, the slope is 24 dB per
octave, and the bass thins as resonance rises. It's a second VCF beside the SVF, with a different
character.

```
stage:     y = G x + (1 - G) s,   s' = 2y - s           G = g / (1 + g),  g = tan(pi fc / fs)
cascade:   y4 = G^4 u + S,        S = (1 - G) (G (G (G s1 + s2) + s3) + s4)
feedback:  u = a (x' - k S),      a = 1 / (1 + k G^4)    k = 4 * resonance
input:     x' = x [* (1 + k) with bass_comp] [* drive with saturate]
saturate:  u = tanh(u)            (passed through below |u| = 1/64)
output:    a pole mix of u, y1 .. y4 per mode (LP24 = y4)
```

* **Zero-delay-feedback (TPT) ladder after Zavalishin**: the bilinear-transformed analogue one-pole,
  with the feedback loop solved exactly. **The cutoff is exact, resonance doesn't detune it, and at
  resonance 1 the filter rings at the cutoff**: measured within 0.013 cents at 440, 1234 and
  5000 Hz in every build.
* **Same control model as the SVF**, from the same code (`src/core/pcl_cutoff.h`): cutoff in Hz,
  smoothed in octaves, an exponential `cutoff_mod` input times a depth in octaves, coefficients at
  control rate (every `PCL_CONTROL_DIV` = 16 samples) and the filter at audio rate.
* **`a = 1/(1 + k G^4)` is tracked by the SVF's Newton step**, never interpolated linearly: `G`, `k` and
  the drive ramp across a control period, and `a <- a (2 - d a)` follows `d = 1 + k G^4` sample by
  sample. Random cutoff jumps of +-4 and +-6 octaves every period stay bounded (peak 0.37-0.72; see
  "Tests").
* **Eight modes by pole mixing** (Xpander style).

## Decisions (from the review of the draft)

All seven were settled with the recommendation:

1. ZDF/TPT linear ladder with one optional `tanh` (not Huovilainen's per-stage model, not the
   unit-delay form).
2. All eight pole-mix modes.
3. Resonance 0 .. 1.1, `k = 4 r`: 1 is the edge of self-oscillation, above it the filter oscillates.
4. `saturate` flag plus `drive` (linear), output not rescaled.
5. `bass_comp` flag.
6. The SVF's cutoff-control code moved into a shared internal header. Verified bit-identical (see
   [svf.md](svf.md)).
7. One output selected by `mode`.

### What changed from the draft while building

* **`drive` scales the input `x`, not `u`.** The draft wrote `u = tanh(drive * u)`. That multiplies
  the loop gain, so drive would also have raised the resonance and detuned it. Now `drive` is a pure
  input gain before the feedback sum, and `tanh` acts on the solved sum at unity small-signal gain.
* **`tanh` is bypassed below `|u| = 1/64`**, where `tanh(u) = u` within 1.3e-6. Found by the
  self-oscillation test: in Q15, `pcl_tanh` rounds to 16 bits, and the resulting deadband stopped a
  self-oscillation at 220 Hz from ever starting from a small click (it decayed to exactly 0). The
  table's first segment also has slope 0.99992, which took 8e-5 off the small-signal loop gain in every
  build.
* **Self-oscillation is quieter than I implied.** At resonance 1.1 the `tanh` settles where its gain is
  4/4.4, so `u` is about 0.55 and the LP24 output, which is `u/4` at the cutoff, is about **0.146
  (-17 dBFS)**. The band-pass modes, with unity gain at the cutoff, output the full ~0.55. Measured
  identically in all three builds.
* **Fixed-point formats:** states, `x`, `u`, `S`, `k`, drive in Q4.28; `G`, `1-G`, `a` in Q2.30; `a k`
  in Q4.28. The Horner sum for `S` runs in **Q6.26**, because it reaches about 26 when the states sit
  at the +-8 clamp. Each stage uses the two-multiply form `G x + (1-G) s`, a convex combination that
  can't leave +-8. `u = a x - (a k) S` is accumulated in Q8.56, since `(a k) S` reaches about 35 at
  the clamp.
* **A UBSan finding:** the first Horner code shifted negative states left (`(int64_t)s << 30`), which is
  undefined in C99. It's now a multiplication by the power of two (same code).

## Parameters

| Parameter | Type / format | Range | Notes |
|-----------|---------------|-------|-------|
| `sample_rate_hz` | `uint32_t` | `PCL_SAMPLE_RATE_MIN_HZ` .. `MAX_HZ` | |
| `cutoff_hz` | `pcl_hz_t` | `PCL_LADDER_CUTOFF_MIN_HZ` (5) .. 0.45 `sample_rate_hz` | Smoothed in octaves over `PCL_SMOOTH_MS`. As in the SVF, fixed point truncates the limit to Q16.16 (21599.87 Hz at 48 kHz) |
| `resonance` | `pcl_coef_t` | 0 .. `PCL_LADDER_RESONANCE_MAX` (1.1) | `k = 4 r`, smoothed linearly. In fixed point Q2.30 `r` has the bit pattern of Q4.28 `k` |
| `mod_depth_oct` | `pcl_oct_t` | 0 .. < 8 | Octaves of cutoff per unit of `cutoff_mod`; not smoothed |
| `mode` | `pcl_ladder_mode_t` | `LP24`, `LP18`, `LP12`, `LP6`, `BP12`, `BP24`, `HP12`, `HP24` | Switches at once (may click) |
| `saturate` | `int` | 0 / non-zero | `tanh` on the solved feedback sum |
| `drive` | `pcl_wide_t` | 1 .. < 8 (linear, up to +18 dB) | Input gain; applied only with `saturate`. Smoothed linearly. `drive x` is clipped at +-8 (the Q4.28 range) |
| `bass_comp` | `int` | 0 / non-zero | Input times `1 + k`: the low-pass keeps unity DC gain |
| `cutoff_mod` (input) | sample | [-1, 1] | Optional; control rate |

`init` and the setters validate (NaN included), return `PCL_ERR_OUT_OF_RANGE` or `PCL_ERR_NULL_POINTER`,
and leave the filter unchanged on error. `process(ld, in, out, cutoff_mod, frames)`: `in` and `out` may
alias.

### Modes

| Mode | Mix | Notes |
|------|-----|-------|
| `LP24` / `LP18` / `LP12` / `LP6` | `y4` / `y3` / `y2` / `y1` | |
| `BP12` | `2 (y1 - y2)` | Gain 1 at the cutoff without resonance |
| `BP24` | `4 (y2 - 2 y3 + y4)` | Gain 1 at the cutoff without resonance |
| `HP12` | `u - 2 y1 + y2` | |
| `HP24` | `u - 4 y1 + 6 y2 - 4 y3 + y4` | |

Fixed point mixes in 64 bits with integer coefficients (no rounding) and saturates the result at full
scale. Taps inside a resonant loop can be far above the input: at resonance 0.9, `LP6` and the
band-passes have gain 7 at the cutoff, so leave headroom (the render patches attenuate the saw to 0.3).

## Algorithm

### Per sample

1. Input to Q4.28 (rounded in Q31). With `bass_comp`, `x += k x`. With `saturate`, `x *= drive`
   (saturated to +-8).
2. `S` by Horner in Q6.26 (four 32x32 -> 64 multiplies).
3. `u = a x - (a k) S` in one 64-bit accumulator, saturated to +-8. With `saturate` and `|u| >= 1/64`,
   `u = tanh(u)` (`pcl_tanh`).
4. The four stages `y = G x + (1-G) s` (two multiplies each), `s' = 2y - s` saturated to +-8. Float adds
   the SVF's anti-denormal guard (1e-24) to each state.
5. The mode's mix, then the output conversion.

While the coefficients ramp, each sample also does `G += dG`, `k += dk`, `drive += ddrv`,
`d = 1 + k G^4` (three multiplies), the Newton step (two), and `a k` (one).

### Control tick

The SVF's: advance the cutoff, resonance and drive smoothers a period; `g = tan(...)` for the smoothed
cutoff plus the modulation; then `G = g/(1+g)` and `a = 1/(1 + k G^4)` exactly (one 64-bit division each
in fixed point), and the per-sample steps. With nothing moving, the tick does nothing and the
non-ramping copy of the loop runs. At the end of every period the coefficients snap to the exact
targets. As in the SVF, the loop body is force-inlined twice (static and ramping).

## Accuracy (measured)

**Response against the exact formula** `H = H1^4 / (1 + k H1^4)` (and the mode mixes) with `H1 = 1 / (1 + j
tan(pi f / fs) / g)`: all 8 modes, cutoffs 100 Hz / 1 kHz / 8 kHz, resonance 0 / 0.5 / 0.9, frequencies
0.25-4 x the cutoff, at 48 and 96 kHz (about 700 points):

| Build | worst magnitude error | worst phase error |
|-------|----------------------:|------------------:|
| float | 0.0043 dB | 0.00036 rad |
| Q31 | 0.0016 dB | 0.00032 rad |
| Q15 | 0.0049 dB | 0.00057 rad |

**Tuning:** at resonance 1 an impulse rings at the cutoff within 0.013 cents (440, 1234, 5000 Hz), and
the level holds for 2 s (within 0.5 % in float; unchanged to four digits in fixed point). With `saturate` at resonance 1.1
the filter starts from a 0.001 click and settles 0.04-0.10 cents flat of the cutoff (220, 1000, 4000
Hz).

**Against a double-precision model of the same per-sample algorithm** (static coefficients taken from the
filter, noise input), worst error: float 7.7e-7, Q31 1.8e-7, Q15 1.5e-5 (half an LSB). With `saturate`:
about 6e-6 in float and Q31 (`pcl_tanh` is a table, 2.3e-5 from libm), 3.3e-5 in Q15.

**Bass loss:** the low-pass DC gain is `1/(1+k)` without compensation and 1 with it (within 2e-3).

## Memory (measured)

* **State:** `sizeof(pcl_ladder_t)` = **148 B** in all three builds (the SVF is 116 B; estimate 130-150).
* **Tables:** none of its own.
* **Code** (`pcl_ladder.c`, text, arm-none-eabi-gcc 15 `-O2` with the presets' flags): **5156 B (M0+
  Q15), 5232 B (M0+ Q31), 3088 B (M33 float), 3996 B (M33 Q31)**, about 1.3 x the SVF compiled the same
  way (4044 / 4036 / 2204 / 3144). The estimate was 5-8 KB. The M0+ objects use only integer runtime
  helpers (`__aeabi_lmul`, `ldivmod`, `uldivmod`), and the soft-float check passes.
* **Stack in `process` (`-fstack-usage`, `-O2`):** 632 B (M0+ Q15), 648 B (M0+ Q31), 152 B (M33 float),
  184 B (M33 Q31). That's large next to the SVF's 40 B: on the M0+ the 64-bit intermediates of the
  inlined loop spill. Not optimized.

The `cm0plus-*` / `cm33-*` presets compile without an optimization flag, so their objects are 2-3 times
larger (18.7 KB for M0+ Q15). Measure code size with `-O2` as above.

## Cost

**Host (x86-64, GCC `-O2`, noise in, cutoff 1.2 kHz, resonance 0.6, 256-frame blocks), ns per sample,
against the SVF in the same conditions:**

| Case | float | Q15 | Q31 |
|------|------:|----:|----:|
| static | 14.7 (1.9 x SVF) | 14.9 (1.8 x) | 15.0 (1.9 x) |
| `cutoff_mod` | 16.8 (1.8 x) | 19.2 (1.7 x) | 19.5 (1.8 x) |
| `saturate` | 22.4 (1.4 x) | 21.3 (2.0 x) | 21.7 (2.0 x) |
| `saturate` + `cutoff_mod` | 23.8 (1.5 x) | 25.1 (1.8 x) | 25.3 (1.8 x) |

So **about 1.7-2 times the SVF**, as the draft expected. Scaled by the SVF's measured M33 cycles (Q31
155 static / 232 modulated, float 64 / 85), that suggests **roughly 280-420 cycles per sample in Q31 and
120-160 in float on the M33**. That's about 9-13 % of a 150 MHz M33 per voice at 48 kHz in fixed point,
4-5 % in float. **This is an estimate, not a measurement:** run `scripts/mcxn947_bench.py` with the board
attached to get the real figures (cases `ladder_lp24`, `ladder_hp24`, `ladder_mod`, `ladder_saturate`,
`ladder_mod_saturate`).

## Rate

Audio-rate filter; coefficients at control rate (every `PCL_CONTROL_DIV` samples), stated in the header.

## Tests (`tests/test_ladder.c`: 2374 checks float, 2195 Q15, 2366 Q31)

* **Init and setters:** valid and invalid configs (rates, cutoff below 5 Hz or above 0.45 fs, resonance
  above 1.1, negative depth, unknown mode, drive below 1 or at 8, NaN in float, NULL), unchanged on
  error, range edges accepted.
* **Response** against the exact formula as above, with the input level chosen so the output stays below
  full scale (the first version saturated the fixed-point output at resonance 0.9 and looked like a
  4 dB error).
* **Bass loss and compensation**, DC gain at three resonances.
* **Resonance 1 rings at the cutoff** (frequency and constant level); **self-oscillation with `saturate`**
  at 1.1 starts from a click, settles at a steady level (within 2 %) near the cutoff (within 1 cent),
  states below 2.
* **Double-precision model** of the same per-sample algorithm, 7 configurations including every
  `saturate` / `drive` / `bass_comp` path and three output modes. It uses the filter's own `G` and `k`
  (white-box) and checks `G` against `tan` separately: the float build's cutoff path is accurate to about
  1e-5, which shows at resonance 0.95, so tuning is left to the response test.
* **Modulation:** a constant +1 octave modulation equals the filter set an octave up (bit-identical in
  all builds); the coefficients return exactly to the unmodulated ones when modulation stops; they snap
  to the targets at the end of every period; random jumps of +-4 and +-6 octaves every period, 10 s
  (5 s noise, 5 s silence), at resonance 0 and 0.5, stay below 2.5 (peak 0.37-0.72).
* **Determinism and blocking:** blocks of 1, 7, 16, 17 and 256 bit-identical to one block, with and
  without modulation, with `saturate`, and with cutoff, resonance, drive and mode changes mid-stream;
  `reset` repeats the output; in-place processing; a zero modulation buffer equals `NULL`.
* **Extremes:** resonance 1.1 without `saturate` (states in the clamp), drive 7.9, `bass_comp`, +-7.5
  octaves of noise modulation, full-scale noise, 5 s each for three modes: states within +-8, no NaN, no
  denormals; then silence at resonance 0 decays below 1e-4 with no denormals on the way.
* **Saturation** limits a drive-6 input to a peak between 0.9 and 1.05 (5.4 without it).

All 26 suites pass in the float, Q15 and Q31 host builds, and `test_ladder` also runs clean under
Clang ASan + UBSan and GCC `-O2` in all three. Not done: an SVF-style model that mirrors the control-rate
ramps and the Newton step sample by sample (the modulation tests above bound its behaviour instead),
mutation testing, and MSVC.

## Render patches (`render ladder`, measured from the WAV files)

| Patch | What it is | Measured |
|-------|------------|----------|
| `ladder_sweep.wav` | saw 110 Hz at 0.3, LP24, resonance 0.8, saturate, drive 2, ADSR-swept 5.5 octaves, VCA | peak 0.43; zero-crossing rate 230 Hz at the onset falling to 110 Hz by 0.8 s |
| `ladder_selfosc.wav` | resonance 1.1, saturate, a 0.01 click; cutoff 440 Hz, 880 Hz from 1 s | **439.97 Hz and 879.95 Hz**, level constant (0.585 after the x4 render gain) |
| `ladder_modes.wav` | saw 110 Hz at 0.3 through the eight modes, 0.4 s each, cutoff 800 Hz, resonance 0.5 | peaks 0.27-0.74, no clipping |

The 16-bit files are identical in float, Q15 and Q31.

## Known limits

* M33 cycles not measured yet (board not connected); M0+ not measured at all.
* The `tanh` is Zavalishin's "cheap" placement (after solving the linear loop), not the implicit
  nonlinear solution, and it is the only nonlinearity. Per-stage saturation (Huovilainen) was rejected
  for cost.
* Without `saturate`, resonance above 1 grows into the +-8 state clamp: hard, harsh clipping.
* `process` uses 632-648 B of stack on a Cortex-M0+.

## References

* V. Zavalishin, *The Art of VA Filter Design*, rev. 2.1.0, 2018: TPT one-pole, ZDF ladder, the cheap
  nonlinear form.
* A. Huovilainen, "Non-linear digital implementation of the Moog ladder filter," Proc. DAFx-04, 2004.
* V. Valimaki, A. Huovilainen, "Oscillator and filter algorithms for virtual analog synthesis,"
  Computer Music Journal 30(2), 2006 (pole mixing).
* T. Stilson, J. Smith, "Analyzing the Moog VCF with considerations for digital implementation,"
  Proc. ICMC, 1996.
* R. A. Moog, US patent 3,475,623, 1969.

All citations are from memory and have not been checked against the sources.
