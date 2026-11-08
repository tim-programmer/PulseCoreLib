# LFO (low-frequency oscillator)

Files: `include/pcl/pcl_lfo.h`, `src/mod/pcl_lfo.c`, `src/core/pcl_wave.h` and `src/core/pcl_octave.h`
(internal, shared with the oscillator), `tests/test_lfo.c`, the `lfo` subcommand of `tools/render.c`

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds** (Clang with ASan +
UBSan, GCC `-O2`, and the `cm0plus-*` / `cm33-*` compile checks including the no-soft-float check).
Sizes, accuracy and the **Cortex-M33 cycle counts are measured** (on a FRDM-MCXN947, bit-exact against
the host build in all three types). **Cortex-M0+ is unmeasured**: there is no hardware.

## Summary

A modulation source: it produces a normalised control signal at audio rate, to be fed into the
`pitch_mod`, `pwm_mod` or `gain_mod` inputs of the other modules (vibrato, tremolo, PWM, wobble,
random walks).

```
out[n] = shape( phase[n] ; pulse_width )          phase: uint32_t, 2^32 == one cycle
phase advances by  inc = f / sample_rate * 2^32   once per control period, f = f_base * 2^(rate_mod * depth)
```

* **Seven shapes:** sine, triangle, saw up, saw down, square/pulse, random step (sample-and-hold)
  and random smooth (linear interpolation between random values).
* **Control rate** (CLAUDE.md): the shape is evaluated once every `PCL_CONTROL_DIV` = 16 samples and
  **linearly interpolated** to audio rate in between.
* Optional **rate modulation** input in octaves, **unipolar/bipolar** output, settable **phase**,
  `retrigger()`, deterministic **seed** for the random shapes.
* One code path, a base rate plus a modulation input, like the oscillator ("VC" module).

## Decisions taken

All eight were implemented as recommended in the draft.

| # | Decision | Notes from the implementation |
|---|----------|-------------------------------|
| L1 | Control rate with linear interpolation, not audio rate. | An edge inside a control period becomes a 16-sample ramp (0.33 ms at 48 kHz); see "Edges and DC". |
| L2 | Maximum rate `sample_rate / (16 * PCL_CONTROL_DIV)` (at least 16 control points per cycle; 187.5 Hz at 48 kHz). | Config above it is an error; modulated rates clamp to it (tested). |
| L3 | All shapes aligned with the sine at phase 0. | Table below. |
| L4 | Float reference path, Q15 and Q31 paths, compared within a documented tolerance (not the integer core used for noise). | Fixed-point builds share one Q2.30 internal path; Q15 and Q31 differ only in the sine table and the output shift. |
| L5 | `rate_mod` input in octaves, sampled at each control point, no lag. | Measured: the period that reads a new modulation value already uses it (tested). |
| L6 | Sine and octave scaling moved into internal headers, used by the oscillator and the LFO. | **Verified before/after: the oscillator's output is byte-identical in all three builds** (all 25 then-existing benchmark cases compared, and every existing test passes). Its cycle counts moved by at most about 5 %; see `oscillator.md`. |
| L7 | `retrigger()` / `set_phase` take effect at the next control point; no sample-accurate event retrigger yet. | |
| L8 | No fade-in, delay or tempo sync in v1. | |

Added during implementation: `start_phase` in the config (what `retrigger()` returns to) and an
accessor `pcl_lfo_phase()` (the phase at the next control point), which also makes the frequency
tests exact.

## Parameters

| Parameter | Unit | Range | Notes |
|-----------|------|-------|-------|
| `sample_rate_hz` | Hz | 8000 .. 768000 | `uint32_t` |
| `shape` | enum | `PCL_LFO_SINE`, `TRIANGLE`, `SAW_UP`, `SAW_DOWN`, `SQUARE`, `RANDOM_STEP`, `RANDOM_SMOOTH` | Setter; phase and random state carry over; the output ramps to the new shape over one control period |
| `frequency` | Hz (`pcl_hz_t`) | 0 .. `sample_rate / 256` | Smoothed ~5 ms. 0 freezes the phase (the random shapes too). `PCL_HZ(x)` |
| `rate_depth_oct` | octaves (`pcl_oct_t`) | 0 .. < 8 | Octaves per unit of `rate_mod`. Not smoothed. `PCL_OCT(x)` |
| `pulse_width` | fraction of a cycle (`pcl_coef_t`) | 0.01 .. 0.99 | `SQUARE` only. Smoothed |
| `unipolar` | bool | 0 / 1 | 0: output in [-1, 1]; 1: output in [0, 1], computed as `(v + 1) / 2` (fixed-point formats stop just short of +1) |
| `start_phase` | `uint32_t` | any | Phase at `init`, `reset` and `retrigger()` |
| `seed` | - | any `uint32_t` | Random shapes. 0 means `PCL_NOISE_DEFAULT_SEED`. `set_seed` reseeds and resets |
| `pcl_lfo_set_phase` | `uint32_t` | any | Effective at the next control point; a deliberate jump (the output steps over one period) |
| `rate_mod` (input) | normalised | [-1, 1] | Optional; octaves = `rate_mod * rate_depth_oct`. May alias `out` |
| `out` | normalised | [-1, 1] or [0, 1] | Required |

Output never exceeds the stated range in any build (tested, including extreme parameters and the
first control period). `init` and the setters range-check (NaN included), return `PCL_ERR_*` and
leave the module unchanged; `process` trusts its arguments.

**Frequency resolution.** The phase advances by a whole number of 2^-32 cycles per control period
(rounded), so the rate is exact to one part in the advance: 1.1e-5 Hz at 48 kHz for the advance itself,
which is 0.1 % at 0.01 Hz and 1e-8 relative at 100 Hz (tested). `PCL_HZ` is Q16.16 in fixed-point
builds, a further resolution of 1.5e-5 Hz (4e-6 relative at 0.7 Hz): the frequency you ask for and the
one the module holds differ by that, and the tests compare against the quantised value.

## Algorithms

### Control period

Let `p` be the phase at the next control point (every 16 samples, counted from the start of the
stream so the output does not depend on how it is split into blocks), `v0 = shape(p)` known from the
previous period. At each tick:

1. advance the frequency and width smoothers one step (ramps are sized in ticks, so a glide still
   lasts ~5 ms), read `rate_mod` at this sample, form the period's phase advance;
2. `p1 = p + advance`, `v1 = shape(p1)` (a look-ahead, which is free for a pure function of phase);
3. output `v0 + (v1 - v0) * i / 16` for the 16 samples `i = 0 .. 15`;
4. `p = p1`, `v0 = v1`.

The interpolation lands on `v1` at the next tick. Fixed point works in Q2.30 (`pcl_coef_t`) with one
bit of headroom, so the difference of two values fits an `int32_t`: no 64-bit arithmetic on the
per-sample path, in Q15 either. The ramp step is the difference divided by 16 **truncated toward zero**,
so the ramp can never overshoot its end point (at most 15 LSB of Q2.30, 1.4e-8, short at the last sample).

### Shapes

Values at phase `p` (a fraction of a cycle):

| Shape | `shape(p)` | Notes |
|-------|-----------|-------|
| Sine | quarter-wave table, 257 entries, linear interpolation | Shared with the oscillator |
| Triangle | `4p` for p < 0.25, `2 - 4p` for p < 0.75, `4p - 4` above | Exact between control points; a corner inside a period is rounded over at most 16 samples |
| Saw up | `2 frac(p + 0.5) - 1` | Zero rising at 0, just under +1 before 0.5, then -1 |
| Saw down | `-1 - (saw up)` in fixed point (ones' complement, no overflow) | |
| Square | `+1` for p < `pulse_width`, else `-1` | Both edges smeared |
| Random step | `r` (held for the cycle) | New `r` at each phase wrap |
| Random smooth | `r_prev + (r_cur - r_prev) * p` | Piecewise linear; 64-bit product, once per control period |

Random values come from the same xorshift32 as the noise module (`core/pcl_rng.h`), uniform in
[-1, 1). They advance on **every** phase wrap whatever the current shape (so switching shape does not
change the sequence), and at frequency 0 they do not advance (tested; an early mutation `<=` for `<`
in the wrap test would have drawn a new value every period). `v1` is evaluated one control period
ahead, so a value is drawn up to 0.33 ms before it is heard.

### Edges and DC

An edge (square, saw wrap, random step) that falls inside a control period is smeared over **that whole
period**, so its timing is accurate to one period (16 samples), not to the sample. A consequence worth
knowing: when an edge lands **exactly on a control point**, the ramp occupies the period *before* it,
which biases the mean. A saw whose wrap is on a control point has a DC offset of -/+ 1 per cycle's
control points (**-1/32 for saw up at 93.75 Hz and +1/32 for saw down**, -0.0625 at the 187.5 Hz limit;
tested). At generic frequencies the edge falls at varying places inside periods and the offsets average
out. For LFO rates of a few Hz it is below -50 dB. A square with its edges on control points keeps the
right mean (`2 * width - 1`, tested).

### Interpolation distortion

A sine evaluated at `N` control points per cycle and linearly interpolated (total power of all
harmonics relative to the fundamental; `N = sample_rate / (16 f)`). Measured first with a script
before the implementation, **then reproduced to the decimal by the module's own test**:

| Control points / cycle | LFO rate at 48 kHz | Distortion | Fundamental amplitude |
|-----:|-----:|-----:|-----:|
| 16 | 187.5 Hz (the limit) | -44.5 dB | 0.9873 |
| 24 | 125 Hz | -51.6 dB | 0.9943 |
| 32 | 93.8 Hz | -56.6 dB | 0.9968 |
| 64 | 46.9 Hz | -68.7 dB | 0.9992 |
| 128 | 23.4 Hz | -80.8 dB | 0.9998 |

(The module test checks the 16, 32 and 64 rows within +/-3 dB; all three came out at -44.5, -56.6
and -68.7 dB in float, Q15 and Q31.) For vibrato and tremolo (a few Hz) it is far below audibility.

### Rate modulation

`advance = advance_base * 2^(rate_mod * rate_depth_oct)` via the shared 256-segment `2^x` table
(`core/pcl_octave.h`), clamped to the maximum. Because the next control point is computed with the
new rate, there is no lag. Tested: +1, -1, +0.5 x 2, -0.25 x 2.1 and +0.75 x 3 octaves land within
3e-4 octaves of the applied amount, the clamp holds exactly (16 control points per cycle, from both a
rate already at the limit and one pushed past it), and a modulation step is reflected in the very
next period.

## Memory (measured)

| Item | Size |
|------|-----:|
| `sizeof(pcl_lfo_t)`, all builds | **88 B** (two `pcl_smooth_t`, phase, advance state, value/ramp state, random state, flags) |
| Extra `.rodata` | **0 B**: reuses the sine and `2^x` tables the oscillator already needs |
| Code, `-O2` (`pcl_lfo.c` text) | M0+ Q15 2842 B, M0+ Q31 2858 B, M33 float 3244 B, M33 Q31 2870 B |

No heap, no caller buffers, state fully contained.

## Rate

Control rate (every `PCL_CONTROL_DIV` samples) with per-sample linear interpolation; stated in the
header. The tick counter lives in the struct.

## Cost (cycles per sample): MEASURED on a Cortex-M33

Measured on the **FRDM-MCXN947 (Cortex-M33 + FPU, 150 MHz)** with the DWT cycle counter: `-O2`, flash
cache on, 48 kHz, 256-frame blocks, the LFO alone (`scripts/mcxn947_bench.py`, cases in
`platform/bench/pcl_bench.c`; 5.3 Hz, bipolar). Fastest of 8 blocks divided by 256. Every case is
**bit-exact against the host build** in Q15 and Q31, and float matches to the last bit.

| Case | Q15 | Q31 | float |
|------|----:|----:|------:|
| `lfo_sine` | 11.9 | 11.2 | 16.7 |
| `lfo_triangle` | 10.7 | 10.6 | 16.0 |
| `lfo_saw_up` | 10.4 | 10.2 | 15.6 |
| `lfo_square` | 10.6 | 10.5 | 15.7 |
| `lfo_random_step` | 10.3 | 10.2 | 15.5 |
| `lfo_random_smooth` | 10.9 | 10.8 | 16.1 |
| `lfo_sine_rate_mod` (noise on the rate input, 2 octaves deep) | 13.8 | 13.4 | 20.9 |

* **10 to 21 cycles per sample**, i.e. 0.3-0.7 % of the core at 48 kHz (float is the dearest, see the bug fix below). The draft's estimate was
  15-30, so for once the estimate was on the high side. About 10 cycles is the interpolation loop
  itself (a store and an add per sample); the choice of shape changes the cost by at most 1.6
  cycles per sample (sine is the dearest), and rate modulation adds 2 to 4.
* **Flash cache off** (Q15): 14.5 (saw up, random step) to 17.0 (sine), 19.9 with rate modulation,
  about 40 % more; instruction fetch from flash matters here as elsewhere.
* **Cortex-M0+ is unmeasured** (no hardware); the M33 numbers do not transfer. The per-sample loop is
  32-bit only (no 64-bit on it), which is the part that matters most there; the draft's M0+ guess of
  2-3x the M33 figure remains a guess.

### A float range bug, found later (fixed)

The first float ramp accumulated (`cur += step`). Where two neighbouring control points are a few ulps
apart (the flat top of a slow sine), fifteen rounded additions could drift past the end point: **the float
LFO output reached 1.0000002**, above its documented range, in 33 samples out of 96 000 runs near sine peaks.
My range test had missed it, so the earlier statement here that the range is "tested" was too strong. This
was found while testing the ADSR, which had the same bug. Fixed-point builds were never affected (integer
adds are exact). The float ramp is now `v0 + diff * t` with `t` stepped by exact multiples of 1/16, bounded
by construction; `test_peak_never_exceeds_range` sweeps start phases across the +1 peak at six low rates, both
polarities, and **fails on the old accumulating code**. The cost is that the float LFO went from about 12 to
about 16-17 cycles per sample (a first, cruder fix measured 21.5). The board numbers above were re-measured
after the fix.

## Tests (implemented)

`tests/test_lfo.c`, 20 test groups, about 10 700 checks (10 690 float, 10 684 Q15 and Q31), in all three
sample types:

* init/setter validation (NULL, range edges, NaN/Inf in float, the 187.5 Hz limit exactly), state
  unchanged on error;
* **every shape against an independent double-precision model** of the algorithm, at 4 rates (0.7, 5.3,
  41.9, 180.3 Hz), two start phases, both polarities and a width of 0.37 / 0.8 -- sample by sample;
  tolerance is the build's own: float 2e-6, Q15 2 LSB (sine 3 LSB), Q31 3e-8, plus a bound for the
  phase advance's rounding (one count per period);
* shape facts: zero crossings and peaks at the L3 phases, saw up/down mirror images, means over whole
  cycles (sine/triangle 0, saws +/-1/32 as explained above, square `2 * width - 1`), unipolar mapping;
* frequency 0 freezes every shape; output range for every shape in both polarities at the maximum
  rate with +/-1 modulation at depth 7.9, extreme widths and 200 phase jumps; finite, no denormals;
* **block-size independence** (1, 7, 16, 17, 256, bit-identical, final phase too) for every shape with
  rate modulation, a frequency glide and a width glide active;
* `reset` and `set_seed` determinism, seed 0 equals the default, different seeds differ (random shapes)
  and are irrelevant (deterministic ones), `rate_mod` aliasing `out`;
* frequency accuracy from `pcl_lfo_phase()` from 0.01 to 187.5 Hz; octave accuracy; the clamp; the
  no-lag claim; the frequency glide (monotone, lands on target, not instant); `set_phase`,
  `retrigger` and `set_shape` (no step larger than one ramp slope); pulse-width glide;
* interpolation distortion at 16 / 32 / 64 points per cycle against the table above;
* random shapes: step holds one value per cycle (apart from the ramps), values uniform (16-bin
  histogram, 4000 cycles) with mean near 0; smooth is continuous with per-sample change bounded by the
  steepest segment;
* long runs: 1 minute per shape and 10 minutes of random smooth with noise on the rate input, finite
  and in range throughout.

**Mutation checks** on a scratch copy (saw sign, wrap detection, rate modulation read one sample late,
unclamped maximum rate, wrong period counter, wrong unipolar mapping) were all caught in every build
that contains the mutated code (the unipolar one only exists in the fixed-point path); two of them (`<=` for `<` in wrap detection, the unclamped maximum in Q15) **initially survived**,
which exposed two missing tests (random shapes at frequency 0; an explicit clamp test). Both were added
and the mutations are now caught in all three builds.

Not verified: MSVC (`/W4 /WX`).

## Render tool

`build-*/tools/render lfo <outdir> [seconds] [seed]` writes five 5-second patches (48 kHz, mono,
16-bit), all built from the library's own modules:

| File | Patch | Measured from the written file |
|------|-------|--------------------------------|
| `lfo_tremolo.wav` | saw 220 Hz through a VCA whose `gain_mod` is a 5 Hz unipolar sine LFO | envelope rms 0.004 .. 0.297 |
| `lfo_vibrato.wav` | sine 440 Hz, pitch from a 5.5 Hz sine LFO, +/-0.5 semitone | frequency 427.6 .. 452.7 Hz (predicted 427.5 .. 452.9) |
| `lfo_pwm.wav` | pulse 110 Hz, `pwm_mod` from a 0.5 Hz triangle LFO | duty cycle 0.11 .. 0.89 (predicted 0.10 .. 0.90) |
| `lfo_wobble.wav` | saw 110 Hz, pitch from a 2 Hz square LFO, +/-1 octave | two frequencies, 55 and 220 Hz |
| `lfo_random.wav` | saw 220 Hz, pitch from a 3 Hz smooth-random LFO, +/-1 octave | 119 .. 370 Hz (inside 110 .. 440) |

The same patches in Q15 differ from float by a small pitch error: Q15 cannot represent +1.0 (it is
32767/32768), so a full-scale modulation is 3e-5 octave short, which drifts the saw's edges by a
fraction of a cycle over 5 seconds (inaudible, but a sample-by-sample comparison with float shows it,
especially on `lfo_wobble.wav` and `lfo_random.wav`). Also run as a 0.2 s smoke test under `ctest`
(`tool_render_lfo`).

## References

* Linear interpolation of control signals to audio rate is standard practice; no specific paper.
* Random step / smooth shapes are the usual sample-and-hold and "smooth random" LFO waves of analogue
  and virtual-analogue synthesisers. The distortion table and the edge/DC analysis are my own.
