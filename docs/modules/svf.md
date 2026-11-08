# State-variable filter (SVF)

Files: `include/pcl/pcl_svf.h`, `src/filter/pcl_svf.c`, `tests/test_svf.c` and `tests/test_svf_internal.c`,
the `svf` subcommand of `tools/render.c`, the `svf_*` cases of `platform/bench/pcl_bench.c`.
Related additions: `pcl_tan_inc` in `pcl_math.h`; `PCL_OPAQUE_I32` and `PCL_FORCE_INLINE` in
`pcl_platform.h`.

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds** (Clang with ASan +
UBSan, GCC `-O2`, and the `cm0plus-*` / `cm33-*` compile checks including the no-soft-float check).
Accuracy, size and the **Cortex-M33 cycle counts are measured** (FRDM-MCXN947, bit-exact against the
host build in all three types). **Cortex-M0+ is unmeasured**: there is no hardware.

## Summary

The library's main voltage-controlled filter (VCF): a resonant filter that outputs low-pass,
band-pass and high-pass at once, with an exponential (1 V/oct style) cutoff modulation input.

```
        hp = a1 * (x - c*s1 - s2)         a1 = 1 / (1 + g*(g+k))     c = g + k
        bp = g*hp + s1        s1' = bp + g*hp        g = tan(pi * fc / fs)   (pcl_tan_inc)
        lp = g*bp + s2        s2' = lp + g*bp        k = 1/Q                 (damping)
```

The topology-preserving-transform (trapezoidal) SVF of Zavalishin: the analogue prototype with its
integrators discretised by the bilinear transform. **The cutoff is exact (no frequency warping), the
resonance does not detune it, and the response matches the analogue formulas to 0.003 dB.** Four
multiplications per sample give all three outputs.

* **Outputs:** low-pass, band-pass, high-pass from one call (any may be `NULL`); plus a single-output
  call with seven responses (LP, BP, normalised BP, HP, notch, peak, all-pass).
* **Cutoff** 5 Hz .. 0.45 * sample rate, smoothed over ~5 ms in the octave (log) domain; **resonance**
  0 .. 1 from Butterworth (Q = 0.71) up to self-oscillation at 1.
* **Cutoff modulation input** (`cutoff_mod`, normalised [-1, 1]) times a depth in octaves, like the
  oscillator's pitch input. **Control rate**: evaluated once per `PCL_CONTROL_DIV` = 16 samples; the
  filter itself runs at audio rate.
* **Bounded:** the states are limited to +-8 (hard clamp), optionally with a `tanh` soft limiter.

## Decisions taken

The draft's recommendations were all accepted. What changed while building them:

1. **Resonance `k = sqrt(2) (1 - r)^2`**, as drafted (Q 0.71 / 2.8 / 17.7 / 71 / infinity at r = 0 / 0.5 /
   0.8 / 0.9 / 1).
2. **State limiting:** hard clamp always, `saturate` flag for the `tanh` limiter on the band-pass state.
3. **Outputs:** the three-output `pcl_svf_process` and `pcl_svf_process_mode`, as drafted.
4. **Control-rate coefficients: `a1` is NOT interpolated linearly. This was the one thing the draft got
   wrong, and the tests found it** (see "A stability bug found by the tests" below). `g` and `c` are
   ramped linearly across the control period; `a1` follows `1/(1 + g c)` sample by sample with one
   Newton step, anchored to the exact value (one division) at the end of each period.
5. **32-bit states in every build**, as drafted. Fixed point: states, `g`, `c` Q4.28; `a1` Q2.30.
6. **`pcl_tan_inc(phase_increment)` added to `pcl_math.h`** and `pcl_tan_hz` now calls it. Verified
   bit-identical: all 48 earlier benchmark cases are byte-identical before and after, and the new
   `test_tan_inc` checks equality with `pcl_tan_hz` and the clamp.
7. **Out of scope**, as drafted.

Other departures from the draft, all small:

* **Float outputs are not clamped** (the draft said the filter stays within +-8). The *states* are
  clamped to +-8; with finite input the float outputs are finite and follow them, 8.9 at worst in the
  nastiest case tested (a self-oscillating filter under white-noise modulation of +-6 octaves). Fixed
  point saturates its outputs at full scale as always. Clamping three float outputs per sample cost
  about 35 cycles on the M33, which was not worth it. Non-finite input is not supported (as for the
  VCA), but it does not poison the filter: the states are clamped, so the output is finite again within
  two samples.
* **Float denormals use an anti-denormal guard** (1e-24 added to each state, -480 dBFS) instead of a
  compare-and-flush: the compare cost a flag transfer from the FPU per state per sample. The decay
  therefore ends at ~1e-24, not at exactly 0.
* **The highest accepted cutoff is 0.45 * fs truncated to Q16.16**: 21599.87 Hz at 48 kHz, so
  `pcl_svf_set_cutoff(21600)` is refused in fixed point (float accepts exactly 0.45 fs).
* **In fixed point `hp` is computed as `a1 (x - s2) - (a1 c) s1`** with `a1 c` kept as a coefficient
  (it is at most about 1.5). The obvious `a1 (x - c s1 - s2)` needs `t = x - c s1 - s2`, which reaches
  70 (c is up to 7.7) and does not fit a 32-bit register; clamping it to +-8 changed the cutoff gain
  by 6 % near Nyquist (a test caught this). Float keeps the direct form.
* **Code size is 2-3 times my estimate** (below) and the **cost 3.5-5 times** for fixed point.

## A stability bug found by the tests

The draft ramped `g`, `c` and `a1` linearly across the control period, and a prototype said the result
differed from exact per-sample coefficients by -26 to -59 dB. That measured the *size* of the
difference and not its *sign*. `1/(1 + g c)` is convex in time, so the straight line between its two
ends lies *above* it: `a1` is too large, which is negative damping. Under large cutoff jumps the
filter then grows exponentially. The time-domain model test exposed it as a 100 % mismatch, and a
standalone experiment (no clamp, noise then silence, the cutoff re-drawn every control period):

| cutoff jump per period | linear `a1` (as drafted) | exact `a1` per sample | Newton step (as built) |
|---|---|---|---|
| +-2 octaves | bounded (2.5) | 2.5 | 2.5 |
| +-4 octaves, Butterworth | **blows up** | 0.58 | 0.61 |
| +-4 octaves, Q = 17 | **blows up** | 3.3 | 4.1 |
| +-6 octaves, Butterworth | **blows up** | 0.59 | 0.6 |

Quadratic interpolation through three exact points, a linear ramp with a Newton correction, and
exact values every 4 or 8 samples all blew up too in some cases. What works is a Newton step from
the *previous sample's* `a1`: `a1 <- a1 (2 - d a1)` with `d = 1 + g c`. A tracking error never
accumulates, because Newton's method contracts toward `1/d`. `d` is quadratic in the sample index, so
two additions per sample (forward differences) provide it with no multiplication. The cost is two
multiplications per sample while the coefficients move, none otherwise. A second Newton iteration
brings the worst-case error from 18 % to 4 % but is not needed for stability.

Caveat: at `k = 0` (resonance 1) the *exact* per-sample filter also gains energy from fast
modulation (parametric pumping: 12-45 times full scale in the experiment), so the state limit is
required there, not optional. A test fixes the regression (`test_heavy_modulation_needs_no_clamp`:
Butterworth under +-7.5 octaves of white-noise modulation every 16 samples must keep the states
below 2.5, where the linear scheme ran into the +-8 clamp).

## Parameters

| Parameter | Type / format | Range | Notes |
|-----------|---------------|-------|-------|
| `sample_rate_hz` | `uint32_t` | `PCL_SAMPLE_RATE_MIN_HZ` .. `MAX_HZ` | |
| `cutoff_hz` | `pcl_hz_t` | `PCL_SVF_CUTOFF_MIN_HZ` (5) .. 0.45 `sample_rate_hz` | smoothed in octaves over `PCL_SMOOTH_MS` |
| `resonance` | `pcl_coef_t` | 0 .. 1 | smoothed (linear in `k`); 1 = self-oscillation |
| `mod_depth_oct` | `pcl_oct_t` | 0 .. < 8 | octaves of cutoff per unit of `cutoff_mod`; not smoothed |
| `saturate` | `int` | 0 / non-zero | `tanh` state limiter |

The modulated cutoff is clamped to [3.9 Hz, 0.45 fs]. Setters validate (`PCL_ERR_OUT_OF_RANGE`, NaN
included) and leave the module unchanged on error. Q4.28 cannot even express a depth of 8 octaves, so
only the float build can be given an out-of-range depth.

## Algorithm

### Per sample

`x`, `s1`, `s2`, `g`, `c` are Q4.28 (`pcl_wide_t`), `a1` and `ac = a1 c` Q2.30 (fixed point):

```
hp = a1 * (x - s2) - ac * s1          (one 64-bit accumulator, one rounding)
gh = g * hp           bp = gh + s1    (saturated: bp feeds a product)
gb = g * bp           lp = gb + s2
s1' = bp + gh   s2' = lp + gb         (saturated to +-8; with saturate: s1' = tanh(s1'))
```

Intermediate values stay in 64 bits and are saturated only where they must be: with the states within
+-8, `|hp| < 21` and `|g hp| < 10` for every coefficient set the tick can produce, so nothing
overflows. `x`, the outputs and the saturated values are converted by `svf_in` / `svf_out`.

Exact transfer functions, with `u = tan(pi f / fs) / g`: `H_lp = 1 / (1 - u^2 + j k u)`, `H_bp =
j u H_lp`, `H_hp = -u^2 H_lp`; at `f = fc`, `|H_lp| = |H_bp| = 1/k = Q`. The tests compare against them.

### Control tick (every 16 samples, counted from the start of the stream)

1. Advance the cutoff and resonance smoothers a period (only while one is moving).
2. `oct = smoothed_cutoff_oct + cutoff_mod * mod_depth` (Q4.28), the modulation sample at the tick.
3. `inc = octave_scale(inc_1k, oct)` clamped at 0.45 * 2^32, `g = pcl_tan_inc(inc)`; `c = g + k`.
4. `a1_end = 1 / (1 + g c)` by one 64-bit division, `ac_end = a1_end c_end`.
5. Per-sample steps of `g` and `c`, and the forward differences of `d`; at the end of the period all
   coefficients are set exactly to the targets, so rounding never accumulates.

With no modulation input, settled smoothers and no coefficients left over from a modulated period,
the tick does nothing and the sample loop carries no ramp state. If the previous period was
modulated but this one is not, the tick runs once more so the cutoff comes back to its base value
(`test_cutoff_returns_after_modulation`). The coefficients lag the control signal by up to one period
(0.33 ms at 48 kHz): bandwidth about `fs / 32`.

### Code structure

The sample loop is one body, `PCL_FORCE_INLINE`d twice with a constant argument (moving or not), so
the common static case does not carry the ramp state in registers. State and coefficients live in
locals for the loop, since the output pointers could alias the struct as far as the compiler knows (in
Q31 both are `int32_t`). The saturation helper is wrapped in `PCL_OPAQUE_I32` (an empty `asm`, only
in `pcl_platform.h`): without it GCC folds the 64-bit saturation into the next multiplication and
emits a full 64x64 product (three multiplies and spills) for what is a single `smull`; with it the
core loop shrank from 159 to 108 instructions.

## Accuracy (measured)

The response of the module against the exact analogue formulas (`tests/test_svf.c` repeats this for
two sample rates, five cutoffs from 20 Hz to 19 kHz, three resonances and seven frequencies from 0.1
to 4 times the cutoff):

| Build | worst magnitude error | worst phase error | cutoff error (from the band-pass phase at fc) |
|-------|----------------------:|------------------:|-----------------------------------------------|
| float | 0.0008 dB | 0.0002 rad | 0.009 cents |
| Q31 | 0.0006 dB | 0.0002 rad | 0.011 cents |
| Q15 | 0.0023 dB | 0.0020 rad | 0.10 cents (limited by the 16-bit measurement) |

The draft predicted "0.05 cents"; float and Q31 are 5 times better. The state precision table from the
draft stands: 28 fractional bits give 93-101 dB SNR at a 20 Hz cutoff and 115 dB or more from 100 Hz,
against 0-25 dB at 20 Hz for 16-bit states.

## Memory (measured)

* **State:** `sizeof(pcl_svf_t)` = **116 bytes** in all three builds (the draft estimated 110).
* **Tables:** none of its own.
* **Code (`pcl_svf.o`, text):** Cortex-M0+ Q15 6474 B, Q31 6510 B; Cortex-M33 float 4046 B, Q31 6760 B
  (the draft estimated 1.5-2.5 KB). The force-inlined body twice, three output paths and the seven
  modes account for most of it. The M0+ objects reference only `__aeabi_lmul` and
  `__aeabi_uldivmod`; no soft-float routine (the build's check enforces it).

## Rate

The filter runs at **audio rate**; its coefficients update at **control rate** (every
`PCL_CONTROL_DIV` samples). The header says so.

## Cost (cycles per sample): MEASURED on a Cortex-M33

Measured on the **FRDM-MCXN947 (Cortex-M33 + FPU, 150 MHz)** with the DWT cycle counter, `-O2`, from
flash with the SDK's default flash cache on, 48 kHz, 256-frame blocks of noise in (and noise as the
modulation signal where used), cutoff 1.2 kHz, resonance 0.6. Bit-exact against the host build in
all three types; spread between blocks under 1 %.

| Case | Q15 | Q31 | float | Draft estimate (Q15 / Q31, float) |
|------|----:|----:|------:|----------------------------------:|
| static cutoff, 3 outputs | 165 | 155 | 64 | 30-45, 25-40 |
| static cutoff, single mode (notch) | 157 | 157 | 75 | |
| `cutoff_mod` connected | 240 | 232 | 85 | 50-70, 40-55 |
| `saturate` on | 198 | 190 | 111 | +25-35, +30-40 |
| `saturate` + `cutoff_mod` | 273 | 267 | 131 | |

* **Fixed point is 3.5-5 times my estimate; float 1.6-2 times.** One voice is **5 % of the core in Q31
  at 48 kHz (3125 cycles per sample are available), 7.4 % with modulation**, and 2 % / 2.7 % in
  float. Eight modulated Q31 voices would take 60 % of the core.
* Where the cycles go: the core arithmetic is about 108 instructions per sample after optimisation
  (a micro-benchmark of that loop alone); the rest is output conversion, branches on the optional
  outputs, and loads and stores of the state (the register file is too small for it). Connecting
  `cutoff_mod` adds 77 cycles per sample (Q31: 232 vs 155): the control tick (a 64-bit division,
  `pcl_tan_inc`, the octave scaling) plus the Newton step on every sample while the coefficients move;
  I did not separate the two. `saturate` adds 33-47 cycles (the cost of `pcl_tanh`).
* **Optimisation steps that mattered:** removing five of the six 64-bit saturations (217 -> 172
  cycles), `PCL_OPAQUE_I32` (194 -> 166, after a reformulation that had cost 22), splitting the
  static and ramping loops (166 -> 155); in float, replacing the clamp-and-flush compares (each a
  flag transfer from the FPU) cut 111 to 64.
* **Cortex-M0+ is unmeasured.** There the 64-bit products are library calls (`__aeabi_lmul`), about
  four to six per sample plus the Newton step, so I expect several hundred cycles per sample, which
  is 15-25 % of a 48 MHz part per voice. Treat a Q15 filter on an M0+ as expensive; I have not
  verified the number, and a cheaper variant (lower-precision states) would be a separate decision.

## Tests (`tests/test_svf.c`, 22 groups in fixed point and 24 in float, ~45,000 checks; `tests/test_svf_internal.c`, 43 / 74 / 74 checks)

* **Init and setters:** valid and invalid configs (NaN, rates out of range, cutoff below 5 Hz or above
  0.45 fs, resonance outside 0..1, NULL), module unchanged on error.
* **Response against the analogue formulas:** magnitude and phase of LP, BP and HP at 7 frequencies, 5
  cutoffs, 3 resonances and 2 sample rates; **cutoff tuning** from the band-pass phase at fc; the Q at
  the cutoff for r = 0, 0.5, 0.8; the all-pass has unit gain at five frequencies; the derived modes
  equal the sums of the three outputs sample by sample.
* **Time-domain model** (double precision) that follows the control-rate ramps, the Newton step, the
  setters called between blocks and the smoothers: tight against a model that mirrors the Newton step
  (1e-5 to 1e-4 relative) and loose against one with exact per-sample `a1` (bounds what the Newton step
  gives up: 2 % for moderate modulation). Chaotic cases (high Q, violent modulation) cannot be
  compared sample by sample and are covered by bounds instead.
* **Determinism and block size:** bit-identical output for blocks of 1, 7, 16, 17, 100, 256 and 1000
  frames, with and without modulation and with a smoothing ramp in progress; `reset` gives the same
  output again; in-place aliasing of the input with each output; `NULL` outputs; a modulation input of
  zeros is bit-identical to none.
* **Modulation:** nothing changes before the first control point after the modulation steps (bit for
  bit); a constant modulation equals a filter set that many octaves up; the cutoff returns to its
  base after modulation stops; the coefficients land exactly on their targets at the end of every
  period; the cutoff clamps at the range ends.
* **Smoothing:** after 7 control points a 4-octave `set_cutoff` is the right *fraction of the way in
  octaves* (not in Hz) to 3 cents; a sweep produces no click (sample step bounded by the input's).
* **Stability:** self-oscillation holds its level and frequency for 2 s (amplitude within 10 %, 2000
  zero crossings in 1 s); a 10-minute run per build with `k` = 0 and full-scale random cutoff
  modulation of +-6 octaves, then 60 s runs at r = 0.97, 0.999 and with `saturate`: states always
  within +-8, `saturate` holds the band-pass state to +-1; decay to the noise floor after the input
  stops (-120 dBFS; 2 LSB in Q15); float denormals never appear in 20 s of decay; garbage (NaN, Inf,
  1e30) in float input is finite again within two samples.
* **White-box (`test_svf_internal.c`, which includes the source):** the saturation helper at every
  edge, the sample/wide conversions, the Newton step (convergence, no negative `a1`), `a1 c`, the
  resonance and cutoff mappings, and the target coefficients against `tan` and `1/(1 + g c)`.

### Mutation testing

38 deliberate bugs, one at a time, in scratch copies (damping and cutoff mappings, the Newton sign and
removal, `a1 c` tracking, smoother advance, the stale-coefficient flag, the period-end snap, the
control-period counter, the modulation sign, the `saturate` flag, the sign of the `hp` terms, the
all-pass / notch / normalised band-pass formulas, state limiting, range checks, `reset`, the second
difference of `d`, the 1 kHz reference):

* **34 are caught.** The first run left 11 alive. Real test gaps, now closed: the cutoff did not
  return to base after modulation (a stale-coefficient bug); nothing checked the period-end snap; the
  fixed-point all-pass and notch had been mutated in dead code (my pattern matched the float copy
  first), and with the right target both are caught.
* **Four stay alive and are not defects:** removing the anti-denormal guard from *one* state is harmless
  (the other state's guard keeps the coupled loop out of the denormal range; removing both is caught,
  with 962,505 subnormal samples); forcing the Newton step on when the coefficients do not move changes
  nothing in float (Newton's fixed point is exact there); and **the one that matters, removing the
  fixed-point saturation of `s1`** (it wraps instead): a wrapped state is still numerically within +-8,
  so no test of the filter's behaviour can see it, and the white-box test checks the helper but not
  that the update uses it. It is a guard that only acts in the clamp regime.

## Render patches (`render svf`), measured from the WAV files

| Patch | What it is | Measured |
|-------|------------|----------|
| `svf_sweep.wav` | saw 110 Hz, low-pass, resonance 0.7, swept by an ADSR | spectral centroid 810 Hz at the onset, falling to 115 Hz over 0.9 s |
| `svf_wah.wav` | white noise, normalised band-pass, resonance 0.85, LFO 1.2 Hz +-2 octaves | spectral peak follows the LFO: correlation 0.999 in log frequency, median error 0.06 octave |
| `svf_selfosc.wav` | impulses into resonance 1.0, cutoff 440 Hz then 880 Hz at 1 s | **440.00 Hz and 880.00 Hz**, amplitude constant (0.311 / 0.433) |
| `svf_voice.wav` | oscillator -> filter -> VCA, two ADSRs, 8 notes | **fundamentals within 0.14 cents of the MIDI notes** (via `pcl_midi_to_hz`); filter closes 899 -> 350 Hz over the first note |

Peaks are 0.43-0.49 in float, Q15 and Q31 (no clipping). The saw is attenuated to 0.15 before the
filter, because in a fixed-point build the filter output saturates at 1.0 before any output gain: a
resonant filter needs input headroom. (My first levels clipped the PCM at full scale, and the first
pitch measurement, by autocorrelation, was 9-20 cents flat because the resonance biases the apparent
period; the spectral peak gives the figures above.)

## Known limits

* Cortex-M0+ cost is unmeasured and probably high (above).
* At resonance above about 0.5, violent continuous modulation (white-noise modulation of +-7 octaves
  every 16 samples) drives even the exact filter into parametric pumping and the +-8 limit. Envelopes,
  LFOs and key tracking do not.
* In a Q15 build `saturate` limits the band-pass state to 15-bit precision (the `tanh` output is a
  Q15 sample): about -85 dB noise while it is on.
* The cutoff modulation is sampled once per 16 samples: no audio-rate FM of the cutoff.
* `cutoff_hz` above 0.45 fs is refused; near that limit the filter is accurate but the response of
  any bilinear filter is strongly warped relative to a real analogue one.

## References

* Zavalishin, V. *The Art of VA Filter Design* (Native Instruments), chapters 3-4: the
  topology-preserving transform and the state-variable filter, including the direct-form equations
  used here and the discussion of modulation.
* Simper, A. (Cytomic), *Solving the continuous SVF equations using trapezoidal integration and
  equivalent currents*: the alternative (5-multiplication) form, used as the reference in the
  prototype.
* Chamberlin, H. *Musical Applications of Microprocessors* (1980): the older SVF this replaces, for
  its limits at high cutoff.
