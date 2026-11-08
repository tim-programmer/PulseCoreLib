# Biquad filter

Files: `include/pcl/pcl_biquad.h`, `src/filter/pcl_biquad.c`, `tests/test_biquad.c` and
`tests/test_biquad_internal.c`, the `biquad` subcommand of `tools/render.c`, the `biquad_*` cases of
`platform/bench/pcl_bench.c`. `include/pcl/pcl_types.h` gained `pcl_quality_t` / `PCL_QUALITY`.

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds** (Clang with ASan +
UBSan, GCC `-O2`, and the `cm0plus-*` / `cm33-*` compile checks including the no-soft-float check).
Accuracy, size and the **Cortex-M33 cycle counts are measured** (FRDM-MCXN947, bit-exact against the
host build in all three types). **Cortex-M0+ is unmeasured**: there is no hardware.

## Summary

The workhorse second-order filter for tone shaping and EQ: eight responses from one structure, the
**RBJ "Audio EQ Cookbook" set**:

| Type | Response |
|------|----------|
| `LP` / `HP` | 12 dB/oct low-pass / high-pass, resonance `Q` |
| `BP` | band-pass, 0 dB gain at the centre (constant peak gain) |
| `NOTCH` | notch |
| `ALLPASS` | phase-shifting all-pass |
| `PEAK` | peaking EQ: `gain_db` boost or cut at the centre, bandwidth by `Q` |
| `LOWSHELF` / `HIGHSHELF` | shelving EQ: `gain_db` below / above the corner, steepness by `Q` |

```
y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] - a1 y[n-1] - a2 y[n-2]        (Direct Form I)
```

The coefficients are the bilinear transform of each type's analogue prototype, evaluated with
`K = tan(pi * f / fs)` (`pcl_tan_inc`, as the state-variable filter does). **They are the cookbook's:** in
the prototype the generic bilinear form of the cookbook's analogue prototypes equalled the cookbook's
digital formulas to 3e-10 dB for all eight types, and the tests compare the module's coefficients with
an independent implementation of the cookbook (below).

**Where it sits next to the SVF.** The SVF is the voltage-controlled filter: stable under fast cutoff
modulation and equally accurate at every cutoff. The biquad is the cheap filter for parameters that hold
still or move smoothly: EQ, tone, crossovers, fixed formant filters. It has a cutoff modulation input
for completeness, good for smooth modulation only.

* **Audio-rate filter, control-rate coefficients** (once per `PCL_CONTROL_DIV` = 16 samples, counted
  from the start of the stream so the output does not depend on block splits), linearly interpolated.
* **Smoothed setters** (about 5 ms): cutoff and Q in octaves, gain in dB. A type change is reached by a
  ramp over one control period with the history kept.
* Any `frames >= 1`, in-place safe, one mono stream per instance.

## Decisions taken

The draft's recommendations were all accepted. What changed while building them:

1. **Topology: Direct Form I with error feedback in fixed point, plain in float**, as drafted. The
   32-bit history is Q4.28; all five products go into one 64-bit accumulator, one saturation per sample.
2. **`pcl_quality_t` / `PCL_QUALITY(x)` added** (float, or unsigned Q16.16; the representation of
   `pcl_hz_t` so `pcl_hz_to_oct` applies). Q range 0.2 .. 100.
3. **Cutoff modulation input included.** In float the output history is clamped to +-8 (and carries an
   anti-denormal guard) so even a filter driven unstable cannot reach NaN/Inf; fixed point saturates.
4. **Type changes** by a one-period coefficient ramp with the history kept (tested).
5. **Formats** as drafted: history Q4.28, `b` Q5.27, `a` Q2.30, gain +-18 dB.
6. **Float at low cutoffs: a documented limitation**, now measured (below).

What the draft did not anticipate:

* **The coefficient design is much more careful than "one 64-bit division".** With `a1` near -2 at low
  cutoffs, the cutoff is only right if `1/D0` is accurate to about 1e-10 (32 bits), which a Q4.28
  pipeline cannot give: a 28-bit reciprocal moves a 20 Hz cutoff by about a cent. So the polynomial
  values are held in Q.48, multiplied exactly in 128 bits by a small portable helper
  (`bq_mulshr`, 64x64 -> 128 with a shift, verified against `__int128`), and `1/D0` is refined by a
  Newton step to about 54 bits. One 64-bit division, then multiplications. The result reproduces the
  exact coefficients to within one LSB of the format from 7 Hz up (`test_design_precision_at_low_cutoff`).
* **Code size is 2-3 times the estimate** (below), and the cost of the moving case higher.
* **Float outputs are not clamped**, only the history (as for the SVF): with finite input they stay
  finite.

## Parameters

| Parameter | Type / format | Range | Notes |
|-----------|---------------|-------|-------|
| `sample_rate_hz` | `uint32_t` | `PCL_SAMPLE_RATE_MIN_HZ` .. `MAX_HZ` | |
| `type` | `pcl_biquad_type_t` | the eight above | |
| `cutoff_hz` | `pcl_hz_t` | `PCL_BIQUAD_CUTOFF_MIN_HZ` (5) .. 0.45 `sample_rate_hz` | corner, centre or cutoff; smoothed in octaves. Full accuracy from 20 Hz |
| `q` | `pcl_quality_t` | 0.2 .. 100 | smoothed in octaves of Q |
| `gain_db` | `pcl_db_t` | -18 .. +18 dB | `PEAK`, `LOWSHELF`, `HIGHSHELF`; stored but unused by the others; smoothed linearly in dB |
| `mod_depth_oct` | `pcl_oct_t` | 0 .. < 8 | octaves of cutoff per unit of `cutoff_mod` |

The modulated cutoff is clamped to [3.9 Hz, 0.45 fs]. The highest accepted cutoff is 0.45 fs truncated
to Q16.16 in fixed point (21599.87 Hz at 48 kHz). Setters validate (`PCL_ERR_OUT_OF_RANGE`, NaN
included) and leave the module unchanged on error.

## Algorithm

### Coefficients (control tick, every 16 samples, and at init)

Each type is an analogue prototype `H(s) = (n2 s^2 + n1 s + n0) / (d2 s^2 + d1 s + d0)` with `k = 1/Q`,
`A = 10^(gain_db/40)`, `r = sqrt(A)`:

| Type | `n2 n1 n0` | `d2 d1 d0` |
|------|-----------|-----------|
| LP | 0, 0, 1 | 1, k, 1 |
| HP | 1, 0, 0 | 1, k, 1 |
| BP | 0, k, 0 | 1, k, 1 |
| NOTCH | 1, 0, 1 | 1, k, 1 |
| ALLPASS | 1, -k, 1 | 1, k, 1 |
| PEAK | 1, A k, 1 | 1, k/A, 1 |
| LOWSHELF | A, r k A, A^2 | A, r k, 1 |
| HIGHSHELF | A^2, r k A, A | 1, r k, A |

With the bilinear transform `s = (1/K)(1 - z^-1)/(1 + z^-1)`: `N0 = n2 + n1 K + n0 K^2`, `N1 = 2(n0 K^2 -
n2)`, `N2 = n2 - n1 K + n0 K^2`, and the same for `D`; `b = N / D0`, `a1 = D1 / D0`, `a2 = D2 / D0`.
`1/Q` comes from the octave-domain Q smoother by `2^(-8u)`, so the Q setter needs no division per tick;
`sqrt(A)` and `1/A` from two `pcl_db_to_gain` calls at `dB/4` (every legal gain gives a result under 2,
which Q2.30 can hold). The coefficients are ramped linearly across the next period (five additions per
sample while moving).

### Per sample (fixed point)

```
acc = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2 + err      (64-bit, scaled 2^-55)
y   = round(acc) to Q4.28, saturated;     err = acc - y*2^27   (0 if saturated)
```

Direct Form I never overflows an intermediate (it is a single sum), so there is one saturation per
sample, made opaque (`PCL_OPAQUE_I32`) so the compiler keeps single `smull`s. The state lives in locals
for the loop, and the body is `PCL_FORCE_INLINE`d twice (coefficients moving or not), as in the SVF.
Float: the same without `err`; `y1` is clamped to +-8 with a 1e-24 guard.

## Accuracy (measured)

### The coefficients against the cookbook

Worst `|H_module - H_cookbook| / (|H_cookbook| + 0.001)` in dB, over 120 log-spaced frequencies from 2 Hz
to 0.49 fs, for Q = 0.2 .. 10 and gains -18 .. +18 dB (the 0.001 floor, -60 dB, keeps the deep stopband
from dominating: the feedforward coefficients' 7.5e-9 resolution leaves a DC leak around -84 dB in a
high-pass). The tests enforce these as tolerances by cutoff:

| cutoff | Q31 / Q15 | float |
|-------:|-----------|-------|
| 1 kHz and up | 0.0001 .. 0.004 dB | 0.0001 .. 0.005 dB |
| 100 Hz | 0.001 .. 0.03 dB | 0.05 .. 0.2 dB |
| 20 Hz | 0.004 .. 0.2 dB (a high-pass 0.8 dB) | 0.14 .. 3 dB |
| 5 Hz | 0.01 .. 1.5 dB | 8 .. 22 dB: **unusable** |

(At a Q of 100 the narrow resonance magnifies any tuning error: at 100 Hz, float shelves reach 0.5 .. 0.9 dB and the fixed-point high-pass 0.27 dB.)

### Precision of the arithmetic

SNR of a low-pass output against a double-precision filter with the module's own coefficients, -6 dBFS
noise plus a sine in:

| fc | Q | Q31 (Q4.28 history, error feedback) | Q31 without error feedback (prototype) | float32 |
|---:|--:|------------------------------------:|---------------------------------------:|--------:|
| 20 Hz | 0.71 | **111 dB** | 59 | 70 |
| 20 Hz | 8 | 110 | 60 | 59 |
| 100 Hz | 0.71 | **132** | 94 | 90 |
| 100 Hz | 8 | 127 | 89 | 80 |
| 1 kHz | 0.71 | 159 | 142 | 121 |
| 8 kHz | 0.71 | 163 | 164 | 144 |

(Q15 builds have the same 28-bit history; their 16-bit *output* limits what can be seen, so the test
checks that the output is the correctly rounded result within 1.5 LSB, and reads the history itself
for the SNR.) **Float's arithmetic alone is acceptable; its weak point is the coefficients:** a float
`a1` near -2 resolves 1.2e-7, which at 20 Hz is a cutoff error of order a percent. Below about 100 Hz at
48 kHz (200 Hz for shelves) use a fixed-point build or the SVF.

## Memory (measured)

* **State:** `sizeof(pcl_biquad_t)` = **160 bytes** in all three builds (the draft estimated 170).
* **Tables:** none of its own.
* **Code (`pcl_biquad.o`, text):** Cortex-M0+ Q15 8706 B, Q31 8726 B; Cortex-M33 float 4328 B, Q31
  8222 B (the draft estimated 3-5 KB). In the M33 Q31 object: `bq_design` 2.1 KB, the 128-bit helper
  1.2 KB, the two sample loops about 1 KB each. Not an inlining effect (marking the helper `noinline`
  changed nothing, so I reverted it). The M0+ objects reference only `__aeabi_lmul`,
  `__aeabi_uldivmod` and `__aeabi_ldivmod` (a constant division in a setter); no soft-float routine.

## Rate

The filter runs at **audio rate**; its coefficients update at **control rate**. The header says so.

## Cost (cycles per sample): MEASURED on a Cortex-M33

Measured on the **FRDM-MCXN947 (Cortex-M33 + FPU, 150 MHz)** with the DWT cycle counter, `-O2`, from
flash with the SDK's default flash cache on, 48 kHz, 256-frame blocks of noise in; LP or peaking
(1.2 kHz, Q 2, +6 dB); the modulated case has noise on `cutoff_mod` with a 2-octave depth; the "moving"
case calls `set_cutoff` before every block, so the coefficients are always gliding. Bit-exact against
the host build in all three types; spread between blocks under 1 %.

| Case | Q15 | Q31 | float | Draft estimate (Q15 / Q31, float) |
|------|----:|----:|------:|----------------------------------:|
| static LP | 71 | 79 | 34 | 60-110, 30-60 |
| static peaking | 71 | 79 | 34 | |
| `cutoff_mod` connected | 179 | 196 | 64 | 100-170, 50-90 |
| coefficients gliding (`set_cutoff` every block) | 200 | 217 | 92 | 100-170, 50-90 |

* **Static: inside the estimate.** One biquad is 2.5 % of the core at 48 kHz in Q31 (3125 cycles per
  sample are available) and 1.1 % in float: about half the SVF's cost (155 / 64) for the same job
  without modulation.
* **Moving: above the estimate in fixed point.** Connecting `cutoff_mod` adds 117 cycles per sample in
  Q31 (196 vs 79), a gliding cutoff 138: the five coefficient additions per sample are a small part of
  it; the rest is the coefficient design (`pcl_tan_inc`, the exp2 for `1/Q`, up to two dB conversions,
  about a dozen 128-bit multiplies, one division), roughly 1,700-2,000 cycles per control tick, which
  I did not separate more finely. A modulated Q31 biquad is 6.3 % of the core, a gliding one 6.9 %.
  Float designs far more cheaply: it adds 30 cycles per sample (64 vs 34).
* **Cortex-M0+ is unmeasured.** The 128-bit helper becomes four `__aeabi_lmul` calls per use there, and
  the per-sample path has five 32x32 -> 64 products (library calls): expect several hundred cycles per
  sample, and a design that costs thousands of cycles per tick. Treat the moving case on an M0+ as
  expensive; I have not verified it.

## Tests (`tests/test_biquad.c`: 18 groups in each build, ~1,550 checks; `tests/test_biquad_internal.c`: 39 / 38,409 / 38,409 checks)

* **Init and setters:** valid and invalid configs (NaN, rates, cutoff, Q, gain, type, depth, NULL); the
  module is unchanged on error; limits accepted.
* **Coefficients against an independent RBJ implementation:** all eight types x 6 cutoffs (5 Hz to 19 kHz)
  x 4 Qs x 4 gains, 120 frequencies each, with the tolerances above by cutoff; and at 8, 44.1, 96 and
  192 kHz.
* **Gain landmarks:** the peaking gain at the centre is `gain_db` to 0.01 dB for six gains; shelf gains
  at DC and Nyquist; band-pass 0 dB at the centre for Q 0.5 .. 50; all-pass flat; notch null.
* **The running filter** against the cookbook: steady-state sine measurements of magnitude and phase at
  seven frequencies for every type.
* **Precision:** the SNR figures above as thresholds (111 dB at 20 Hz in Q31, 38 dB for float at 20 Hz
  as regression bounds); in Q15 the output within 1.5 LSB; `test_history_precision` reads the 28-bit
  history after every sample (the only way to see the error feedback in a 16-bit build).
* **Determinism and block size:** bit-identical output for blocks of 1, 7, 16, 17, 100, 256 and 1000
  frames, with and without modulation and with a ramp in progress; `reset`; in-place aliasing; a
  modulation input of zeros is bit-identical to none.
* **Smoothing and modulation:** a 3-octave `set_cutoff` is the right fraction of the way in *octaves*
  after 7 control points; the gain glides linearly in dB; a Q change lands on the exact coefficients; a
  type change keeps the history and reaches the new coefficients in one period; a mid-period modulation
  step changes nothing until the next control point (bit for bit); a constant modulation equals a
  cutoff that many octaves up; the coefficients land exactly on their targets at every period end; the
  cutoff returns to base after modulation stops.
* **Time-domain model** (double precision, with the control-rate ramps, the smoothers and a mid-run
  `set_cutoff`) for all eight types: below 6e-4 relative in fixed point, 1e-3 in float.
* **Stability:** smooth modulation (a product of two slow sines, +-2 octaves) at Q 2, 8 and 17 for 60 s per
  build stays bounded; violent modulation (full-scale white noise, +-6 octaves, Q 70) for 60 s: no
  NaN/Inf and the float history within +-8; float denormals never appear over 20 s of decay; fixed point
  decays to the floor.
* **White-box (`test_biquad_internal.c`, which includes the source):** the 128-bit multiply against
  `__int128` for 40,000 random operand pairs including full-width ones; saturation and conversions; the
  64-bit coefficient step (a coefficient swinging across its whole range); the parameter mappings; the
  low-pass design against the formulas; and the 1-LSB design precision at 7, 20, 60 and 300 Hz for Q 0.5,
  2 and 40.

### Mutation testing

52 deliberate bugs, one at a time, in scratch copies (every type's prototype entry, the bilinear
terms, the Newton step, the error feedback and its sign, the feedback sign, the history shift, the
smoothers, the stale-coefficient and retarget flags, the period-end snap, the modulation sign, the Q and
dB mappings, `1/A`, the range checks, `reset`, the 64-bit step, the 128-bit helper):

* **49 were caught on the first run.** Of the three survivors, one was a real gap in the Q15 build (the
  error feedback is invisible at a 16-bit output; closed by `test_history_precision`), one was
  **dead code**: the carry of the 128-bit multiply's middle partial sum, which cannot occur for int64
  magnitudes (the high limbs are below 2^31), so I removed it instead of leaving a branch no test can
  reach; and one is **equivalent**: replacing the output's rounding by truncation changes nothing
  measurable, because the error feedback carries the truncation residual into the next sample (that is
  what it is for).

## Render patches (`render biquad`), measured from the WAV files

| Patch | What it is | Measured |
|-------|------------|----------|
| `biquad_eq_sweep.wav` | noise through a +12 dB peaking EQ, Q 4, swept 100 Hz to 8 kHz by `set_cutoff` | the spectral peak follows the sweep to 30 cents median (scatter from estimating a spectrum off noise); the boost over the flat floor is 10.1 dB averaged over a 1/6-octave band, against 11.1 dB computed from the cookbook for that band |
| `biquad_tilt.wav` | a 110 Hz saw, low shelf +12 dB at 400 Hz, switched half-way to a high shelf -12 dB at 2 kHz by `set_type` / `set_cutoff` / `set_gain` | harmonics 1 to 60 follow the cookbook shelf curves within 0.54 dB, in both halves |
| `biquad_notch.wav` | noise through a Q 6 notch swept +-2 octaves around 1 kHz by a 0.5 Hz LFO through `cutoff_mod` | at the LFO's turning points the spectral minimum is at 3984 Hz (expected 4000 Hz, -7 cents) and, at 250 Hz, within one FFT bin (23 Hz); 14 dB deep in 0.1 s windows |
| `biquad_butter4.wav` | noise through two `LP` instances at 1 kHz, Q = 0.5412 and 1.3066: a 4th-order Butterworth | against the ideal curve, over 20 s: -1.47 / -3.29 / -7.39 / -14.57 / -24.22 / -49.06 dB at 905 / 1000 / 1200 / 1500 / 2000 / 4000 Hz, ideal -1.61 / -3.01 / -7.24 / -14.25 / -24.10 / -48.16 (within 0.3 dB, the estimate's own noise, except 0.9 dB at 4 kHz where the band holds almost no power) |

Peaks are 0.14-0.33 and identical in float, Q15 and Q31. My first levels saturated the fixed-point
builds (full-scale white noise plus a 12 dB boost clips the *filter output*, before any output gain),
so the noise is attenuated to 0.25 on the way in.

## Known limits

* **Float below about 100 Hz** (200 Hz for shelves) at 48 kHz: coefficient rounding, see the table. Use
  fixed point or the SVF.
* **Violent modulation** of a high-Q biquad is not supported: the time-varying filter becomes large and
  noisy (the float history is held at +-8, fixed point saturates), but the output stays finite. Smooth
  envelopes and LFOs are fine; use the SVF for fast sweeps.
* **5 Hz** is accepted but is only good to a cent in cutoff and about 1 dB in response (fixed point).
* Gains beyond +-18 dB would need a wider feedforward format.
* Cortex-M0+ cost is unmeasured and probably high in the moving case.

## References

* Bristow-Johnson, R. *Cookbook formulae for audio EQ biquad filter coefficients* (the type set and the
  analogue prototypes).
* Orfanidis, S. J. *Digital Parametric Equalizer Design with Prescribed Nyquist-Frequency Gain* (JAES
  1997) and Zoelzer, U. *DAFX*, ch. 2: the `tan`-based coefficient forms.
* Oppenheim & Schafer, *Discrete-Time Signal Processing*: Direct Form I, error feedback (noise shaping)
  and the bilinear transform.
