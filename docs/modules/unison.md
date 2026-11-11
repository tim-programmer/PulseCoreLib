# Unison oscillator (multi-voice, detuned, stereo spread)

Header: `include/pcl/pcl_unison.h` - Source: `src/osc/pcl_unison.c` - Builds on: `pcl_osc` (see
[oscillator.md](oscillator.md)) - Tests: `tests/test_unison.c`

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds. Sizes and host
timings below are measured. Cortex-M33 and M0+ cycle counts are not: there is no bench case or board
run yet.**

## Summary

One "fat" oscillator: up to `PCL_UNISON_MAX_VOICES` (default 8) copies of the band-limited oscillator,
detuned symmetrically around the base pitch, started at different phases, and mixed to mono or
spread across a stereo field.

```
offset_i = detune_oct * (i / (N - 1) - 1/2)        i = 0 .. N-1   (N = 1: offset 0)
f_i      = f_base * 2^offset_i
out_L/R  = 1/sqrt(N) * sum_i  g_L/R(pan_i) * osc_i(f_i, pitch_mod, pwm_mod, sync_in)
pan_i    = stereo_width * (2 i / (N - 1) - 1)       outer voices at +/- stereo_width
g_L(pan) = cos((pan + 1) * pi/4),  g_R(pan) = g_L(-pan)
```

* **Built from `pcl_osc_t`, not a new oscillator loop.** Each voice is a full `pcl_osc_t`, so unison
  inherits PolyBLEP/BLAMP, PWM, hard sync, the control-rate pitch path and the one-sample latency
  exactly. The tests check the output bit-for-bit against standalone `pcl_osc_t` voices. This
  follows the CLAUDE.md rule against duplicated code paths. The cost is speed (see "Cost").
* **Shared modulation.** `pitch_mod`, `pwm_mod` and `sync_in` go to every voice. Pitch modulation is
  in octaves, so it multiplies every voice's frequency by the same ratio and the detune ratios hold
  under vibrato and keyboard CV.
* **Detune is set in octaves** (CLAUDE.md pitch unit): `detune_oct` is the distance between the
  lowest and highest voice. With an even voice count no voice sits exactly on the base pitch.
* **Start phases** come from a `seed`: voice i starts at the (i+1)-th xorshift32 draw (the PRNG of
  `pcl_noise`). That stops the voices starting in phase, which would give a loud attack transient and
  comb-like early beating. `reset` restores the same phases, so output stays deterministic.

## Parameters

| Parameter | Unit | Range | Notes |
|-----------|------|-------|-------|
| `osc` | `pcl_osc_config_t` | as for `pcl_osc` | Sample rate, waveform, base frequency, pitch depth, pulse width, PWM depth: shared by all voices, validated as `pcl_osc_init` does |
| `voices` | count (`size_t`) | 1 .. `PCL_UNISON_MAX_VOICES` | Setter takes effect at once: offsets glide, gains jump, a re-enabled voice resumes at its stored phase (may click) |
| `detune_oct` | octaves (`pcl_oct_t`) | 0 .. 1 | Total spread, lowest to highest voice. Changes glide (~5 ms, through each voice's frequency smoother) |
| `stereo_width` | `pcl_coef_t` | 0 .. 1 | 0 = all voices centred, 1 = outer voices hard left/right. Changes ramp the pan gains (~5 ms) |
| `seed` | `uint32_t` | any | Start phases. 0 means `PCL_NOISE_DEFAULT_SEED`, as in `pcl_noise` |
| `pitch_mod`, `pwm_mod`, `sync_in` (inputs) | as `pcl_osc` | | Optional, shared by all voices |
| `out_l`, `out_r` (outputs) | sample | | `out_r == NULL` gives a mono mix in `out_l` (no pan gains). Outputs may alias inputs; `out_l` and `out_r` must not alias each other |

The waveform, pitch-depth, pulse-width and PWM-depth setters validate on voice 0 (which the
oscillator leaves unchanged on error) and then apply the value to every voice, active or not. A voice
pushed above `0.45 * sample_rate` by its detune offset is held there. This is a behaviour, not an
error, like the oscillator's own modulation clamp.

## Algorithms

### Voice frequencies

Each voice's offset `2^offset_i` uses the shared `2^x` table (`pcl_exp2m1_q16` / `pcl_octave_scale_f`
from `src/core/pcl_octave.h`), the same as the oscillator's pitch path. It is applied to the base
`pcl_hz_t` in the setter, followed by one `pcl_osc_set_frequency` per voice. Fixed point uses a 64-bit
product here, because `pcl_octave_scale_inc` would overflow for bases above 32768 Hz. So there is no
float in fixed-point builds, and nothing extra in `process`. Changing the base frequency, the detune
or the voice count calls `PCL_UNISON_MAX_VOICES` voice setters, each with one 64-bit divide, on the
control thread. The fixed-point offsets truncate toward zero, so voices `i` and `N-1-i` are exact
mirror images.

Detune spacing is **linear in octaves** (equal pitch steps). Szabo measured a non-linear spread on the
JP-8000's supersaw. It was left out (see "Decisions").

### Mixing and pan

* Voices render in **chunks of `PCL_UNISON_CHUNK` frames (default 32) into a stack buffer**. Each
  chunk is then scaled by the voice's gains and accumulated. The buffer is a fixed-size array (no VLA,
  no heap), and chunking keeps the stack bounded on an M0+. Chunks are consumed before outputs are
  written, so outputs may alias inputs.
* **Pan law: constant power.** `g_L = cos((pan + 1) * pi/4)` comes from `pcl_cos` in the setter. The
  right gain is computed as `g_R(pan) = g_L(-pan)` (equal to `sin`), not with `pcl_sin`. The first
  build used `pcl_sin`, and in Q31 the interpolated table put `sin(pi/4)` and `cos(pi/4)` one LSB
  apart, so width 0 did not give L == R. With one function the field is exactly mirror-symmetric.
  Gains are stored premultiplied by the mix gain, in Q2.30 in fixed-point builds.
* **Mix gain `1 / sqrt(N)`** (a 16-entry constant table), so detuned voices keep roughly the level of
  one voice (uncorrelated voices add in power). Phase-aligned peaks can exceed full scale: fixed point
  **saturates** and float does not clip, as in the other modules. The test forces 8 voices into phase
  with a shared sync to check this.
* Each voice sample is scaled with `pcl_mul_q15_q30` / `pcl_mul_q31_q30` (rounded), accumulated in
  `int32_t` (Q15) or `int64_t` (Q31), and saturated once at the end, so the result does not depend on
  voice order.
* **Width changes ramp the gains** linearly, per sample, over `pcl_smooth_ramp_samples` (one shared
  counter, so not N x 2 smoothers). The last sample of the ramp lands exactly on the target. The ramp
  counts stream samples, so it also advances during mono calls and does not depend on block size.
  Voice-count changes snap the gains (no ramp).

## Memory (measured)

| Item | Q15 | float | Q31 |
|------|----:|------:|----:|
| `sizeof(pcl_unison_t)`, 8 voices (x86-64 GCC) | 1056 B | 1120 B | 1120 B |
| `pcl_unison_process` frame, Cortex-M0+ `-O2` (`-fstack-usage`) | 456 B | | 832 B |
| `pcl_unison_process` frame, Cortex-M33 `-O2` | | 488 B | |
| Peak stack in `process`, including `pcl_osc_process` (288 / 392 B on M0+) | ~0.75 KB | | ~1.2 KB |
| New tables | none | none | none |

The stack is mostly the chunk buffer and the two accumulator arrays. For Q31 on an M0+, building with
`-DPCL_UNISON_CHUNK=16` roughly halves it. `PCL_UNISON_MAX_VOICES` (1..16) is a compile-time maximum
because the voices live inside the struct. An M0+ build can set it to 2-4 (about 130 B per voice).

**Code size** (`pcl_unison.c`, text, arm-none-eabi-gcc `-O2`, from the preset builds): 5016 B (M0+ Q15),
5112 B (M0+ Q31), 4898 B (M33 float), 5020 B (M33 Q31). That's more than expected. It hasn't been
analyzed or optimized for size; the likely causes are the forwarding setters and the 64-bit setter
arithmetic. The M0+ Q15 object depends only on integer runtime helpers (`__aeabi_idiv`, `ldivmod`,
`lmul`, `uldivmod`), and the soft-float check passes.

## Rate

Audio rate (the voices). Pitch/PWM are control rate inside each voice, as in `pcl_osc`. Pan-gain ramps
advance per sample. One sample of latency, inherited from the voices.

## Cost

**Host (x86-64, GCC `-O2`, 8 saw voices, 256-frame blocks), ns per output sample:**

| Build | 8 bare `pcl_osc_t` | unison mono | unison stereo |
|-------|-----:|-----:|------:|
| float | 21.1 | 24.2 (+15 %) | 24.5 (+16 %) |
| Q15 | 16.0 | 25.4 (+59 %) | 32.0 (+100 %) |
| Q31 | 30.9 | 44.2 (+43 %) | 51.4 (+66 %) |

The mix costs about 1-2 ns per voice per channel on the host. The percentages look large only because
the host runs a voice in 2-4 ns. On the Cortex-M33 a voice measured 82-162 cycles (see
[oscillator.md](oscillator.md)), and the mix should be a few cycles per voice per channel. So the
expectation is still **about N x the oscillator cost: 750-1100 cycles per sample for 8 saw voices,
25-35 % of a 150 MHz M33 at 48 kHz**. This is **not measured on hardware**; there is no bench case for
unison yet. The Q15 mix could be made cheaper (`pcl_mul_q15_q30` does two multiplies), but it has been
left alone until a board measurement shows that it matters. On an M0+ only 2-4 voices are realistic.

## Fixed-point behaviour

Voices behave exactly as `pcl_osc`, which is tested against its float/double reference. The mix error
against the exact (unrounded) mix of the same voices is at most **1.5 N + 1 LSB**: per voice, 0.5 LSB of
rounding plus up to 1 LSB from the Q2.30 gain. Measured for 5 pulse voices: Q15 2.5 LSB (7.6e-5), Q31
3.5 LSB (1.6e-9), float 2.2e-7. Pan gains against the exact constant-power law (width 0 vs mono, and
hard left/right): within 1.1e-4 (Q15), 1.4e-7 (Q31), 2.2e-7 (float).

## Tests

`tests/test_unison.c`, in all three sample types:

* init/setter validation (NULL, voice count 0 and above max, detune and width out of range, NaN in
  float, invalid `osc` fields, range edges accepted), with the state unchanged on error;
* **one voice equals `pcl_osc_t`** bit-exact in all builds, for every waveform, with pitch mod, PWM
  and sync active (start phase reproduced from the documented xorshift draw);
* **mono mix equals 1/sqrt(N) x the sum of the voices run standalone**: bit-exact in fixed point,
  within 1e-6 in float, plus the error bound above;
* detune: every voice's frequency (from `sync_out` timing) within 5e-5 of `f * 2^offset` for 2, 3, 4, 7
  and 8 voices, symmetric around the base. A new detune glides there, and a voice pushed above 0.45 x
  sample rate is held at 21600 Hz;
* stereo: width 0 gives L == R exactly and L = mono x cos(pi/4); width 1 puts two voices hard left and
  right. A width ramp has no jump and ends bit-identical to a unison initialized at the new width;
* voice count: reducing to 1 gives exactly that voice; increasing again stays finite;
* `reset` determinism. Block sizes 1, 7, 33 and 256 are bit-identical to one block, in stereo and mono,
  with pitch mod, PWM, sync, a glide, a detune change and a width ramp mid-stream;
* 8 in-phase voices saturate (exact match to a saturated reference) rather than wrap. Extreme settings
  (8 voices at 21.6 kHz, detune 1, width 1, square modulation at depth 7.9, a sync every 3rd sample)
  stay finite and bounded, with no denormals in float.

All 25 suites pass in the float, Q15 and Q31 host builds. `test_unison` also runs clean under Clang
with ASan + UBSan and with GCC `-O2` in all three builds. The `cm0plus-q15/q31` and `cm33-float/q31`
preset builds compile without warnings. Not verified: MSVC.

## Decisions (from the review of the draft)

The draft's seven open questions were all settled with the recommendation:

1. Built on `pcl_osc_t`, not a dedicated N-phase loop. A faster path can come later if a board
   measurement asks for one.
2. Mix gain `1 / sqrt(N)` (steady loudness; may saturate on in-phase peaks), not `1 / N`.
3. Linear detune only: no supersaw curve and no centre/side mix control.
4. Voice-count changes jump (may click) rather than fading.
5. Name `pcl_unison`.
6. No detune modulation input.
7. No `sync_out`.

## References

* A. Szabo, "How to Emulate the Super Saw," 2010 (analysis of the Roland JP-8000 supersaw: detune
  curve, mix, phase behaviour). Cited from memory, not checked.
* Constant-power panning: standard practice (e.g. C. Roads, *The Computer Music Tutorial*, 1996).
  Cited from memory, not checked.
* G. Marsaglia, "Xorshift RNGs," J. Statistical Software 8(14), 2003 (start phases).
