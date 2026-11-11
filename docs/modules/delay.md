# Delay (modulatable, feedback, caller memory)

Files: `include/pcl/pcl_delay.h`, `src/fx/pcl_delay.c`, `tests/test_delay.c`, the shared internal header
`src/core/pcl_delayline.h` (circular buffer, storage formats, linear and Hermite reads; meant for the
reverb and the sampler too; its interpolation kernels moved to `src/core/pcl_interp.h` when the sampler
was added, verified bit-identical), the `delay` subcommand of `tools/render.c`, and the `delay_*` cases of
`platform/bench/pcl_bench.c`.

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds** (Clang with ASan +
UBSan, GCC `-O2`, the `cm0plus-*` / `cm33-*` compile checks including the no-soft-float check, and the
`mcxn947-*` firmware). Response, timing, sizes, host speed and **Cortex-M33 cycle counts are measured**:
on the FRDM-MCXN947 every delay case is bit-exact against the host build in all three types, float
included. M0+ is unmeasured, as for every module.

## Summary

The library's first memory-heavy module, and the pattern for the sampler and the reverb: a sizing
function, a caller-provided buffer, and a small-memory storage option.

```
y[n]   = line(n - d(n))                          fractional read: linear or 4-point Hermite
w[n]   = x[n] + feedback * tone(sat(y[n]))       written after the read
out[n] = x[n] + mix * (y[n] - x[n])
d(n)   = glided time + time_mod * mod_depth      Q20.12 samples, control rate, linear in between
```

* **One code path for echo, slapback, chorus, flanger and vibrato.** It's a delay with a time
  modulation input (the render patches show echo, chorus and flanger).
* **Caller memory.** `pcl_delay_required_bytes(&cfg)` then `pcl_delay_init(&dl, &cfg, buffer, bytes)`.
  Any byte buffer works (alignment slack is included). No heap, no tables.
* **Tape-style time changes:** the delay glides and the pitch bends while it moves. Gliding from 10
  to 20 ms over 1 s plays a 1 kHz input at **990.006 Hz** (theory: 990), then at exactly 1000 Hz.
* **Feedback** -0.99 .. 0.99 with an optional one-pole **tone** low-pass and **`tanh`** in the loop.
* **Mono.** Stereo means two instances. Ping-pong would need a coupled stereo module (not built).

## Decisions (from the review of the draft)

All nine were settled with the recommendation:

1. Both linear and 4-point Hermite (Catmull-Rom), selectable.
2. Tape-style smoothed time with a settable `glide_ms` (0 = jump, at most 10 s). The draft's
   suggested default of 50 ms is not hard-wired: `glide_ms` is a config field, and the render patches
   and tests choose their own.
3. Modulation at control rate with linear interpolation.
4. `PCL_DELAY_STORE_Q15` storage as the small-memory variant.
5. Tone low-pass and `tanh` in the loop; no DC blocker.
6. Mono; stereo by two instances.
7. `void *` buffer plus `bytes`, aligned internally.
8. The line internals in `src/core/pcl_delayline.h`.
9. `mix` inside the module.

### What changed from the draft while building

* **Hermite's minimum delay is 2 samples** (linear: 1). Hermite reads `x[n-i+1]`, one sample newer than
  the one it interpolates from. With the line read before the current sample is written (the feedback
  depends on the read), that sample doesn't exist yet at `i = 1`. The delay is clamped to the
  interpolator's minimum, and `set_interp` lifts a delay that is too short at once.
* **No per-sample read-head limit was needed.** The draft planned one so the read head could never
  overtake the write head. Clamping both ends of every control period to [minimum, maximum] does that
  already, because the per-sample path between them is a straight line.
* **`time_ms` uses a new `pcl_ms_t`** (a typedef of `pcl_hz_t`: float, or Q16.16) with `PCL_MS(x)`,
  declared in `pcl_delay.h`, so the time fields don't read as frequencies.
* **The setters validate against `max_delay_ms` stored in the struct.** The first version
  reconstructed it from the line length, and the rounding could reject exactly the maximum that
  `init` accepts.

## Parameters

| Parameter | Type | Range | Notes |
|-----------|------|-------|-------|
| `sample_rate_hz` | `uint32_t` | 8000 .. 768000 | |
| `max_delay_ms` | `uint32_t` | 1 .. 60 000, and at most 2^20 - 8 samples (21.8 s at 48 kHz, 1.36 s at 768 kHz) | Sizes the line; fixed after init |
| `time_ms` | `pcl_ms_t` | 0 .. `max_delay_ms` | Glides over `glide_ms`; 0 = the minimum (1 or 2 samples) |
| `mod_depth_ms` | `pcl_ms_t` | 0 .. `max_delay_ms` | Swing per unit of `time_mod`; the result is clamped to [minimum, maximum] |
| `glide_ms` | `uint32_t` | 0 .. 10 000 | 0 = jump (still ramped over one control period) |
| `feedback` | `pcl_coef_t` | -0.99 .. 0.99 | Smoothed per sample |
| `tone_hz` | `pcl_hz_t` | 0 (off), or 20 .. 0.45 fs | One-pole low-pass in the loop; switches at once |
| `saturate` | `int` | 0 / non-zero | `tanh` on the loop signal, passed through below 1/64 |
| `mix` | `pcl_coef_t` | 0 .. 1 | Linear dry/wet; smoothed per sample |
| `interp` | enum | `PCL_DELAY_LINEAR`, `PCL_DELAY_HERMITE` | Switches at once |
| `storage` | enum | `PCL_DELAY_STORE_NATIVE`, `PCL_DELAY_STORE_Q15` | Fixed at init |
| `time_mod` (input) | sample | [-1, 1] | Optional; sampled at control rate |

`required_bytes` returns 0 for an invalid config. `init` returns `PCL_ERR_NULL_POINTER`,
`PCL_ERR_OUT_OF_RANGE` (NaN included) or `PCL_ERR_BUFFER_TOO_SMALL`, leaving `dl` untouched. Setters
validate the same way. `in` and `out` may alias.

## Algorithm

* **Line:** `length = ceil(max_delay_ms * fs / 1000) + 4` samples (the 4 cover the Hermite taps), not
  rounded to a power of two. Indices wrap with one compare.
* **Delay:** `uint32_t` Q20.12 samples in every build. `ms -> Q20.12` is a 64-bit integer multiply and
  divide in fixed point, and float arithmetic in the float build (setters only).
* **Control tick** (every 16 samples): the glide advances one step (snapping to the target on its last
  step), the modulation is added, the result is clamped, and `d` moves linearly toward it over the
  period, snapping exactly at the end. All of this counts stream samples, so output doesn't depend on
  block size.
* **Reads** (`pcl_delayline.h`): linear `p1 + f (p2 - p1)`; Hermite with doubled Catmull-Rom
  coefficients (`C1 = p2 - p0`, `C2 = 2p0 - 5p1 + 4p2 - p3`, `C3 = (p3 - p0) + 3(p1 - p2)`,
  `y = p1 + t (C1 + t (C2 + t C3)) / 2`). That needs no halves in fixed point. Q15 linear reads use 32-bit
  arithmetic, everything else 64-bit, and rounding uses arithmetic shifts. Hermite can overshoot
  between samples; fixed point saturates.
* **Loop:** fixed point converts `y` to Q4.28 for `tanh` and the tone filter. Only the feedback signal
  goes through Q4.28, so `x` reaches the line at full precision, and with feedback 0 a delay is
  bit-exact. The written sum saturates. Float adds the filters' anti-denormal guard (1e-24) to the
  written sample and to the tone state.

## Accuracy (measured)

* **Integer delays are exact:** an impulse comes out at exactly `time * fs` samples (1, 3, 480 and 2400
  = the maximum) and nowhere else, in all builds, with both interpolators, both storage formats and an
  odd buffer address. `time = 0` gives 1 sample (linear) or 2 (Hermite).
* **Half-sample delay against the interpolators' analytic response**, at 1, 6, 12 and 18 kHz: within
  0.0001 dB and 0.00001 rad in every build. Useful figures for choosing (48 kHz, fraction 1/2, the
  worst case):

| | 1 kHz | 6 kHz | 12 kHz (fs/4) | 18 kHz |
|---|------:|------:|-------------:|-------:|
| linear | -0.02 dB | -0.69 dB | -3.01 dB | -8.34 dB |
| Hermite | -0.00 dB | -0.07 dB | -1.07 dB | -5.26 dB |

* **Echoes:** the n-th echo is `feedback^(n-1)` of the first (0.5, -0.5, 0.875), within 2 LSB per echo.
* **Against a double-precision model of the whole loop** (integer delay; tone, `saturate`, feedback up to
  0.99, mix): worst error float 1.4e-7, Q31 7.4e-9, Q15 5e-5. With `saturate`: about 1e-4, because
  `pcl_tanh` (2.3e-5 from libm) sits in a loop with gain up to 0.99.
* **`Q15` storage** against native storage, Hermite with modulation: within 1.8e-5 (16-bit rounding of the
  stored samples, at most 1.25 x half an LSB through the Hermite weights).

## Memory (measured)

* **State:** `sizeof(pcl_delay_t)` = **136 B** in all three builds (estimate 80-100 B).
* **Line:** `required_bytes` = `(ceil(max_ms * fs / 1000) + 4) * element + (element - 1)`, element = 4 B
  (float, Q31 native) or 2 B (Q15 build, or `Q15` storage). At 48 kHz that's 192 000 B per second
  native in float/Q31 and 96 000 B per second in 16 bits. With half the RAM and 16-bit storage: about
  85 ms on a 16 KB MKM14Z128A (slapback, chorus, flanger; not echo), 1.3 s on an nRF52840, 1.7 s on an
  LPC55S69, 2.7 s on an ESP32-C6.
* **Code** (`pcl_delay.c`, text, arm-none-eabi-gcc 15 `-O2`): **3680 B (M0+ Q15), 3928 B (M0+ Q31), 3004 B
  (M33 float), 3080 B (M33 Q31)**. The estimate was 2-4 KB. The M0+ objects use only integer runtime
  helpers, and the soft-float check passes.
* **Stack in `process`:** 400 B (M0+ Q15), 416 B (M0+ Q31), 208 B (M33 float), 216 B (M33 Q31).

## Cost (cycles per sample): MEASURED on a Cortex-M33

FRDM-MCXN947 (Cortex-M33 + FPU, 150 MHz), DWT cycle counter, `-O2`, from flash with the cache on,
48 kHz, 256-frame blocks of noise, 20 ms of a 50 ms line (native storage), feedback 0.6, mix 0.5;
modulated cases: +-5 ms of noise on the time; tone cases: 3 kHz tone and `saturate`. Bit-exact against
the host build in all three types (float: 2048/2048 samples identical).

| Case | Q15 | Q31 | float |
|------|----:|----:|------:|
| `delay_linear` | 147 | 160 | 83 |
| `delay_linear_tone` | 196 (180-210) | 213 (196-228) | 122 (101-141) |
| `delay_hermite_mod` | 223 | 226 | 129 |
| `delay_hermite_mod_tone` | 272 (256-286) | 278 (262-293) | 168 (147-188) |

(Ranges: fastest and slowest block, where they differ by more than 1 %.)

* **One delay is 5-9 % of the core in fixed point at 48 kHz and 2.7-5.4 % in float.** A plain linear echo
  costs about as much as a static SVF in fixed point (147-160 vs 155-165), and more in float (83 vs 65).
* **The float figures are about twice my estimate** (30-95 cycles, scaled from host timings against the
  SVF). The fixed-point figures fell inside theirs (110-275 for Q31). The host-to-M33 scaling held for
  the filters but not for the delay. Likely reasons, not separated: the loop's run-time branches
  (interpolator, storage, tone, `saturate`) are cheap on the host's predictor but not on the M33; each
  sample converts the Q20.12 fraction to float; and the two per-sample smoothers live in the loop.
* **The tone cases vary by about ±7 % between blocks.** Probably data-dependent: `tanh` is skipped below
  1/64, so its cost depends on how much of the loop signal is small in each block.
* The obvious optimisation, if wanted, is the SVF's: force-inlined loop variants without the run-time
  branches (at least static vs modulated, and with vs without the loop extras).

## Rate

Audio rate; time modulation and glide at control rate with linear interpolation; feedback and mix
smoothed per sample. Stated in the header.

## Tests (`tests/test_delay.c`: 384 checks float, 369 Q15 / Q31)

* Sizing and init: the exact `required_bytes` for both storage formats, 0 for NULL or invalid configs,
  every range edge (including `max_delay_ms` beyond the Q20.12 line: 22 s at 48 kHz is rejected, 21 s
  accepted), NaN in float, NULL buffer, buffer one byte short; `dl` unchanged on error. Setters likewise.
* Exact integer delays and the minimum, as above; half-sample response, as above.
* Feedback echoes (positive, negative, 0.875); the double model in 5 configurations.
* A constant `time_mod` equals the corresponding fixed time, bit for bit, with feedback and Hermite.
* The glide's Doppler shift (990.006 Hz), the pitch after it, and `d` landing exactly on 20 ms.
* A `mix` change ramps monotonically over the smoothing time (half-way at half the ramp).
* Blocks of 1, 7, 16, 17 and 256 bit-identical to one block, with modulation, glide, tone, `saturate`
  and four setters mid-stream; `reset` repeats the output; in-place; a zero `time_mod` equals `NULL`.
* `Q15` storage against native storage.
* Extremes: feedback 0.99, full-scale noise, +-30 ms of noise modulation, tone and Hermite, 10 s, with and
  without `saturate`: no NaN, no denormals, bounded (float peak 3.4 without `saturate`, 2.0 with; fixed
  point saturates at 1). Then 20 s of silence at feedback 0.5 decays below 1e-6, with no denormals.

All 27 suites pass in the float, Q15 and Q31 host builds; `test_delay` also runs clean under Clang ASan +
UBSan and GCC `-O2` in all three. Not done: mutation testing, MSVC.

## Render patches (`render delay`, measured)

| Patch | What it is | Measured |
|-------|------------|----------|
| `delay_echo.wav` | saw plucks (330 Hz, ADSR) into 375 ms, feedback 0.55, tone 2.5 kHz, mix 0.4 | first echo at 375.4 ms (the attack's 2 ms threshold crossing); second echo 0.52 x the first (0.55 less the tone's loss) |
| `delay_chorus.wav` | saw 220 Hz, 15 ms +-4 ms at 0.8 Hz, Hermite, mix 0.5 | peak 0.61 |
| `delay_flanger.wav` | white noise, 2.5 ms +-2 ms at 0.25 Hz, feedback 0.7, Hermite, mix 0.5 | peak 0.53 (render gain 0.8; at 1.5 the PCM conversion clipped) |

Echo timing and levels agree across float, Q15 and Q31.

## Known limits

* Cortex-M0+ cost not measured (no hardware). The float loop is unspecialised (see "Cost").
* Changing `interp` or `tone_hz` switches at once and may click.
* With `saturate` off, float lets the line grow to `1 / (1 - |feedback|)` times the input (100 x at 0.99);
  fixed point saturates it at full scale.
* `Q15` storage in a float build clips the delayed signal at full scale (16-bit storage can't hold more).

## References

* J. O. Smith, *Physical Audio Signal Processing*, online book (CCRMA), 2010: delay lines, fractional
  delay, interpolation.
* T. I. Laakso, V. Valimaki, M. Karjalainen, U. K. Laine, "Splitting the unit delay," IEEE Signal
  Processing Magazine 13(1), 1996.
* J. Dattorro, "Effect design, part 2: Delay-line modulation and chorus," J. Audio Eng. Soc. 45(10),
  1997.
* V. Zavalishin, *The Art of VA Filter Design*, 2018 (the TPT one-pole of the tone control).

All citations are from memory and have not been checked against the sources.
