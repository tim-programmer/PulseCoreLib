# Noise (white, pink, brown) and sample-and-hold

Files: `include/pcl/pcl_noise.h`, `include/pcl/pcl_event.h` (new, see D3), `src/osc/pcl_noise.c`,
`src/core/pcl_rng.h` (internal), `tests/test_noise.c`, `tools/render.c`

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds** (Clang with ASan +
UBSan too, and the `cm0plus-*` / `cm33-*` compile checks including the no-soft-float check). The
decisions D1-D8 below were taken as recommended. Sizes, spectra, levels and the 10-minute runs are
**measured**. **Cortex-M33 cycle counts are measured** on a FRDM-MCXN947 (see "Cost"); **Cortex-M0+ counts are
still estimates**, because there is no M0+ hardware. Where this note says *prototype*, the figure came from the throwaway program written
before the implementation; the test suite has since reproduced it.

## Summary

Two small modules that share one PRNG:

* **`pcl_noise_t`**: white, pink or brown noise, one colour per instance, audio rate. No inputs.
* **`pcl_sah_t`**: random sample-and-hold. Picks a new uniform random value from an internal clock
  (Hz) and/or from triggers (`pcl_event_t` CLOCK events, or a rising edge in a buffer), with optional
  linear slew in ms.

All generators are **integer-only** and the three sample types are three views of the *same* stream:

```
Q31   = the generator's native output
Q15   = Q31 >> 16                (arithmetic shift, truncation)
float = (float)Q31 * 2^-31       (exact scaling; only the int->float conversion rounds)
```

so one seed gives the same noise in every build, and Q15 never needs saturation. See D1.

## Decisions taken

These are the places where I resolved a conflict in the request or stepped outside a CLAUDE.md
rule. All were implemented as recommended.

| # | Question | Recommendation |
|---|----------|----------------|
| D1 | CLAUDE.md rule 3 says "float reference first". For noise the *integer* generator is the reference in every build (no float state, so no drift, no denormals, no NaN, bit-identical across platforms). Float is only the final conversion. Tests compare against an independent double-precision model of the algorithm. | Accept. A float-state pink or brown filter would be less accurate and not reproducible across compilers/FPUs. |
| D2 | Your S&H spec says "runs at control rate" **and** "changes exactly on trigger sample offsets". Those conflict: a control-rate (16-sample) S&H would be up to 15 samples late. | Make the **pick** sample-accurate (O(1) per sample, one compare) and the **slew** a per-sample linear ramp. It is control-rate in *cost* (work happens only on a trigger) but not in *timing*. Say so if you actually want the 16-sample quantisation. |
| D3 | `pcl_event_t` does not exist yet; CLAUDE.md gives its shape. This is the first module to use it, so I would add `include/pcl/pcl_event.h` with exactly that struct plus three constants: `PCL_EVT_NOTE_ON`, `PCL_EVT_NOTE_OFF`, `PCL_EVT_CLOCK`. | Accept; the future arpeggiator/sequencer will own any other event types. Only CLOCK triggers S&H. Add NOTE_ON too if you want random-per-note. |
| D4 | One type with a "mode" enum, or two types? One type would give `process` a signature where three of four colours ignore the trigger arguments and S&H needs a different state. | Two types, one header: `pcl_noise_t` (white/pink/brown) and `pcl_sah_t`. |
| D5 | Seed 0 is a fixed point of xorshift32 (it would output zeros forever). | `seed == 0` is replaced by a documented default constant, not an error. |
| D6 | Slew time unit: CLAUDE.md says ms but not the type. | `uint32_t slew_ms` (integer ms, like `PCL_SMOOTH_MS`); no new type, no float. |
| D7 | Brown-noise DC blocker: I measured that the leak alone already prevents drift. | Keep the blocker anyway (3 ops/sample) as you asked; details in "Brown". |
| D9 | (added during implementation) A diagnostic accessor `pcl_noise_clamp_count()` exposes how often brown noise's state clamp was hit, so the 10-minute test can assert it never was. | It costs nothing on the hot path (the counter is bumped only inside the rare clamp branch). |
| D8 | Levels: no gain/level parameter (that is what the VCA is for). Each colour has a fixed nominal level (table below). | Accept. The render tool equalises loudness for listening. |

## Parameters

### `pcl_noise_t`

| Parameter | Unit | Range | Notes |
|-----------|------|-------|-------|
| `sample_rate_hz` | Hz | 8000 .. 768000 | `uint32_t`. Only brown uses it (sets the leak, see below) |
| `colour` | enum | `PCL_NOISE_WHITE`, `PINK`, `BROWN` | `set_colour` = `reset` + new colour; the sequence restarts from the seed |
| `seed` | - | any `uint32_t` | 0 is replaced by `PCL_NOISE_DEFAULT_SEED`. `set_seed` reseeds and resets |

`pcl_noise_process(n, out, frames)`: audio rate, any `frames` >= 1, `out` required. No error
checking in `process`; `init`/`set_colour` return `PCL_ERR_*` for NULL or out-of-range values and
leave the module unchanged.

### `pcl_sah_t`

| Parameter | Unit | Range | Notes |
|-----------|------|-------|-------|
| `sample_rate_hz` | Hz | 8000 .. 768000 | |
| `seed` | - | any `uint32_t` | Same rule as above |
| `rate_hz` | Hz (`pcl_hz_t`) | 0 .. 0.5 * sample_rate | Internal clock. **0 turns it off** (trigger-only). `PCL_HZ(x)`. Fixed-point builds also top out at 65535 Hz (Q16.16) |
| `slew_ms` | ms (`uint32_t`) | 0 .. 10000 | 0 = steps. Linear ramp, see below |
| `trig_in` (input) | normalised | [-1, 1] | Optional buffer; rising edge = trigger |
| `events`, `num_events` (input) | `pcl_event_t` | sorted by `offset` | Optional; `PCL_EVT_CLOCK` = trigger |
| `out` | normalised | [-1, 1) | Required |

`pcl_sah_process(s, out, trig_in, events, num_events, frames)`. `trig_in` and `events` may each be
`NULL` and may be used together (a sample with both counts as one trigger). The internal clock and
the triggers are independent; if you use both you get both.

## Algorithms

### White

xorshift32, shifts 13/17/5 (G. Marsaglia, "Xorshift RNGs", J. Stat. Software 8(14), 2003). Never
`rand()`. State is one `uint32_t` in the module struct, so there is no global state and a module can
be copied. Native output is `(int32_t)state` (full range; xorshift32 never outputs 0, a bias of
2^-32).

Tests pin the generator: seed 1 produces 270369 as its first value. Quality is audio-grade, not
statistical-grade; low bits of xorshift32 are weaker, so the Q15 view takes the *top* 16 bits.

### Pink: Voss-McCartney, 16 rows

Rows 0..15 each hold a uniform random value. Row 0 is replaced every sample; row `k` (1..15)
is replaced every `2^k` samples, namely whenever the sample counter has exactly `k-1` trailing zeros
(a counter that is a multiple of 2^15 updates nothing).
The output is the sum of the rows, kept as a running sum
(`sum += new - old`), so cost is constant. Rows are Q4.27 (`random >> 4`) so the sum of 16 rows is
exactly Q31 and **cannot overflow**. Per sample: two PRNG draws (white row first, then the octave
row, an order that is part of the determinism contract), one add/sub, a short trailing-zero loop
(average 2 iterations, at most 15) and no multiplies, no tables.

Why this method and not the alternatives (only Voss-McCartney was measured; the other two rows are
my reasoning from their structure):

| Method | Fit for Q15 / no-FPU | Notes |
|--------|----------------------|-------|
| **Voss-McCartney (chosen)** | Excellent: add, sub, shift | **Prototype: slope -3.13 dB/oct, worst 1/3-octave deviation 0.56 dB over 100 Hz..20 kHz at 48 kHz** (-3.07 / 0.34 dB over 100 Hz..5 kHz). Output is bounded, so it can never saturate |
| IIR pinking filters (Kellet 3/7-pole, Smith) | Poor | Poles sit within 0.1 % of 1 (e.g. 0.99886), finer than a Q15 coefficient can express, so they need Q31 state and 64-bit products on M0+; multiplies per pole; error depends on rounding. Not measured |
| Bank of shift-only leaky integrators | OK | N state updates per sample instead of one; no better accuracy than VM at this cost. Not measured |

**Low-frequency limit.** Row 15 changes every 32768 samples, so the spectrum flattens below about
`sample_rate / 65536` (0.7 Hz at 48 kHz, 12 Hz at 768 kHz, 0.12 Hz at 8 kHz). The ripple was only
measured at 48 kHz.

**Level (prototype):** rms 0.143, peak 0.74 over 10 minutes (hard bound 1.0). Mean over 10 minutes
up to 3e-3: that is the sub-hertz content of the lowest rows wandering, not a bias.

### Brown: leaky integrator of white noise, with a DC blocker

```
x[n]   = (top 16 bits of the PRNG draw)                      signed 16-bit white
u[n]   = (x[n] * g) >> 2                                     16x16 -> 32-bit multiply
acc[n] = clamp( acc[n-1] + u[n] - (acc[n-1] >> k) )          leaky integrator, pole 1 - 2^-k
y[n]   = acc[n] - acc[n-1] + y[n-1] - (y[n-1] >> j)          DC blocker, pole 1 - 2^-j, j = k + 2
out    = clamp(y[n])
```

* State is `int32_t` in **Q3.29** (full scale = 2^29), so every intermediate fits in 32 bits with
  two bits of headroom: **no 64-bit arithmetic, no float, and no wrap-around**, in all builds. The
  two clamps are defence; they were never hit in the prototype (see below).
* **Leak and sample rate.** `k = round(log2(sample_rate / (2*pi*15 Hz)))` clamped to 6..13 (a 7-entry
  threshold table, no log or float), which puts
  the integrator corner near 15 Hz at any sample rate (k = 9 at 48 kHz, 6 at 8 kHz, 13 at 768 kHz).
  The DC blocker corner is about 4 Hz (3.7 Hz at 48 kHz). The corner is fixed in v1.
* **Gain.** `g` comes from an 8-entry `uint16_t` table indexed by `k` (16 B of `.rodata`; the
  thresholds above are another 28 B) chosen to
  give an rms of about 0.10 FS for every `k`. At k = 9, `g` = 709.
* **Blocker state is clamped too**, so `(acc - prev_in) + (prev_out - (prev_out >> j))` can never
  exceed 2^30 + 2^29 and never overflows an `int32_t`, whatever the input.
* **Why the blocker, given the leak:** I measured (prototype, 10 minutes, k = 9) a mean of 3.2e-4 FS
  without it and about 5e-6 with it. The 3.2e-4 matches the statistical wander you expect from a
  15 Hz leaky integrator, not a rounding bias, so the leak alone does not cause drift and the blocker
  is a refinement, not a fix. It also trims sub-4 Hz wander that would otherwise eat headroom. Cost:
  3 operations. Say so if you would rather drop it.
* **Spectrum (prototype, 48 kHz, 87 s):** slope -5.96 dB/oct over 100 Hz..10 kHz, worst 1/3-octave
  deviation 0.14 dB; -5.90 and 0.76 dB if the band runs up to 20 kHz. The extra deviation near Nyquist
  is inherent to a one-pole integrator (a discrete integrator sits 0.6 dB above an ideal -6 dB/oct
  line at 10 kHz and 3.9 dB at Nyquist), so the test band stops at 10 kHz.
* **Level and range (prototype):** rms 0.090. Over 10 minutes at 48 kHz and four different seeds the
  peak was 0.41..0.49 FS (about 5 sigma), the clamps were hit **0 times**, mean was below 6e-6.
  The state's worst-case bound is about 8 FS, so "never saturates" is *statistical* for brown (10
  sigma of headroom at the chosen level), unlike white and pink, which are bounded by construction.

### Sample-and-hold

**Value.** A pick is `(int32_t)draw >> 1`, uniform in [-1, 1) as Q2.30 (the `pcl_coef_t` format),
from the module's own xorshift32. Output conversion: Q31 = value << 1, Q15 = value >> 15, float =
value * 2^-30.

**Initial state.** `init`/`reset` seed the PRNG and take the first pick immediately, so the output is
not zero for the first 1/rate seconds (100 s at 0.01 Hz).

**Internal clock.** A `uint32_t` phase accumulator (`inc = rate / sample_rate * 2^32`, rounded, computed in
the setter, same method as the oscillator). A carry out of the add is a trigger. The rate resolution
is `sample_rate / 2^32` (1.1e-5 Hz at 48 kHz, so 0.5 Hz can be off by up to 5e-6 relative); individual
intervals jitter by up to 1 sample because the period is not an integer number of samples. The first
clock pick comes one period after `init`.

**Triggers.**
* *Event:* a `PCL_EVT_CLOCK` with `offset` = n triggers at sample n. Other event types are ignored.
  Events with `offset >= frames` are ignored. Events must be sorted by `offset`; `process` trusts
  that and does not check. Several events at one offset make one pick.
* *Buffer:* a Schmitt trigger on `trig_in`: it fires when the signal rises through **0.5**, and
  re-arms when it falls below **0.25**. State persists across calls, so a gate that spans a block
  boundary fires once. After `init`/`reset` the gate is "low", so a buffer that starts high fires
  on its first sample.

**Timing contract (D2).** A trigger at sample n makes `out[n]` the new value (zero latency) when
`slew_ms == 0`. With slew, `out[n]` is the previous value and the output reaches the new value at
`n + slew_samples`.

**Slew.** Linear ramp of `round(slew_ms * sample_rate / 1000)` samples, using `pcl_smooth_t` (no
change to that module): `set_slew_ms` re-initialises the smoother at the value it was heading for
(so a ramp in progress ends immediately) with the new ramp length; a trigger calls
`pcl_smooth_set_target`. `process` emits the held value *before* advancing the ramp, which is what
makes `out[n]` still the old value at the trigger sample. It reads `value.current` directly for that. A trigger during a ramp restarts from the
current value toward the new pick. The ramp lands exactly (the smoother snaps on its last step).
`pcl_smooth.h` documents `set_target` as a control-thread call; here it runs inside `process` on a
smoother no other thread touches, which is safe. Slew 0 skips the smoother's division entirely.

**Rate.** Per sample: the clock compare, a trigger check for each input that is connected, and the
smoother step only while a ramp is active.

## Output levels

| Colour | Distribution | rms (FS = 1) | Peak | Bound |
|--------|--------------|-------------:|-----:|-------|
| White | uniform | 0.577 | 1.0 | hard |
| Pink | sum of 16 uniforms | 0.143 (prototype) | 0.74 in 10 min | hard, +/-1 |
| Brown | approx. Gaussian | 0.090 (prototype) | 0.49 in 10 min | statistical, clamped at +/-1 |
| S&H | uniform | 0.577 | 1.0 | hard |

Q15 is a truncation (`>> 16`), which adds a constant -0.5 LSB (-96 dBFS) offset to every colour;
below the quantisation noise, and not corrected.

## Memory (measured)

| Item | Size |
|------|-----:|
| `sizeof(pcl_noise_t)`, all builds | **88 B** (header 16 B, then a union: pink 72 B, brown 20 B) |
| `sizeof(pcl_sah_t)`, all builds | **48 B** (smoother 20 B included) |
| `.rodata` | **44 B**: brown gain table 16 B + leak threshold table 28 B. No sine or pink tables |
| Code, `-O2`, both modules (`pcl_noise.c` text) | M0+ Q15 2756 B, M0+ Q31 2736 B, M33 float 2866 B, M33 Q31 2698 B |

The code figures include the setters and the sample-and-hold; the first draft estimated 1.5-2.5 KB
at `-Os`, which was not measured at `-O2`. Pink rows are `int32_t` even in Q15 builds: that costs 32
bytes more than 16-bit rows would, and keeps `Q15 == Q31 >> 16` exact and Q15 arithmetic 32-bit only.

## Rate

White, pink, brown: audio. S&H: sample-accurate triggers; slew is a per-sample linear ramp that only
runs while active. See D2 for why it is not literally control rate.

## Cost (cycles per sample): MEASURED on a Cortex-M33

Measured on the **FRDM-MCXN947 (Cortex-M33 + FPU, 150 MHz)** with the core's DWT cycle counter, as
for the oscillator: `-O2`, running from flash with the SDK's flash cache on, 48 kHz, 256-frame
blocks, the module alone (`scripts/mcxn947_bench.py`, cases in `platform/bench/pcl_bench.c`). Figures
are the fastest of 8 blocks divided by 256 (the spread between blocks is under 1 % for the noise
colours and under 5 % for S&H). On the board, every case's output is **bit-exact against the host
build** in Q15, Q31 and float (the float noise and S&H outputs match to the last bit), so these are
timings of the code that was tested.

| Case | Q15 | Q31 | float | What it is |
|------|----:|----:|------:|------------|
| `noise_white` | 12.2 | 10.2 | 15.2 | xorshift32 + convert |
| `noise_pink` | 38.2 | 36.7 | 43.2 | Voss-McCartney, 16 rows |
| `noise_brown` | 43.2 | 43.2 | 45.2 | leaky integrator + DC blocker |
| `sah_hold` | 21.3 | 23.3 | 20.3 | **baseline**: clock off, nothing connected, never picks |
| `sah_clock` | 22.0 | 23.9 | 21.0 | 1 kHz internal clock, no slew |
| `sah_slew` | 32.1 | 34.9 | 30.1 | 1 kHz clock, 5 ms slew (a ramp is always active) |
| `sah_all_inputs` | 50.2 | 53.2 | 49.5 | clock + gate buffer + 2 CLOCK events per block + slew |

At 150 MHz and 48 kHz one sample period is 3125 cycles, so a white source costs about 0.3-0.5 % of
the core, pink 1.2-1.4 %, brown 1.4-1.5 %, and the busiest S&H 1.6-1.7 %.

**Flash cache off** (Q15, `-DPCL_MCXN947_DISABLE_LPCAC=ON`): white 12.3 (unchanged: the loop is tiny),
pink 48.5 (+27 %), brown 58.3 (+35 %), `sah_hold` 30.0, `sah_clock` 31.0, `sah_slew` 48.1,
`sah_all_inputs` 81.5 (+40 to +62 %). So instruction fetch from flash is a real part of these
figures; a part with slower flash or no cache will be worse. Fetch stalls in the cached figures were
not separated (that needs a run from SRAM).

**Estimate vs measurement.** The estimates in the first draft came from compiled prototype loops
with everything in registers; the shipped module was measured 1.1-1.4x above them for white, pink
and brown (Q31: 8 / 33 / 30 estimated, 10.2 / 36.7 / 43.2 measured), and **2.4-3.4x above for the
S&H** (7-8 estimated for the hold path, 21-23 measured; 22-24 estimated for all inputs, 50-53
measured). The disassembly shows why: even with no inputs connected the S&H loop executes about 25
instructions per sample, because it re-reads the smoother's `remaining` counter from memory, tests
the gate and event pointers, keeps its state in the struct rather than in registers, and spills the
frame count. The prototype had none of that. `sah_hold` (21.3) against `sah_clock` (22.0) shows the
cost is that fixed per-sample overhead, not the picks: a pick adds well under 1 cycle per sample on
average at 1 kHz (about 34 cycles each without slew). With slew, `sah_slew` minus `sah_hold` is
about 11 cycles per sample, which includes the ramp stepping every sample and one 64-bit divide per
pick in the fixed-point builds; the two are not separated.

**Possible optimisation, not done:** specialised loops for the common cases (no inputs; no ramp
active) would take the S&H hold path close to its prototype figure. I left it as is because 1.7 % of
the core is already small; say if you want it.

**Cortex-M0+ stays unmeasured.** The M0+ column of the first draft (white 13, pink 52, brown 40,
S&H 11 / about 35) came from the same prototype loops and is therefore **a lower bound that the M33
results show can be 1.1-3.4x low**; it has no measurement behind it. Expect more again on M0+, which
has no 32x32 -> 64 multiply (not used here) but also a slower multi-cycle memory pipeline.

## Tests (all implemented)

`tests/test_noise.c` in all three sample types. The CLAUDE.md standard set, plus the ones you listed:

**Standard**
* init/setter validation (NULL, every range edge, NaN/Inf where `pcl_hz_t` is float); state unchanged
  on error; `seed == 0` equals the default seed.
* `reset` is bit-identical; `set_colour` equals a fresh `init`.
* Block sizes 1, 7, 256 and one large block give bit-identical output for every colour, and the
  same for S&H with a fixed set of absolute-time triggers split into per-block events.
* Fixed-point vs "float reference": float equals `Q31 * 2^-31` within the int-to-float rounding
  (<= 6e-8), Q15 equals `Q31 >> 16` exactly, and all three equal an independent double-precision
  model of each algorithm (exact, since every value is dyadic).
* No NaN/Inf, no denormals in float, after long runs and every sample rate corner.

**Your list**
* *Same seed, same output across block sizes:* above; plus different seeds differ, and xorshift32
  seed 1 gives 270369 first.
* *White:* mean within 5 standard errors of 0; 1/3-octave PSD flat (slope within +/-0.1 dB/oct,
  every band within 0.5 dB) over 100 Hz..20 kHz.
* *Pink and brown slopes, FFT in the test:* Welch average (8192-point Hann, 50 % overlap, about
  2^21 samples) then a regression of 1/3-octave power density against log2 frequency. Targets:
  white 0, pink -3.01, brown -6.02 dB/oct. Limits will be the prototype's measured values plus
  margin, as for the oscillator: pink -3.01 +/- 0.3 over 100 Hz..20 kHz with bands within 1.0 dB;
  brown -6.02 +/- 0.3 over 100 Hz..10 kHz with bands within 0.5 dB. The test also runs brown at 8 kHz
  and 96 kHz (different `k`), which the prototype did not.
* *No saturation or overflow over 10 minutes (28.8 M samples at 48 kHz)* for pink and brown in Q15
  and Q31: no sample at either rail, peak below 0.9 FS, plus a debug counter of clamp hits staying 0
  (compiled in for the test only). Brown is statistical (see "Brown"), so this is checked for the
  default seed and a few others rather than proven.
* *S&H rate:* over 20 s at 0.5, 5, 100 and 1000 Hz the number of picks equals
  `floor(N * rate / sample_rate)` (+/-1), and every interval is within +/-1 sample of the ideal.
* *S&H trigger accuracy:* the value changes at exactly sample n for events at offsets 0, 1, 7, 255
  and for buffer edges at the same positions, including a gate that spans two blocks, a gate held high
  (no retrigger), and falling below 0.25 before re-arming; non-CLOCK events and offsets past `frames`
  ignored.
* *Slew:* reaches the new value at exactly `slew_samples`, monotonic, a retrigger mid-ramp restarts
  from the current value, `slew_ms == 0` is a pure step.
* *S&H distribution:* uniform in [-1, 1) (coarse histogram), mean near 0.

**Measured by the suite** (48 kHz unless noted; limits in the test are these plus margin):

| Check | Measured | Test limit |
|-------|---------:|-----------:|
| White slope / worst 1/3-octave band, 100 Hz..20 kHz | 0.000 dB/oct / 0.104 dB | +/-0.1 / 0.5 dB |
| Pink slope / worst band, 100 Hz..20 kHz | -3.134 dB/oct / 0.543 dB | -3.01 +/- 0.3 / 1.0 dB |
| Brown, 100 Hz..9.6 kHz | -5.973 dB/oct / 0.187 dB | -6.02 +/- 0.3 / 0.5 dB |
| Brown at 8 kHz (k = 6), 100 Hz..1.6 kHz | -5.884 dB/oct / 0.156 dB | same |
| Brown at 96 kHz (k = 10), 100 Hz..19.2 kHz | -6.006 dB/oct / 0.462 dB | same |
| rms: white / pink / brown | 0.577 / 0.145 / 0.090 | +/-0.01 |
| 10 minutes, pink (2 seeds) | peak 0.71 .. 0.74, no rails, bounded by construction | peak < 0.9 |
| 10 minutes, brown (3 seeds) | peak 0.47 .. 0.49, clamp hits 0, no rails | peak < 0.9, 0 clamps |
| S&H clock, 0.5 / 5 / 100 / 1000 Hz over 20 s | every pick at the model's instant; worst interval error 1 / 0 / 0 / 0 samples | <= 1 sample |

The suite also passes five **mutations** I applied to a scratch copy to confirm it can fail: an
event offset shifted by one sample, the wrong pink row (crashes), brown without its leak, a wrong
gate re-arm level, and a slew that lands late.

**Also:** GCC (default and `-O2`) and Clang with ASan + UBSan in all three builds, and the
`cm0plus-q15/q31` and `cm33-*` compile checks (the no-soft-float check passes: the only float is the
final `* 2^-31` in the float build). Not verified: MSVC (`/W4 /WX`).

**On hardware:** the benchmark harness (`platform/bench`) now also runs 7 noise and S&H cases
(`noise_white|pink|brown`, `sah_clock|slew|all_inputs|hold`) on the FRDM-MCXN947. In Q15, Q31 and float
every one is bit-exact against the host build and passes its CRC. Run it with
`.venv-tools/bin/python scripts/mcxn947_bench.py --type q15|q31|float`.

## Render tool

`tools/render.c`, built into `build-*/tools/render` (CLAUDE.md names `tools/render`; this is its first
subcommand):

```sh
mkdir out && build-float/tools/render noise out [seconds] [seed]
```

Writes `white.wav`, `pink.wav`, `brown.wav`, `sah_pitch.wav` and `sah_pitch_slewed.wav` (5 seconds each
by default, 48 kHz, mono, 16-bit PCM; the directory must exist).

* The three colours are each scaled by a fixed gain to the same rms (-20 dBFS, verified from the
  written files), printed with the resulting peak, so they can be compared by ear without touching the
  volume: white 0.173 gain, peak 0.17; pink 0.706, 0.41; brown 1.092, 0.38.
* The S&H files are a saw at 220 Hz whose pitch input is the sample-and-hold at 4 Hz with a depth of
  +/-1 octave, stepped and with 80 ms slew (fixed output gain 0.3, about -15 dBFS rms).
* The noise files are the same signal in every build, to within the output quantisation (checked by
  comparing the written files): float differs from Q31 by at most 1 LSB of the 16-bit file in
  9-13 of 240000 samples; Q15 differs by at most 1 LSB (2 for brown) because it truncates where
  the others round after scaling. They are not byte-identical.
* Also run as a 0.2 s smoke test under `ctest` (`tool_render_noise`).

## References

* G. Marsaglia, "Xorshift RNGs," J. Statistical Software 8(14), 2003 (xorshift32, shifts 13/17/5).
* R. F. Voss and J. Clarke, "1/f noise in music and speech," Nature 258, 1975; the row-update
  scheme is James McCartney's, described by Gardner, "White and brown music, fractal curves and
  1/f fluctuations," Scientific American 238(4), 1978. These citations are from memory and have not
  been checked against the papers.
* Brown noise as a leaky integral of white noise is the standard "Brownian/red noise" construction;
  the discrete leak, corner choice and Q3.29 headroom scheme are my design, validated only by the
  prototype above.
