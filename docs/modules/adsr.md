# ADSR envelope

Files: `include/pcl/pcl_adsr.h`, `src/mod/pcl_adsr.c`, `tests/test_adsr.c`, an `adsr` subcommand
in `tools/render.c`, and `adsr_*` cases in `platform/bench` (nothing else changes; the 2^x table and
`core/pcl_octave.h` are already shared)

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds** (Clang with ASan +
UBSan, GCC `-O2`, and the `cm0plus-*` / `cm33-*` compile checks including the no-soft-float check). The
curve accuracy, sizes and the **Cortex-M33 cycle counts are measured** (FRDM-MCXN947, bit-exact against
the host build in all three sample types). **Cortex-M0+ is unmeasured**: there is no hardware.

## Summary

A four-stage envelope generator: a **gate** goes in, a unipolar **level** in [0, 1] comes out, to be
fed into the `gain_mod` input of a VCA (amplitude), or the `pitch_mod`, `pwm_mod` or `rate_mod` inputs
of other modules (the output is unipolar; use the LFO or a VCA to re-centre it).

```
gate on:   IDLE/RELEASE --> ATTACK (level -> 1.0) --> DECAY (-> sustain) --> SUSTAIN (held)
gate off:  any stage    --> RELEASE (-> 0)  --> IDLE
```

* **Exponential or linear segments with exact timing.** Each stage lasts exactly its configured time
  (to within one control period), and its curve is a normalised exponential from the existing 2^x
  table, with a per-stage "curve" amount from 0 (linear) to 1 (96 dB of exponential drop).
* **Control rate** (CLAUDE.md allows it): the level is evaluated every `PCL_CONTROL_DIV` = 16 samples
  and linearly interpolated, like the LFO. No tables beyond the shared 2^x table.
* Gate from a **buffer**, from **`pcl_event_t` NOTE_ON/NOTE_OFF** events, or from a control-thread call.
* Retriggering from any level is click-free: the new attack starts from the current output.
* One code path; the module accepts any gate source. "VC" modulation of the stage times is out of scope.

## Decisions taken

All eight were implemented as recommended.

| # | Question | Recommendation |
|---|----------|----------------|
| E1 | **What does a stage time mean?** (a) the stage lasts that long wherever it starts; (b) the time is for a full-scale excursion, so a stage that travels half as far takes half as long. | **(a).** It makes "stage timings within one control period" (CLAUDE.md) a direct test, and it matches an analog RC envelope, where closing a gap takes the same time whatever its size. Consequence: a retrigger from 0.9 still takes the full attack time, and a decay to a sustain of 0.9 still takes the full decay time. |
| E2 | **Curve shape.** Polynomial blends of (1-x)^p are cheap but cannot make deep exponentials (table below); one-pole recurrences need `exp`/`log` at setup, which fixed-point builds have no float for. | **Normalised exponential from the shared 2^x table**, integers only, exact endpoints, monotone, error -72 dB (nearly linear) to -102 dB (steep). One curve amount per stage. |
| E3 | **Control rate or audio rate?** | Control rate, as for the LFO (15-25 cycles per sample estimated). Cost: **gate edges act at the next control point** (0-15 samples later, 0.16 ms on average at 48 kHz) and stage times are whole control periods. Audio rate would probably cost 2-3x more (an estimate) for sub-0.3 ms precision nobody hears. |
| E4 | **Gate sources.** | `gate` buffer (Schmitt trigger, rises through 0.5, re-arms below 0.25: same as the sample-and-hold), `PCL_EVT_NOTE_ON` / `NOTE_OFF` events (note numbers ignored: voice assignment is the caller's job), and `pcl_adsr_gate()`. Same-sample order: events, then buffer. |
| E5 | **Retrigger and release.** | Gate on while releasing: new attack from the current level. Gate on while already high: ignored. Gate off in ATTACK or DECAY: release from the current level. Gate off while idle: ignored. |
| E6 | **Very short triggers.** A 1-sample pulse from a sequencer would otherwise start a release one control period after its attack began, and the envelope would barely move. | Add `run_through` (default off): while on, a gate-off during ATTACK or DECAY is remembered and applied when SUSTAIN is reached, so every trigger plays the full attack and decay. With sustain 0 this is a drum envelope. |
| E7 | **Setters and smoothing.** CLAUDE.md says setters are smoothed. | Stage times and curves are **latched when their stage begins** (smoothing a duration has no meaning); the sustain level is smoothed (about 5 ms), so moving it in SUSTAIN or DECAY does not click. State this in the header. |
| E8 | **Scope.** | Four stages only: no hold stage, no loop, no velocity scaling (use the VCA), no tempo sync. Times are integer milliseconds. |

## Parameters

| Parameter | Unit | Range | Notes |
|-----------|------|-------|-------|
| `sample_rate_hz` | Hz | 8000 .. 768000 | `uint32_t` |
| `attack_ms`, `decay_ms`, `release_ms` | ms | 0 .. 60000 | `uint32_t`. 0 means one control period (0.33 ms at 48 kHz, ramped, so no click). Latched at stage start |
| `sustain` | level (`pcl_coef_t`) | 0 .. 1 | `PCL_COEF(x)`. Smoothed ~5 ms. 1 means no decay travel |
| `attack_curve`, `decay_curve`, `release_curve` | amount (`pcl_coef_t`) | 0 .. 1 | 0 = linear; 1 = the stage's curve drops through 96 dB (16 octaves) before being normalised to its end point; 0.5 (48 dB) is a typical analog character. `PCL_COEF(x)`. Latched at stage start |
| `run_through` | bool | 0 / 1 | See E6 |
| `gate` (input) | normalised | [-1, 1] | Optional buffer |
| `events`, `num_events` (input) | `pcl_event_t` | sorted by `offset` | Optional; NOTE_ON = gate on, NOTE_OFF = gate off, other types and offsets >= frames ignored |
| `out` | level | [0, 1] | Required. Exactly 0 in IDLE; exactly full scale at the end of ATTACK |

`init` and the setters range-check (NaN included), return `PCL_ERR_*` and leave the module unchanged.
`process` trusts its arguments. `gate` may alias `out`. Accessor `pcl_adsr_stage()` returns IDLE /
ATTACK / DECAY / SUSTAIN / RELEASE, so a voice allocator can free a voice when it reaches IDLE.

## Algorithm

### Control point

Each stage lasts `N = max(1, round(time_ms * sample_rate / 1000 / 16))` control periods; progress `x`
advances by `2^32 / N` per period, and the last period lands on exactly `x = 1`. At each control
point (every 16 samples, counted from the start of the stream, so output does not depend on how it
is split into blocks):

1. apply the gate edges seen since the last control point (E5, E6); this may change the stage;
2. compute the stage's level at the **end** of this period, `v1` (below);
3. output the 16 samples as a linear ramp from the previous end level `v0` to `v1`, with the step
   truncated toward zero so the ramp can never overshoot or go negative;
4. `v0 = v1`; if the stage's progress reached 1, the next control point enters the next stage.

Stage levels, with `start` = the level at the control point where the stage began, `sus` = the
smoothed sustain, `d(x)` the normalised decay curve (1 at x = 0, 0 at x = 1) and `a(x) = 1 - d(x)`:

| Stage | Level at progress x | Ends |
|-------|--------------------|------|
| ATTACK | `start + (1 - start) * a(x)` | at 1.0, exactly |
| DECAY | `sus + (start - sus) * d(x)` | at `sus`; then SUSTAIN |
| SUSTAIN | `sus` | on gate off |
| RELEASE | `start * d(x)` | at 0, exactly; then IDLE |

So the attack curve is the mirror image of the decay curve (fast start, slow finish: a capacitor
charging) and decay and release fall fast then tail off. With curve 0, `d(x) = 1 - x` (linear).

### The curve

`d(x) = (2^(-m x) - 2^(-m)) / (1 - 2^(-m))` with `m = 16 * curve` octaves, so `curve` is directly "96 dB
of drop times the curve amount". It is zero at `x = 1` and one at `x = 0` by construction, so the end
points are exact and the stage lasts exactly its time. `2^(-m x)` uses the shared octave routine
(`core/pcl_octave.h`, Q4.28); `m x` beyond 8 octaves is split as `(2^(-m x / 2))^2`. The constants
`2^(-m)` and `1 / (1 - 2^(-m))` are computed once, in the setter. In the float build the same maths
uses `pcl_octave_scale_f`; fixed point works in Q2.30 with 1.0 held as the largest value below it, as
in the LFO, with 64-bit products at control rate only, never per sample.

**Measured (script using the library's own 2^x table and octave routine in a Q31 build; ideal =
double-precision formula), over 20 001 points each:**

| `m` (octaves) | Drop (dB) | Max error (fraction of full scale) | Monotone | Endpoints |
|-----:|-----:|-----:|:-----:|:-----:|
| 0.0625 | 0.4 | 2.6e-4 (-71.6 dB) | yes | exact |
| 0.25 | 1.5 | 7.0e-5 (-83.1 dB) | yes | exact |
| 1 | 6 | 2.2e-5 (-93.1 dB) | yes | exact |
| 3.32 | 20 | 8.6e-6 (-101.3 dB) | yes | exact |
| 6.64 | 40 | 7.6e-6 (-102.4 dB) | yes | exact |
| 8 | 48 | 7.9e-6 (-102.1 dB) | yes | exact |
| 10 | 60 | 7.6e-6 (-102.4 dB) | yes | exact |
| 13.3 | 80 | 7.3e-6 (-102.7 dB) | yes | exact |
| 16 | 96 | 7.8e-6 (-102.1 dB) | yes | exact |

The error is the table's 8-bit interpolation divided by `1 - 2^(-m)`, so it grows as the curve
approaches linear, while a straight line's own error against that curve shrinks. I measured both near
the crossover (same script, 20 001 points):

| `m` (octaves) | `curve` | Table exponential error | Plain linear error |
|-----:|-----:|-----:|-----:|
| 0.0040 | 1/4096 | 4.1e-3 (-47.7 dB) | 3.5e-4 |
| 0.0080 | 1/2048 | 2.1e-3 (-53.7 dB) | 6.9e-4 |
| **0.0156** | **1/1024** | **1.0e-3 (-59.7 dB)** | **1.4e-3** |
| 0.0300 | 1/533 | 5.4e-4 (-65.3 dB) | 2.6e-3 |
| 0.0625 | 1/256 | 2.6e-4 (-71.6 dB) | 5.4e-3 |

So **`curve < 1/1024` (`m < 1/64`) is treated as exactly linear**: that is where the two errors cross, and
the worst case anywhere is then about 1e-3 of full scale (-60 dB) -- 0.1 %, inaudible in an envelope. The
shape therefore changes by about 0.14 % of full scale as `curve` crosses 1/1024 (a small step, not a
continuous change); I will say so in the header.

**Why not polynomial blends (rejected).** I fitted `(1-c)(1-x) + c(1-x)^p`, p = 2, 3, 4 to the same
normalised exponentials (best `c`, worst-case error as a fraction of full scale):

| Exponential (drop) | p = 2 | p = 3 | p = 4 |
|-----:|-----:|-----:|-----:|
| 4 dB | 0.2 % | 0.6 % | 1.2 % |
| 9 dB | 0.8 % | 0.8 % | 1.9 % |
| 17 dB | 3.0 % | 0.4 % | 2.2 % |
| 26 dB | 11.8 % | 2.2 % | 1.2 % |
| 40 dB | 25.3 % | 12.0 % | 2.9 % |
| 60 dB | 39.0 % | 26.3 % | 16.7 % |

Cheap, but 12-39 % wrong at the 40-60 dB drops that give an envelope its analog tail. Rejected for E2.

### Gate handling

Gate edges are latched as they are seen (every sample, from the buffer's Schmitt trigger and the
events) and **applied at the next control point**; the transition itself is therefore up to 15 samples
late, and a stage's first level step is a 16-sample ramp, never a jump. Both an on and an off inside
one control period are applied in order at the next point, so a very short trigger still starts an
attack; with `run_through` off its release then begins at once from the level just reached (nearly
nothing, as for any ADSR with a very short gate). The Schmitt state persists across calls, so a gate
that spans a block boundary fires once, and a gate already high on the first sample after
`init`/`reset` fires.

### Sustain, times and curves while running

The sustain level is smoothed, so changing it in SUSTAIN glides, and in DECAY the curve's target moves
smoothly. Times and curves are read when a stage starts: changing `release_ms` mid-release affects
only the next release. `reset` clears to IDLE at level 0 (configuration kept).

## Memory (measured)

| Item | Size |
|------|-----:|
| `sizeof(pcl_adsr_t)`, all builds | **136 B** (three stages' constants plus the running copy 80 B, the sustain smoother 20 B, level/ramp/stage state) |
| Extra `.rodata` | **0 B**: uses the 2^x table the oscillator and LFO already need |
| Code, `-O2` (`pcl_adsr.c` text) | M0+ Q15 3668 B, M0+ Q31 3652 B, M33 float 4096 B, M33 Q31 3754 B |

The struct is a little over the 110-130 B estimated, and the code about 1 KB over the 2-3 KB estimated.
No heap, no caller buffers, state fully contained.

## Rate

Control rate (every `PCL_CONTROL_DIV` samples) with per-sample linear interpolation; the header will
say so. Gate edges are quantised to the control point (E3).

## Cost (cycles per sample): MEASURED on a Cortex-M33

Measured on the **FRDM-MCXN947 (Cortex-M33 + FPU, 150 MHz)** with the DWT cycle counter: `-O2`, flash
cache on, 48 kHz, 256-frame blocks, the ADSR alone (`scripts/mcxn947_bench.py`, cases in
`platform/bench/pcl_bench.c`; A 5 ms, D 10 ms, S 0.5, R 10 ms, curve 0.5 unless noted). Average over the
2048-sample run, with the cheapest 256-sample block in brackets. Every case is **bit-exact against the host
build** in Q15 and Q31, and float matches to the last bit.

| Case | Q15 | Q31 | float |
|------|----:|----:|------:|
| `adsr_idle` (never gated) | 16.6 | 15.9 | 23.2 |
| `adsr_sustain` (gated on, in SUSTAIN) | 16.5 | 15.9 | 23.2 |
| `adsr_cycle` (NOTE_ON, then NOTE_OFF half-way: every stage) | 19.3 (16.2) | 18.7 (15.6) | 27.7 (23.5) |
| `adsr_cycle_linear` (curve 0) | 17.9 | 17.3 | 25.0 |
| `adsr_gate_buffer` (the same cycle from a gate buffer) | 29.8 (24.2) | 29.2 (23.6) | 38.2 (29.8) |

* **16 to 19 cycles per sample for a whole envelope, 0.5-0.6 % of the core at 48 kHz** (Q15/Q31 without a
  gate buffer): inside the 15-25 range estimated. A full cycle costs about 2.7 cycles more than idling:
  the exponential curve is cheap, and linear instead of curve 0.5 saves 1.4.
* **The gate buffer is the expensive part**: connecting one adds about 13 cycles per sample (Schmitt scan
  on every sample), more than the whole rest of the module. Events cost almost nothing (`adsr_cycle` is
  driven by events). If a gate is a few events per second, use `NOTE_ON`/`NOTE_OFF` events instead.
* **Idle costs as much as sustain** because the per-sample ramp loop and the event/gate checks run either way
  (about 10 cycles is the ramp loop, as for the LFO; the rest is those checks).
* **Float is dearer (23-28)**: its ramp is evaluated as `v0 + diff * t` rather than accumulated, see below.
* **Flash cache off** (Q15): idle 18.6, sustain 19.4, cycle 23.9 (linear 21.8), gate buffer 42.8, about
  12-45 % more. Instruction fetch from flash matters here too.
* **Cortex-M0+ is unmeasured** (no hardware); the M33 numbers do not transfer. The ramp loop is 32-bit only
  in Q15 and Q31; the control point has several 64-bit products, which M0+ does in software.

### A float bug found while testing this module

The first version of the float ramp accumulated (`cur += step`). On the flat top of a steep attack the
two end levels of a control period are a few ulps apart, and fifteen rounded additions drifted past them:
the output reached **1.00000024** (above full scale) and went non-monotone. Fixed point is immune (integer
adds are exact). The float ramp is now `v0 + diff * t` with `t` stepped by exact multiples of 1/16, which
is monotone and bounded by construction. The same fix was applied to the **LFO**, which had the same latent
bug (33 samples above 1.0 in 96 000 runs near sine peaks), with a regression test
(`test_peak_never_exceeds_range`); see `lfo.md`. The price is that the float ramp is dearer: the float LFO
went from 12.5 to 16.7 cycles per sample.

## Tests (implemented)

`tests/test_adsr.c`, 15 test groups, about 525 checks (527 float, 523 Q15 and Q31), in all three sample
types:

* init/setter validation (NULL, range edges incl. 60000 ms, NaN/Inf in float, sustain and curve range),
  state unchanged on error;
* **the whole module against an independent double-precision model** of the state machine (gate edges
  applied at the next control point, stages in whole periods, exact end points) over nine scenarios: linear,
  curve 0.5, mixed curves, small curves (0.002 / 0.01 / 0.1), sustain 0 and 1, retriggers during release,
  an off during attack, `run_through`, and `run_through` re-gated before sustain; tolerance is the build's
  own plus the exponential's table error for that curve;
* the **stage reported after every control period** equals the model's;
* **stage timing measured from the output alone** for times 0, 1, 7, 100 and 1000 ms at 8, 44.1, 48 and 96
  kHz: the attack's end, the decay's landing on the sustain level and the first exactly-zero release sample
  are each within one control period of the requested time;
* shape: attack never falls and ends at exactly the largest level, decay and release never rise, sustain is
  held, release ends at exactly 0 and the module reports IDLE, idle output is exactly 0, for curves 0, 0.3, 1;
* **no clicks**: at every transition of every scenario no sample-to-sample step exceeds one full-scale ramp
  (1/16);
* gate sources: events (NOTE_ON/OFF ordering, ignored types, out-of-range offsets), `pcl_adsr_gate()`, the
  buffer's Schmitt trigger (start high, a 40-sample dip to 0.3 inside the hysteresis band, the exact
  threshold, a 1-sample pulse, block boundaries from 1 to 20000 samples, the buffer aliasing the output),
  and `run_through` with 1-sample triggers (the whole attack plays; without it the peak stays under 0.1);
* **block-size independence** (1, 7, 16, 17, 256, 1000, bit-identical) for every scenario; `reset`
  determinism; sustain glides without a step; times are latched when a stage begins (a release started at
  100 ms stays 100 ms after `set_release_ms(1000)`, the next one takes 1000 ms);
* extreme parameters (0 ms, 60 s, sustain 0 and 1, curve 0 and 1, with re-gating): in range, finite, no jumps;
* a **10-minute run with random gates, timings, sustain and curves** (225 000 random blocks): in range,
  finite, no step larger than one ramp (0.0625).

**Mutation checks** on a scratch copy: a stage one period too long, an attack that stops short of full scale,
a gate-on that always restarts the attack, a pending release not cleared by a re-gate, the Schmitt trigger
re-arming at the high threshold, an event applied one sample late, an off applied although the gate came
back, and a release starting from the wrong level were all caught in float and Q15. **Two initially
survived** and exposed gaps in the suite, now closed: a re-gate before sustain under `run_through` (a new
scenario) and the hysteresis band (the 2-sample dip I first used falls and recovers inside one control
period, which the module ignores by design, so it hid the bug; it is now 40 samples). One mutation **still
survives**: removing the final output clamp (`v1 = adsr_clamp(v1)`). The clamp is defensive; I could not
construct an input that needs it in any build (fixed-point products cannot leave [0, 1], and the float ramp
is bounded by construction), so it stays without a test that forces it.

Not verified: MSVC (`/W4 /WX`).

## Render tool

`build-*/tools/render adsr <outdir> [seconds] [seed]` writes five 5-second patches (48 kHz, mono, 16-bit):

| File | Patch | Measured from the written file |
|------|-------|--------------------------------|
| `adsr_pluck.wav` | saw 220 Hz through a VCA driven by the ADSR (2 / 250 / 0.2 / 300 ms, curve 0.5), four notes | four identical note peaks (rms 0.206); silent before the next note |
| `adsr_pad.wav` | saw 110 Hz, 800 ms attack, 0.7 sustain, 1.2 s release, one long note | rms 0.011 at 0.2 s, 0.231 at 0.6 s, sustain 0.171, 0.0000 by 4.5 s |
| `adsr_perc.wav` | white noise, 0 ms attack, 120 ms decay, sustain 0, `run_through`, triggered by **1-sample pulses** in a gate buffer | each pulse plays a full hit (peaks 0.186-0.207); exact silence between |
| `adsr_curves.wav` | the same note linear, then (from 1.9 s) with curve 0.6 | rms +0.1 s into the decay: 0.198 (linear) vs 0.103 (exponential); both reach sustain |
| `adsr_retrigger.wav` | notes retriggered while still releasing | not separately measured; the no-click property is asserted by `test_no_clicks_at_transitions` |

The three sample types give the same patches to within the output quantisation plus the usual saw-edge timing
differences (`adsr_perc.wav`: at most 1 LSB of the 16-bit file; the saw patches up to about 360 LSB at an
edge). Also run as a 0.2 s smoke test under `ctest` (`tool_render_adsr`).

## References

* Exponential envelope segments as a capacitor charging or discharging: standard analogue synth
  practice; there is no paper to cite. The normalised-exponential-from-a-table construction and the
  curve-family comparison above are my own.
* The one-pole "target ratio" exponential envelope (e.g. as popularised by EarLevel Engineering) is the
  usual digital alternative; I did not use it because its setup needs `exp` and `log`. This reference
  is from memory and has not been checked.
