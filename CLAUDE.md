# PulseCoreLib — CLAUDE.md

PulseCoreLib is a portable DSP library for music and sound synthesis: oscillators, LFOs,
filters, envelopes, VCAs, noise, arpeggiators, samplers, delays and reverbs. The same
source must build and run on desktop (Windows/macOS/Linux) and on microcontrollers,
including parts with **no FPU**.

Read this file before every task. The rules under "Hard rules" are non-negotiable;
if a task seems to require breaking one, stop and ask instead.

---

## Hard rules

1. **No heap allocation in `src/` or `include/`.** No `malloc`, `calloc`, `realloc`, `free`,
   `new`, or anything that calls them. The caller owns all memory: module state lives in
   caller-declared structs, and large buffers (delay lines, reverb tanks, sample data) are
   passed in by pointer with their size.
2. **Real-time safe processing.** `*_process()` functions must not allocate, lock, block,
   do I/O, call `printf`, or contain loops whose length is not bounded by the block size
   or a fixed constant.
3. **Every module supports every sample type** (`float`, Q15, Q31 — see below). Write the
   float reference first, then the fixed-point paths, in the same change.
4. **C99 only in the core.** No VLAs, no compiler-specific extensions outside
   `include/pcl/pcl_platform.h`, no C++ in `src/`. Must compile cleanly with
   `-std=c99 -Wall -Wextra -Wpedantic -Werror` on GCC and Clang, and `/W4 /WX` on MSVC.
5. **No `float`/`double` in fixed-point builds' processing paths.** Q15/Q31 builds must
   link and run on a soft-float Cortex-M0+ without pulling in the float runtime in
   `*_process()`. Init/config functions may use float only if guarded by
   `PCL_ALLOW_FLOAT_INIT`.
6. **No I/O in the core.** Audio drivers, file loading and MIDI parsing belong in
   `platform/` or `examples/`, never in `src/`.
7. **No new dependencies** (libraries, submodules, FetchContent) without asking first.
8. **Tests accompany code.** A module is not done until its tests pass in all three
   sample-type builds.

---

## Repository layout

```
include/pcl/          Public headers — one per module, plus pcl.h umbrella
  pcl_types.h         Sample types, status codes, fixed-point helpers
  pcl_platform.h      Compiler/arch detection, inline/restrict/alignment macros
  pcl_math.h          Fast math: sin tables, tanh approx, dB/Hz/pitch conversions
src/
  core/               Shared internals (smoothing, interpolation, tables)
  osc/                Oscillators, multi-voice oscillator, noise
  mod/                LFO, envelopes, VCA
  filter/             Biquad, SVF, ladder
  event/              Arpeggiator, sequencer, clock
  fx/                 Delay, reverb
  sampler/            Sample playback
  arch/               Optional optimized paths (cmsis/, esp/, sse/, neon/)
platform/             Audio I/O backends (desktop via miniaudio, embedded I2S examples)
tests/                Host-side unit tests (pcl_test.h harness, no external deps)
tools/                Host utilities: render patches to WAV, plot responses
examples/             Small example patches and target projects
docs/modules/         One design note per module (algorithm, params, ranges, CPU cost)
```

---

## Sample types

Selected at compile time with `PCL_SAMPLE_TYPE` = `PCL_FLOAT` | `PCL_Q15` | `PCL_Q31`.

| Type | `pcl_sample_t` | Range | Use |
|------|----------------|-------|-----|
| Float | `float` | nominally [-1, 1] | Desktop, Cortex-M4F/M33 with FPU |
| Q15 | `int16_t` | [-32768, 32767] | Cortex-M0+, RISC-V without FPU, low RAM |
| Q31 | `int32_t` | full int32 | No FPU but better quality than Q15 |

Fixed-point rules:
- Use the helpers in `pcl_types.h` (`pcl_mul_q15`, `pcl_sat_q15`, etc.); never hand-roll
  shifts in module code.
- Arithmetic that can overflow **must saturate**, never wrap.
- Accumulate Q15 products in `int32_t`, Q31 products in `int64_t`.
- Filter coefficients are stored as Q31 (or Q2.30 where |coef| can exceed 1) even in
  Q15 builds; document the format next to each coefficient field.
- Phase accumulators are `uint32_t` in every build (wraparound is the intended behaviour
  there and only there).

Float rules:
- Use `float` literals (`0.5f`) and `sinf`/`tanhf`-style functions; never `double` in
  process paths.
- Guard against denormals in feedback paths (filters, reverb, delay feedback).
- Never allow NaN/Inf to escape a module; clamp feedback and resonance.

---

## Module API shape

Every module follows the same pattern. Using the state-variable filter as the template:

```c
typedef struct
{
    float sample_rate_hz;
    float cutoff_hz;
    float resonance;            /* 0.0 .. 1.0 */
} pcl_svf_config_t;

typedef struct
{
    /* private — callers must not touch fields directly */
} pcl_svf_t;

pcl_status_t pcl_svf_init(pcl_svf_t *svf, const pcl_svf_config_t *cfg);
void         pcl_svf_reset(pcl_svf_t *svf);
void         pcl_svf_set_cutoff(pcl_svf_t *svf, float cutoff_hz);
void         pcl_svf_process(pcl_svf_t *svf,
                             const pcl_sample_t *in,
                             pcl_sample_t *out_lp,
                             pcl_sample_t *out_bp,
                             pcl_sample_t *out_hp,
                             const pcl_sample_t *cutoff_mod,   /* may be NULL */
                             size_t frames);
```

Conventions:
- **Block processing.** `process` handles `frames` samples per call. Support any `frames`
  from 1 to `PCL_MAX_BLOCK` (default 256); never assume a power of two.
- **In-place safe.** `in` and `out` may alias unless the header says otherwise.
- **Optional outputs and modulation inputs may be `NULL`.** A `NULL` modulation input means
  "unmodulated"; a `NULL` output means "don't compute it" where that saves work.
- **"VC" modules are just modulatable modules.** A VCO is an oscillator with pitch/PWM
  modulation inputs; a VCF is a filter with a cutoff modulation input; a VCA is a gain with
  a modulation input. Don't create separate VCO/VCF/VCA code paths that duplicate logic.
- **Setters are smoothed.** Parameter changes ramp over a short time (default ~5 ms) to
  avoid zipper noise, using `src/core/pcl_smooth`.
- **`init` validates, `process` trusts.** Range-check config in `init`/setters and return
  `PCL_ERR_*`; `process` does no error checking.
- **State is fully contained in the struct** so a module can be copied, reset, or
  serialized without hidden globals. No static mutable state anywhere.

### Units

| Quantity | Unit |
|----------|------|
| Frequency | Hz |
| Times (envelopes, smoothing, delay) | milliseconds |
| Gain | linear; dB only in conversion helpers |
| Pitch modulation | octaves (1.0 = +1 octave, 1V/oct style) |
| Generic modulation signals | normalized [-1, 1] |
| Pan | -1 (left) .. +1 (right) |

### Control rate

LFOs, envelopes and parameter smoothing may run at control rate: once every
`PCL_CONTROL_DIV` samples (default 16), with linear interpolation to audio rate when the
destination needs it. Each module's header must say whether it runs at audio or control rate.

### Event-driven modules

Arpeggiators, sequencers and clocks operate on events, not audio:

```c
typedef struct
{
    uint16_t offset;      /* sample offset within the current block */
    uint8_t  type;        /* PCL_EVT_NOTE_ON, NOTE_OFF, CLOCK, ... */
    uint8_t  note;
    uint8_t  velocity;
} pcl_event_t;
```

They consume and produce sorted `pcl_event_t` arrays per block, with caller-provided
output buffers and capacities. Timing must be sample-accurate via `offset`.

### Memory-heavy modules

Delay, reverb and sampler expose a sizing function and take caller memory:

```c
size_t       pcl_reverb_required_bytes(const pcl_reverb_config_t *cfg);
pcl_status_t pcl_reverb_init(pcl_reverb_t *rv, const pcl_reverb_config_t *cfg,
                             void *buffer, size_t buffer_bytes);
```

Offer a small-memory variant or quality setting for embedded use (e.g. a Schroeder-style
reverb alongside a higher-quality one).

---

## Target platforms

| Tier | Example parts | Notes |
|------|---------------|-------|
| Desktop | x86-64, Apple Silicon | Float default; SSE/NEON paths optional |
| FPU MCU | nRF52840 (Cortex-M4F), LPC55S69 (Cortex-M33) | Float or Q31; CMSIS-DSP optional |
| No-FPU MCU | MKM14Z128A (Cortex-M0+), ESP32-C6 (RISC-V) | Q15/Q31 only; no SIMD; keep tables small |

Optimized code in `src/arch/` is optional and selected by `PCL_ARCH_*` macros. The generic C
version is always the reference: every optimized path must match it within the tolerance
defined in its tests.

Keep lookup tables `const` (so they live in flash) and note their size in the module's
design doc. Prefer tables of 256–1024 entries with interpolation over larger tables.

---

## Build and test

```sh
# Host build + tests for each sample type
cmake -B build-float -DPCL_SAMPLE_TYPE=FLOAT && cmake --build build-float && ctest --test-dir build-float
cmake -B build-q15   -DPCL_SAMPLE_TYPE=Q15   && cmake --build build-q15   && ctest --test-dir build-q15
cmake -B build-q31   -DPCL_SAMPLE_TYPE=Q31   && cmake --build build-q31   && ctest --test-dir build-q31

# Embedded compile check (no link to hardware; catches float use and warnings)
cmake --preset cm0plus-q15 && cmake --build --preset cm0plus-q15
```

Run all three host builds before reporting a task as done.

### What every module's tests must cover

- Init with valid and invalid configs (correct status codes).
- `reset` gives bit-identical output for identical input (determinism).
- Odd block sizes (1, 7, 256) give the same output as one large block.
- No NaN/Inf, and no denormals in float, after long runs with extreme parameters.
- Fixed-point output matches the float reference within a documented tolerance.
- Module-specific checks, e.g.:
  - Oscillators: frequency accuracy, and aliasing below a threshold for band-limited waveforms.
  - Filters: magnitude response at a few frequencies vs. the analytic response; stability at max resonance with fast cutoff modulation.
  - Envelopes: stage timings within one control period.
  - Arpeggiator: event order and sample offsets for each mode.

Use `tools/render` to write WAV files for listening checks; never commit generated audio
except small golden files under `tests/golden/`.

---

## Coding style

- Formatting is enforced by `.clang-format` (WebKit base, Allman braces). Run
  `clang-format -i` on changed files.
- Names: `snake_case`; public symbols prefixed `pcl_`; types end in `_t`; macros and
  enum values are `PCL_UPPER_CASE`; file names match the module (`pcl_svf.h`/`pcl_svf.c`).
- Use fixed-width integer types (`int16_t`, `uint32_t`) and `size_t` for counts.
- `const` on every pointer the function doesn't write through; `PCL_RESTRICT` on
  non-aliasing buffers in hot loops.
- Every public function gets a header comment stating: what it does, parameter units and
  ranges, audio vs control rate, and whether buffers may alias.
- No magic numbers in DSP code; name constants and cite the source of an algorithm
  (paper, book, or well-known reference) in a comment and in `docs/modules/`.

---

## Workflow for adding a module

1. Write `docs/modules/<module>.md`: algorithm, parameters with units and ranges, rate,
   memory use, and expected CPU cost per sample.
2. Add the public header in `include/pcl/` following the API shape above.
3. Implement the float reference in `src/<group>/`.
4. Add tests and get the float build passing.
5. Implement Q15 and Q31 paths; get all three builds passing.
6. Run the embedded compile check.
7. Add the module to `include/pcl/pcl.h` and the module table in `README.md`.

## Build order (current plan)

1. Foundations: `pcl_types`, `pcl_platform`, `pcl_math`, parameter smoothing, test harness.
2. Sources: band-limited oscillator (PolyBLEP), multi-voice/unison oscillator, noise
   (white, pink, sample-and-hold).
3. Modulators: LFO, ADSR envelope, VCA.
4. Filters: biquad, state-variable (primary VCF), ladder.
5. Events: clock, arpeggiator, step sequencer.
6. Memory-heavy: delay, sampler, reverb (small and quality variants).
7. Patching layer: connect module outputs to modulation inputs for test patches.

## When unsure

Ask rather than guess on: public API changes to existing modules, new dependencies,
anything that would break a hard rule, or algorithm choices with large CPU/memory trade-offs.
