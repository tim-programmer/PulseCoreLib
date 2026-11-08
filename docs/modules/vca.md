# VCA (gain with modulation input)

Header: `include/pcl/pcl_vca.h` - Source: `src/mod/pcl_vca.c` - Smoothing: `src/core/pcl_smooth.c`

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds. Cortex-M33 cycle counts
are measured (see "CPU cost"); Cortex-M0+ is not.**

## Algorithm

```
out[n] = in[n] * mod[n] * gain(n)
```

* `mod[n]` is an optional normalized [-1, 1] modulation signal. `NULL` means "unmodulated"
  (treated as 1.0, and the multiply is skipped). Negative values invert the signal, so the
  same module serves as an envelope-controlled amplifier, a tremolo, or a ring modulator.
* `gain(n)` is the smoothed linear gain. A change of target starts a straight-line ramp from
  the current value that lands exactly on the target after `PCL_SMOOTH_MS` (default 5 ms,
  rounded to whole samples). There is no exponential tail, so there are no denormals and no
  residual error. The ramp is recomputed from wherever it currently is if the target changes
  again mid-ramp.

This is a "VC" module in the CLAUDE.md sense: one code path, a gain plus a modulation input.
There is no separate VCA/gain implementation.

Reference: linear-ramp parameter smoothing is the standard click-free gain technique; no
specific paper applies.

## Parameters

| Parameter | Unit | Range | Notes |
|-----------|------|-------|-------|
| `sample_rate_hz` | Hz | 8000 .. 768000 | Only used to size the smoothing ramp |
| `gain` | linear | 0 <= gain < 2 | About +6 dB maximum. Float: `float`; Q15/Q31 builds: Q2.30 `int32_t`. Write constants with `PCL_COEF(x)` |
| `mod` (process input) | normalized | [-1, 1] | Optional; `pcl_sample_t` |

The upper limit of 2 is the Q2.30 range and is applied to the float build as well, so every
sample type accepts and rejects the same configurations. `init` and `set_gain` return
`PCL_ERR_OUT_OF_RANGE` outside it (NaN included) and leave the module unchanged.

## Rate

Audio rate. The smoother advances once per sample. (Control-rate smoothing with interpolation
is allowed by CLAUDE.md but would add nothing for a single multiply per sample.)

## Fixed-point behaviour

| Build | Sample | `in * mod` | `* gain` |
|-------|--------|------------|----------|
| Float | `float` | `float` multiply | `float` multiply |
| Q15 | `int16_t` | `pcl_mul_q15` (int32 accumulator) | `pcl_mul_q15_q30` (32-bit only, exact vs 64-bit) |
| Q31 | `int32_t` | `pcl_mul_q31` (int64 accumulator) | `pcl_mul_q31_q30` (int64 accumulator) |

* Each product rounds to nearest (ties toward +infinity) and **saturates**; full-scale input
  times a gain above 1 clips rather than wraps. The float build does not clip.
* The modulation multiply happens before the gain multiply, so a modulated block costs two
  roundings (at most about 1 LSB total error against an exact reference, below the 2 LSB
  tolerance the tests use).
* With `mod == NULL` and unity gain the output is bit-identical to the input in every build.
* The smoother's per-step division happens in `set_gain`, not in `process`. In fixed-point
  builds it is a 64-bit integer divide (`__aeabi_ldivmod` on Cortex-M0+); no float runtime
  is referenced.

## Memory

`sizeof(pcl_vca_t)` is one `pcl_smooth_t`: three `pcl_coef_t` (4 bytes each) and two `uint32_t`,
i.e. 20 bytes on all three builds. No lookup tables, no caller-provided buffers.

## CPU cost per sample: MEASURED on a Cortex-M33

Measured on a **FRDM-MCXN947 (Cortex-M33 + FPU, 150 MHz)** with the core's DWT cycle counter:
`-O2`, running from flash with the SDK's flash cache on, 48 kHz, 256-frame blocks, `pcl_vca_process`
alone (`scripts/mcxn947_bench.py`, cases in `platform/bench/pcl_bench.c`). Figures are the fastest of
8 blocks divided by 256 (spread under 1 %). Every case's output is **bit-exact against the host
build** on the board in Q15, Q31 and float, so these are timings of the tested code.

| Case | Q15 | Q31 | float |
|------|----:|----:|------:|
| `vca_steady` (gain settled, `mod == NULL`) | 22.4 | 24.4 | 7.4 |
| `vca_mod` (gain settled, modulation buffer) | 29.3 | 42.3 | 11.3 |
| `vca_ramp` (gain changed before every block, so a ramp is active ~94 % of the time) | 24.2 | 25.3 | 12.0 |
| `vca_mod_ramp` | 31.2 | 44.2 | 12.2 |

At 48 kHz one sample period is 3125 cycles, so a VCA costs 0.2 % (float, unmodulated) to 1.4 %
(Q31, modulated) of the core. **Flash cache off** (Q15): 30.1 steady (+34 %), 46.5 modulated (+59 %).
The gain change itself (`set_gain`, which holds the 64-bit divide in fixed-point builds) is a
control-thread call and was not timed.

What the disassembly shows:

* **Float: 7.4 cycles.** Once the ramp has landed the compiler emits a bare load / `vmul` / store.
  The ramping case is 12.0 because the smoother's add stays in the loop.
* **Q15: 22.4 cycles for what is one multiply on paper.** The gain multiply is the 32-bit-only split
  product `pcl_mul_q15_q30` (two multiplies, shifts, rounding and a saturating compare), which was
  designed so Cortex-M0+ never needs a 64-bit product. On an M33 a single `SMULL` would be faster;
  that is the price of the portable form. Modulation adds one 16x16 multiply plus saturation
  (+6.9 cycles).
* **Q31: 24.4 cycles; 42.3 modulated.** Each 64-bit product accumulates, rounds and saturates
  (`pcl_mul_q31`, `pcl_mul_q31_q30`); two of them for a modulated block.
* The operation-count table in the first version of this note was wrong in one place: the Q15 gain
  multiply is **two** 32-bit multiplies, not three. Operation counts do not predict cycles well here.

**Cortex-M0+ is still unmeasured** (no hardware), and the M33 figures do not transfer: M0+ has no
`SMULL` and no hardware saturation, so Q15 and Q31 will both cost more there.

## Aliasing and block size

`in`, `out` and `gain_mod` may alias in any combination. Any `frames >= 1` is accepted and
the result does not depend on how a stream is split into blocks (tested for 1, 7 and 256).

## Tests

`tests/test_vca.c` (all three sample types): init and setter validation including NaN; state
untouched on error; unity pass-through; silence at zero gain; modulation (zero, negative,
fractional); fixed-point output against a double-precision reference within 2 LSB; click-free
monotonic ramp that lands exactly; saturation instead of wrap; `reset` determinism; block-size
independence (bit-identical output and final state); in-place processing; long runs with
extreme parameters (finite, no denormals in float).

Supporting tests: `tests/test_smooth.c` (ramp), `tests/test_types.c` (fixed-point helpers
against 64-bit references).
