# Reverb (small and quality variants, caller memory)

Header: `include/pcl/pcl_reverb.h` - Source: `src/fx/pcl_reverb.c` - Tests: `tests/test_reverb.c`

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds, and on two boards
(FRDM-MCXN947 Cortex-M33, Raspberry Pi Pico Cortex-M0+), bit-exact against the host** (Clang with ASan +
UBSan as well, GCC Release, and the Cortex-M0+ / M33 / MCXN947 cross builds, including the M0+ check
that Q15/Q31 pull in no soft-float runtime). Cycle counts are **measured** on both boards and are
**well above the draft's estimates**; see "Cost" and "Open points".

## Summary

Mono in, stereo out, in caller memory, with two algorithms behind one API ("a Schroeder-style reverb
alongside a higher-quality one", CLAUDE.md):

| Variant | Algorithm | Lines (48 kHz, size 1) | Memory, 16-bit lines | Native float / Q31 |
|---------|-----------|-----------------------:|---------------------:|-------------------:|
| `SMALL` | Schroeder-Moorer: 4 parallel damped combs, summed with two sign patterns into two 2-all-pass chains | 6 073 samples | 12.1 KB | 24.3 KB |
| `QUALITY` | 8-line feedback delay network (Jot), Hadamard matrix, per-line damping, 4 input diffusers, 2 modulated lines | 22 065 samples | 44.1 KB | 88.3 KB |

```
SMALL:    x -> predelay -> K_S x -> 4 combs [line <- in + D_i(line out)] -> L = c0+c1-c2-c3, R = c0-c1+c2-c3
                                    -> /2 -> 2 all-passes per channel -> width, x2, mix
QUALITY:  x -> predelay -> K_Q x -> 4 diffusers -> u
          y_i = line i out,  h = H8 (D_i(y_i)),  line i <- h_i +- u (signs of Hadamard row 7)
          wet L = h_1, wet R = h_2 -> width, x2, mix
D_i(y):   lp <- p lp + (1-p) y;  D = g_hf y + (g - g_hf) lp       (the 1/sqrt(8) of H8 is folded in)
```

## Decisions (from the review of the draft)

All eight recommendations were accepted ("go"):

1. `SMALL` = Schroeder-Moorer, `QUALITY` = 8-line FDN (not Dattorro's plate, not 16 lines).
2. One module with a `variant` enum.
3. Parameters: RT60, HF ratio + crossover, size, predelay, width, mix, modulation depth; no early
   reflections.
4. Mono in, stereo out.
5. Storage native or 16-bit (the delay's `pcl_delay_storage_t`); no half-rate tank yet.
6. Line lengths made mutually prime at init (here: distinct primes, so every pair is coprime).
7. `freeze` included.
8. `QUALITY` modulation: 2 of the 8 lines, +-8 samples at 48 kHz, linear interpolation.

### What changed from the draft while building

* **16-bit lines round stochastically, with a small dead zone** (the draft said "round toward zero").
  Truncation everywhere is the textbook cure for limit cycles, but at 16 bits it made the decay
  level-dependent: a 3 s `QUALITY` tail from one impulse measured 2.3 s. Rounding to nearest left
  limit cycles of +-4 LSB (-78 dBFS) forever. Stochastic rounding (add a uniform fraction of an LSB,
  then floor) is unbiased, but on its own the FDN never falls silent: the matrix spreads a 1-LSB value
  over 8 lines at 0.35 LSB each and stochastic rounding regrows about 2.8 of them, a +-3 LSB floor.
  Truncating only values below 2 LSB fixes both: 3 s measures 2.92 s (T20), and every tail ends at
  exactly 0.
  The dither is one 32-bit LCG per line, held in the state (deterministic, block-size independent).
  The Q4.28 products inside the loops still round toward zero, so native Q31 lines also decay to 0.
* **Internal levels halved, with a x2 make-up outside the loops**, and `QUALITY`'s input gain moved
  ahead of the diffusers. The model test found fixed-point lines clipping on loud noise: an all-pass
  state reaches 1/(1-g) times its input (x4 for the 0.75 diffusers) and lines saturate at full scale.
* **Damping is a shelf with a crossover** (`hf_hz`), not Jot's one-pole with the HF RT60 defined at
  Nyquist: below `hf_hz` the tail decays with `rt60_ms`, above it with `rt60_ms * hf_ratio`.
* **Block-wise processing.** Each line runs over up to 16 samples in its own loop, with its state in
  locals. That is exact because every tank line is longer than 16 samples (asserted at compile time for
  the worst case, size 0.25 at 8 kHz); the line loops are compiled once per storage format and called
  through a small table. This cut the code from 9-17 KB to 5-7 KB and the `QUALITY` time by about 20 %.
* **`PCL_NOINLINE`** was added to `pcl_platform.h` for that (GCC/Clang `noinline`, MSVC
  `__declspec(noinline)`).
* **Bench script fix:** `scripts/mcxn947_bench.py` now starts parsing at the *last* banner before the
  case frames. A double reset of the board printed the banner twice with a stray NUL in between, and the
  script stopped at the NUL (the "unexpected byte" failure seen before).

## Parameters

| Parameter | Unit / format | Range | Notes |
|-----------|---------------|-------|-------|
| `sample_rate_hz` | Hz | 8 000 .. 768 000 | |
| `variant` | enum | `PCL_REVERB_SMALL`, `PCL_REVERB_QUALITY` | Fixed at init |
| `size` | `pcl_coef_t` | 0.25 .. 1 | Scales every line: room size and memory. Fixed at init |
| `max_predelay_ms` | ms | 0 .. 500 | Sizes the predelay line. Fixed at init |
| `predelay_ms` | `pcl_ms_t` | 0 .. `max_predelay_ms` | Jumps when set (clicks only in the wet signal) |
| `rt60_ms` | `pcl_ms_t` | 100 .. 30 000 | Decay to -60 dB below `hf_hz` |
| `hf_ratio` | `pcl_coef_t` | 0.1 .. 1 | RT60 above `hf_hz` / `rt60_ms` (1 = no damping) |
| `hf_hz` | `pcl_hz_t` | 1 000 .. 0.45 fs | Damping crossover |
| `width` | `pcl_coef_t` | 0 .. 1 | 0 = mono wet (L == R exactly), smoothed |
| `mix` | `pcl_coef_t` | 0 .. 1 | Linear dry/wet, smoothed; 0 = the input exactly |
| `mod_depth` | `pcl_coef_t` | 0 .. 1 | `QUALITY` only (accepted and ignored by `SMALL`) |
| `freeze` | int | | Line gains 1, damping off, modulation stopped, input faded out over ~5 ms |
| `storage` | `pcl_delay_storage_t` | `NATIVE`, `Q15` | `Q15` = int16_t lines in every build |

Setters: `set_predelay`, `set_rt60`, `set_damping(hf_ratio, hf_hz)`, `set_width`, `set_mix`,
`set_mod_depth`, `set_freeze`. `process(rv, in, out_l, out_r, frames)`; `out_r == NULL` gives the mono
sum `dry + mix (wet_l + wet_r) / 2` in `out_l`; `in` may alias either output.

## Algorithm

* **Line lengths** (size 1): `SMALL` combs 25.31 / 26.94 / 28.96 / 30.75 ms (Freeverb's tunings,
  public domain), all-passes 5.0 / 1.7 ms (L) and 5.3 / 1.9 ms (R), gain 0.7. `QUALITY` lines 31.7,
  37.1, 41.9, 47.3, 53.9, 61.1, 71.3, 83.9 ms; diffusers 4.77 / 3.60 / 12.73 / 9.31 ms with gains 0.75
  / 0.75 / 0.625 / 0.625 (Dattorro's input diffusion). Each length is `us * fs * size` rounded, then
  moved up to the next prime not already used by that reverb (trial division at init; at 48 kHz
  `QUALITY` gets 1523 ... 4027 samples).
* **Gains:** `g = 10^(-3 L / (RT60 fs))` per line, as `pcl_db_to_gain(-60 L / (RT60 fs))`, so fixed point
  needs no float; `g_hf` the same with `RT60 * hf_ratio`. The damping pole is
  `p = exp(-2 pi hf / fs) = pcl_db_to_gain(-54.575 hf / fs)`. `QUALITY` folds `1/sqrt(8)` into both
  gains (rounded down, so the matrix never gains).
* **Hadamard:** a fast Walsh-Hadamard transform, 24 additions. The input enters with the signs of row
  7 (balances L and R: row 1 made L 2x louder than R); the outputs are rows 1 and 2.
* **Modulation:** lines 3 and 6, a 0.6 Hz sine and cosine at control rate, read position moving linearly
  in between (as in the delay), linear interpolation. While frozen the excursion is 0, so the read
  positions glide back to whole samples within one control period: linear interpolation at a moving
  fraction is a low-pass, and with full modulation it drained a frozen tail by 4 dB in 4 s.
* **Levels:** `K_S = 0.1`, `K_Q = 0.165`, wet make-up x2: both variants give the same tail energy at
  equal RT60 (measured).
* **Fixed point:** values Q4.28, coefficients Q2.30; every product in a recursive path rounds toward
  zero (branch-free); lines saturate at full scale; native Q31 lines hold Q4.28 << 3 exactly.
* **Float:** the -480 dB anti-denormal guard on every line write and damping state; lines clamp at +-4
  and store NaN as 0.

## Accuracy (measured)

* **RT60** (T20 of the Schroeder energy decay curve of an impulse response, stereo energy), target
  300 / 1000 / 3000 ms:
  `SMALL` 303 / 998 / 3000 (all builds and storages within 1 %);
  `QUALITY` 317 / 984 / 2916 (float, Q31 and 16-bit), 2992 at 3 s with native float / Q31 lines.
  `QUALITY` reads 6 % long at 300 ms: the diffusers smear the onset. The test accepts 8 %.
* **Damping:** RT60 2 s, `hf_ratio` 0.4 at 2 kHz: the band below 200 Hz decays in 1994-2033 ms, the band
  above 10 kHz in 0.43-0.44 of that (the one-pole shelf is not a brick wall).
* **Against a double-precision model** of both variants (same lengths, gains from the formulas), first
  150 ms of noise: float and Q31 with native lines within 1.5e-6; 16-bit lines (any build) within
  1.7e-4 (about 5.5 LSB of Q15). Tolerances in the test: 5e-6 and 3e-4.
* **Silence:** after a noise burst with RT60 1 s, fixed-point output is exactly 0 within 7 s (all
  variants, both storages, modulation on); float is below -140 dB.
* **Freeze** holds the tail: -0.01 dB over 10 s with full modulation, both storages, all builds (the
  test allows 0.5 dB over 4 s); unfreezing resumes the decay.
* **Stereo:** at width 1, the tail's L/R correlation is below 0.3 (noise input); width 0 gives L == R
  bit-exactly.

## Memory (measured)

`pcl_reverb_required_bytes`, 48 kHz (includes the alignment slack and one predelay slot):

| Variant | size 1 | size 0.5 | size 0.25 | +100 ms predelay |
|---------|-------:|---------:|----------:|-----------------:|
| `SMALL`, 16-bit lines | 12 147 | 6 083 | 3 079 | +9 600 |
| `SMALL`, native float / Q31 | 24 295 | 12 167 | 6 159 | +19 200 |
| `QUALITY`, 16-bit lines | 44 131 | 22 203 | 11 087 | +9 600 |
| `QUALITY`, native float / Q31 | 88 263 | 44 407 | 22 175 | +19 200 |

Proportional to the sample rate (44.1 kHz `SMALL` 16-bit: 11 159 B; 96 kHz: 24 243 B). The Q15 build's
native lines are 16-bit too.

* **State** (`pcl_reverb_t`): 464 B on Cortex-M (568 B on 64-bit hosts). The draft's 150-250 B left out
  the per-line descriptors (13 lines x 16 B) and the three smoothers.
* **Code** (`-O2`, the whole module): Cortex-M33 5.3 KB (Q15), 6.8 KB (Q31), 6.9 KB (float);
  Cortex-M0+ 6.9 KB (Q15), 9.4 KB (Q31). No tables.
* **Stack** (worst chain, `-fstack-usage`): `SMALL` about 420 B on the M33 and 540 B on the M0+;
  `QUALITY` about 1.05 KB (M33) / 1.2 KB (M0+): its 16 x 8 block of line outputs is 512 B in fixed
  point. `init` uses about 600 B.
* So a `SMALL` reverb at size 0.5 with 16-bit lines needs about 7 KB of RAM in all: it fits a 16 KB
  MKM14Z128A in memory (but see "Cost" for its CPU).

## Cost (cycles per sample): MEASURED on a Cortex-M33

FRDM-MCXN947 (Cortex-M33 + FPU, 150 MHz), DWT cycle counter, `-O2`, from flash with the cache on,
48 kHz, 256-frame blocks of noise, size 1, 10 ms predelay, RT60 2 s, `hf_ratio` 0.5 at 4 kHz, width 1,
mix 0.3, `mod_depth` 0.5, stereo out. Bit-exact against the host build in all three types (float:
2048/2048 samples identical).

| Case | Q15 | Q31 | float |
|------|----:|----:|------:|
| `reverb_small_q15` (16-bit lines) | 844 | 867 | 797 (729-859) |
| `reverb_small_native` | 844 | 689 | 522 |
| `reverb_quality_q15` (16-bit lines) | 1522 | 1532 | 1484 (1325-1534) |
| `reverb_quality_native` | 1522 | 1268 | 1080 |

(In the Q15 build both storages are int16_t, so the two rows match. Ranges: fastest and slowest block.)

* **At 48 kHz a 150 MHz M33 spends 17-28 % of its time on `SMALL` and 35-49 % on `QUALITY`.**
* **That is about 1.7x the draft's estimates** (300-500 / 600-1000 fixed point, 200-350 / 400-700
  float). The estimate scaled the delay's per-line cost wrongly: `SMALL` runs 9 lines and `QUALITY` 13,
  and each costs about what one delay line does (float `SMALL` is 6.4 delays, `QUALITY` 13).
* **16-bit lines cost 25-50 % more than native in float and Q31** (the dither, the dead zone and the
  conversions), and save half the RAM.
### MEASURED on a Cortex-M0+ (RP2040)

Raspberry Pi Pico (RP2040, Cortex-M0+, no FPU, 125 MHz), SysTick cycle counter, `-O2`, from flash
through the XIP cache, interrupts off while timing; same cases as above. Bit-exact against the host in
Q15 and Q31; float identical too (2048/2048 samples, soft-float).

| Case | Q15 | Q31 | float (soft) |
|------|----:|----:|-------------:|
| `reverb_small_q15` (16-bit lines) | 2 763 | 2 850 | 12 368 |
| `reverb_small_native` | 2 752 | 2 564 | 9 117 |
| `reverb_quality_q15` (16-bit lines) | 4 565 | 4 683 | 20 761 |
| `reverb_quality_native` | 4 555 | 4 207 | 15 204 |

For scale on the same chip (Q15): `delay_linear` 331, `svf_static` 470, `ladder_lp24` 960.

* **`SMALL` in Q15 needs 2 763 cycles per sample: 106 % of a 125 MHz M0+ at 48 kHz, 66 % at 32 kHz,
  49 % at 22.05 kHz.** On the M0+ it is only usable at reduced sample rates as built; a 48 MHz part
  (the MKM14Z128A class) would manage about 17 kHz with nothing else running.
* The cause is the 64-bit products: the M0+ has no 32x32->64 multiply, so each Q4.28 x Q2.30 product
  is an `__aeabi_lmul` call (about 31 per sample for `SMALL`). The M33 is 3.3x faster per cycle here.
* Float on the M0+ (soft-float) is 3-4x slower again; it is not a configuration for this core.

## Rate

Audio rate, in blocks of up to 16 samples. The `QUALITY` modulation LFO runs at control rate
(`PCL_CONTROL_DIV`, counted from the start of the stream) with the read positions moving linearly in
between; mix, width and the freeze input fade are smoothed per sample over ~`PCL_SMOOTH_MS`; RT60,
damping and predelay switch at once (the decay rate changes, which does not click).

## Tests (`tests/test_reverb.c`: 1712 checks float, 1694 Q15 / Q31)

* `required_bytes` / `init`: every range, NaN (float), NULL, a buffer one byte short, an unaligned
  buffer, the struct untouched on error; 8 kHz at size 0.25 and 192 kHz initialize; sizes scale with
  size, storage and predelay.
* Setters: every range, NULL, unchanged on error.
* Line lengths: distinct primes for both variants at 4 rates and 3 sizes.
* RT60 within 8 % for both variants at 300 / 1000 / 3000 ms and both storages.
* Damping: low band within 10 % of RT60, high band 0.35-0.55 of it for a set ratio of 0.4.
* Silence: exact 0 in fixed point, no denormals in float, after a noise burst (modulation on).
* Freeze (modulation at full depth): held within 0.5 dB over 4 s, decays again after release.
* Predelay and first arrival: the first non-zero wet sample is at exactly predelay + the shortest tank
  line, with the predelay at 25 ms and at 0.
* Width 1: L/R correlation below 0.3; mono output = the average of L and R; width 0: L == R.
* Mix 0 = the input exactly; a mix change ramps then matches a wet-only reverb exactly.
* Blocks of 1, 7, 16, 17, 256 bit-identical to one block (both variants, both storages, modulation
  on); `reset` bit-identical; in place; modulation changes `QUALITY` and leaves `SMALL` alone.
* Against a double model (above).
* Extremes: RT60 30 s, size 0.25, full-scale square input, freeze toggled, damping and RT60 thrown
  around: finite, no denormals, float output below 16.

## Render patches (`render reverb`)

Six stereo files: `reverb_{small,quality}_{snare,pad,pluck}.wav` (`SMALL` with 16-bit lines,
`QUALITY` with native lines). Noise bursts (RT60 1.5 s, damped above 5 kHz), a two-saw chord held 2 s
(RT60 5 s, full modulation), a plucked saw melody (RT60 2.5 s). At 16-bit output the three builds
produce the same levels and L/R correlation (0.0-0.26; 0.70 for `QUALITY` on the held chord). Use 5 s or
more (`render reverb out 7`). Not yet listened to by a person.

## Known limits

* **CPU** (see "Cost"); the M0+ case especially.
* **16-bit lines shorten the last part of a quiet tail**: values below 2 LSB (-84 dBFS) truncate toward
  zero, so a tail that starts quiet ends sooner (`QUALITY` 3 s from one 0.9 impulse, Q15 build: T20
  2.92 s, T30 2.68 s; `SMALL` 3.00 / 2.99 s).
* **Wet level grows as size shrinks** at equal RT60 (shorter lines, more echoes per second): +2.9 /
  +5.7 dB (`SMALL`) and +3.3 / +6.4 dB (`QUALITY`) at size 0.5 / 0.25, noise in. Not compensated.
* **Long RT60 with sustained loud input clips the wet output** in fixed point: at RT60 30 s and mix 1,
  white noise at -17 dBFS RMS already peaks at 1.3 x full scale (float; fixed point saturates there).
  The energy a reverb accumulates grows with RT60; there is no limiter.
* **Float does not keep NaN out completely**: a NaN input is stored as 0 in the lines, but the dry
  path passes it through, like every module.

## Open points (for review)

1. **Speed.** If the measured cost is too high, options in rough order of payoff:
   (a) a 32-bit Q15 path (Q15 x Q15 products, `pcl_mul_q15_q30`-style), which matters most for the M0+
   (measured: `SMALL` Q15 is 2 763 cycles/sample there, more than a 125 MHz RP2040 has at 48 kHz) and
   would also speed up the M33 Q15 build; this is a second arithmetic path with its own accuracy
   tests;
   (b) float: drop the +-4 clamp on line writes (the delay doesn't clamp either), about 10-20 %;
   (c) the half-rate tank from the draft (question 5), about half the cost and memory;
   (d) fewer `QUALITY` lines (4 lines and Hadamard-4 would roughly halve it, at some loss of density).
2. **Listening.** The render patches have only been checked by numbers.

## References

* M. R. Schroeder, "Natural sounding artificial reverberation," J. Audio Eng. Soc. 10(3), 1962.
* J. A. Moorer, "About this reverberation business," Computer Music Journal 3(2), 1979.
* J.-M. Jot, A. Chaigne, "Digital delay networks for designing artificial reverberators," AES 90th
  Convention, 1991.
* J. Dattorro, "Effect design, part 1: Reverberator and other filters," J. Audio Eng. Soc. 45(9), 1997
  (input diffusion lengths and gains).
* J. O. Smith, *Physical Audio Signal Processing*, CCRMA, 2010 (Schroeder/Freeverb structures, FDNs,
  Hadamard feedback matrices).
* Jezar at Dreampoint, Freeverb (2000, public domain): comb tunings.
* Stochastic rounding: e.g. M. Croci et al., "Stochastic rounding: implementation, error analysis and
  applications," R. Soc. Open Sci. 9, 2022.

Citations are from memory and have not been checked against the sources.
