# Sampler (sample playback voice)

Files: `include/pcl/pcl_sampler.h`, `src/sampler/pcl_sampler.c`, `tests/test_sampler.c`; the shared
interpolation kernels `src/core/pcl_interp.h` (moved out of the delay line); the `sampler` subcommand of
`tools/render.c`; `tools/play_sample.c` with the WAV reader `tools/wav_read.c`; and the `sampler_*` cases
of `platform/bench/pcl_bench.c`.

**Status: implemented in float, Q15 and Q31; all tests pass in all three builds** (Clang with ASan +
UBSan, GCC `-O2`, the `cm0plus-*` / `cm33-*` compile checks including the no-soft-float check, and the
`mcxn947-*` firmware). Pitch, timing, sizes and host speed are **measured**. **Cortex-M33 cycles are not:**
the bench cases exist, but the board was disconnected by the time they were ready. M0+ is unmeasured,
as for every module.

## Summary

One playback voice: a caller-owned mono sample played at a MIDI note's pitch, with octave pitch
modulation, four loop modes, reverse playback, sample-accurate note events and declicked starts and
stops.

```
step = (sample rate / output rate) * 2^((note - root) / 12) * 2^(pitch_mod * depth)   Q8.24
pos += step                                       int64_t frames, Q.24
out  = gain * velocity_gain * declick * interp(data, pos)      linear or 4-point Hermite
```

* **The data is the caller's** and is never copied: a `pcl_sampler_sample_t` points at mono `int16_t`
  or `pcl_sample_t` frames and carries the recording's rate, root note and loop points. Both can live
  in flash. There's no caller buffer and no sizing function (decision 8).
* **A voice, not an instrument.** Polyphony means several voices; envelopes come from `pcl_adsr` +
  `pcl_vca`.
* **Integer positions and steps in every build.** The note's step comes from a 12-entry semitone
  table (Q2.30) and the recording/output rate ratio (one 64-bit division when the sample is set);
  `pitch_mod` scales it with the shared `2^x` table (`pcl_exp2m1_q16`, now available to float builds
  as well). Without modulation, float, Q15 and Q31 play from exactly the same positions; with it they
  differ only by the rounding of `pitch_mod x depth`.
* **Events like the ADSR's:** `NOTE_ON` (note, velocity) and `NOTE_OFF` at their sample offsets. A
  `NOTE_ON` with velocity 0 is a `NOTE_OFF` (MIDI convention). `pcl_sampler_note_on/off` do the same
  from the control thread; `pcl_sampler_is_playing` tells a voice allocator when the voice is free.

## Decisions (from the review of the draft)

All ten were settled with the recommendation: aliasing on upward pitch accepted and documented; linear
and Hermite sharing the delay's kernels; four loop modes; reverse playback; no loop crossfades; key
zones left to the caller; Q15 and native mono data; no `required_bytes`; declick always available,
`declick_ms` 0 .. 20; a WAV reader in `tools/` only.

### What changed from the draft while building

* **The kernels take the fraction width as a parameter.** The delay keeps its 12 bits, so its output is
  bit-identical. A render of 32 configurations (3.8 MB per build) and all 66 bench cases match byte for
  byte in all three builds. The sampler uses 16 bits of its 24-bit fraction, which puts the position error at
  -98 dB at fs/4 instead of -74 dB.
* **Edges replicate the end frames** rather than reading 0 before the start (the draft said 0). That's
  consistent with how a one-shot ends: when it runs off the data, the voice holds the last frame and
  fades it out over the declick, so a sample that doesn't end at zero doesn't click.
* **ONE_SHOT ignores NOTE_OFF** (drum-style: it always plays to the end). The draft didn't say; use
  FORWARD or an envelope for gated playback.
* **A retrigger overlaps two playheads.** The sounding note moves to a second, "tail" playhead and fades
  out over `declick_ms` while the new note fades in from the event's sample. A third trigger within one
  declick cuts the oldest tail (a possible small click, documented). This is what the draft described;
  it costs a second read during the fade.
* **Loops engage when the playhead crosses a loop point**, not merely when it is beyond one: a
  `start_offset` past `loop_end` plays to the end of the sample rather than jumping into the loop.
* **A bug the tests found:** with a loop that ends at the last frame (`loop_end == frames`), positions
  between the last frame and `loop_end` are valid (they interpolate across the seam), but the
  end-of-data check stopped the note there. Integer steps jump over that gap, so +12 and +24 semitones
  passed and +7 failed (the note fell silent at 0.77 s). The check now ignores positions inside an
  active forward or sustain loop.

## Parameters

| Parameter | Type | Range | Notes |
|-----------|------|-------|-------|
| `sample_rate_hz` | `uint32_t` | 8000 .. 768000 | Output rate |
| `sample` | `const pcl_sampler_sample_t *` | | Used from the next `NOTE_ON`; must stay valid while it plays |
| `interp` | enum | `PCL_SAMPLER_LINEAR`, `PCL_SAMPLER_HERMITE` | Switches at once |
| `pitch_depth_oct` | `pcl_oct_t` | 0 .. < 8 | Octaves per unit of `pitch_mod` |
| `gain` | `pcl_coef_t` | 0 .. < 2 | Smoothed per sample |
| `velocity_sens` | `pcl_coef_t` | 0 .. 1 | Note gain = `1 - sens + sens * velocity / 127` |
| `start_offset` | `uint32_t` frames | 0 .. frames - 1 | From the end when reversed; clamped to a later, shorter sample |
| `reverse` | `int` | 0 / non-zero | From the next `NOTE_ON` |
| `declick_ms` | `uint32_t` | 0 .. 20 | Linear fades at starts, stops, retriggers and one-shot ends |

Sample description: `data` non-NULL, `frames >= 1`, `format` Q15 or native, `sample_rate_hz` 8000 ..
768000, `root_note <= 127`, `loop_mode` one of `ONE_SHOT`, `FORWARD`, `PING_PONG`, `SUSTAIN`; loop modes
need `loop_start < loop_end <= frames` and a loop of at least 2 frames. `init` and the setters return
`PCL_ERR_NULL_POINTER` / `PCL_ERR_OUT_OF_RANGE` (NaN included) and leave the voice unchanged.

## Algorithm

* **Step:** `base = (sample_rate << 24) / output_rate` (at most 96 in Q8.24), times the semitone table
  entry and shifted by whole octaves, saturating at 256 frames per sample (`0xFFFFFFFF`). At each control
  point, `step = step_note * 2^(pitch_mod * depth)`; within the period it moves linearly and lands
  exactly on the target. A note started mid-period begins at the current modulation.
* **Reads:** the integer frame, the top 16 bits of the fraction, and the kernels. A fast path reads the
  data directly; near the ends or a loop seam, taps wrap across the seam (forward and sustain loops,
  while the playhead is inside the loop) or replicate the edge frames.
* **Loops:** forward and sustain subtract the loop length when crossing `loop_end` (adding it when
  crossing `loop_start` in reverse), so the fraction is kept and the loop length is exact. Ping-pong
  reflects at `loop_start` and at the last loop frame (`loop_end - 1`). A step larger than the loop
  falls back to a modulo, which is rare. `NOTE_OFF` ends a sustain loop (play on to the end) and fades
  out forward and ping-pong loops.
* **Mix:** each active playhead's sample times `gain x velocity gain x declick`, summed and saturated
  (fixed point).
* **Specialisation:** the render loop is force-inlined once per interpolator (the delay's board numbers
  showed that run-time branches in the loop are costly on the M33).

## Accuracy (measured)

* **Exact playback:** at the root note and the recorded rate the output equals the data, sample for
  sample, in all builds, both formats, both interpolators, forwards and reversed, from a start offset.
* **Pitch:** a looped 375 Hz sine played at 0, +1, -5, +7, +12, -12 and +24 semitones, and recorded at
  24 kHz, comes out within 2e-6 of the exact frequency (0.003 cents). `pitch_mod` of half an octave is
  within 1e-5 (the `2^x` table, as in the oscillator).
* **No loop drift:** a one-cycle loop (128 frames) played 5 semitones up for 3 s (1502 wraps) measures
  500.56495 Hz, exactly the expected value to the printed digits.
* **Against a double model at the sampler's own positions:** linear and Hermite within 4.5e-7 in float
  and Q31, and 1.6e-5 / 2.6e-5 in Q15 (under one LSB).
* **Formats:** Q15 and native data with the same values play bit-identically.

## Memory (measured)

* **State:** `sizeof(pcl_sampler_t)` = **176 B** in all builds (two playheads; estimate 100-130 B).
* **No buffer; tables:** a 12-entry semitone table (48 B).
* **Sample data** (exact): 96 000 B per second of 48 kHz mono in Q15, 192 000 B native float/Q31.
* **Code** (`pcl_sampler.c`, text, arm-none-eabi-gcc 15 `-O2`): **5344 B (M0+ Q15), 5648 B (M0+ Q31), 4256 B
  (M33 float), 4516 B (M33 Q31)**, above the 3-5 KB estimate (two inlined loop variants). The M0+ objects
  use only integer runtime helpers, and the soft-float check passes.
* **Stack in `process`:** 408 / 416 B (M0+ Q15 / Q31), 104 / 112 B (M33 float / Q31).

## Cost

**Host (x86-64, GCC `-O2`, a 4096-frame sine table looped, 7 semitones up, 256-frame blocks), ns per
sample, with the SVF and the delay in the same run:**

| Case | float | Q15 | Q31 |
|------|------:|----:|----:|
| `sampler_linear` | 7.4 | 9.1 | 8.9 |
| `sampler_hermite` | 8.4 | 11.1 | 10.9 |
| `sampler_hermite_mod` | 8.6 | 11.3 | 11.5 |
| SVF static / delay linear | 7.9 / 3.2 | 8.4 / 6.0 | 7.8 / 5.7 |

Scaling by those two modules' measured M33 cycles gives **roughly 175-320 cycles per sample in fixed
point**, above the draft's 60-180 estimate. The float scaling disagrees between the two references
(60-230), so I won't quote one. **Not measured on the board:** run `scripts/mcxn947_bench.py` (cases
`sampler_linear`, `sampler_hermite`, `sampler_hermite_mod`). Likely costs, not separated: the two-playhead
loop (the tail is checked every sample), two coefficient multiplies per playhead per sample
(`gain x velocity x declick`), and the per-sample gain smoother. Collapsing the gain chain into one
coefficient updated only when something changes is the first optimisation to try if the board numbers
call for it.

## Rate

Audio rate; `pitch_mod` at control rate with the step interpolated; the gain smoother per sample; events
sample-accurate. Stated in the header.

## Tests (`tests/test_sampler.c`: 248 checks float, 244 Q15 / Q31)

* Init and setters: every invalid sample-description field (through `init` and `set_sample`), every
  invalid parameter, NaN in float; unchanged on error; the edges accepted (a 2-frame loop, a one-shot
  with nonsense loop points).
* Exact playback, pitch, no loop drift, the double model and formats, as above.
* **Loop sequences** at step 1 over a ramp: ping-pong's exact frame order; a reversed forward loop
  wrapping down; sustain looping while held, then playing through 20..99 after `NOTE_OFF` and stopping;
  forward fading out over exactly the declick after `NOTE_OFF`.
* **Events:** `NOTE_ON` at offset 37 starts at sample 37 exactly, `NOTE_OFF` at offset 5 silences from
  sample 5, events beyond the block are ignored, velocity 64 at sensitivity 1 scales by 64/127, and
  velocity 0 stops the note.
* **Declick:** three overlapping triggers of a sine at large signal values: the largest step is 1.3 x
  the sine's own (two overlapping notes at different pitches); the last note stops when expected.
* Blocks of 1, 7, 16, 17 and 256 bit-identical to one block, with ten events (including two notes one
  sample apart), `pitch_mod`, Hermite and a sustain loop; `reset` repeats the output; a zero `pitch_mod`
  equals `NULL`; `pitch_mod` aliasing `out`.
* Extremes: the step saturating at 256 frames per sample with +-7.9 octaves of noise modulation, and the
  slowest possible step: finite, no denormals.
* Tool smoke tests: `render sampler`, and `play_sample` reading the WAV that the render step wrote.

All 32 tests (28 suites, the render smoke tests, `play_sample`) pass in the float, Q15 and Q31 host
builds; `test_sampler` also runs clean under Clang ASan + UBSan and GCC `-O2`. Not done: mutation testing,
MSVC, the board run.

## Render patches and tools

`render sampler` synthesizes its samples (no audio files committed). Measured from the files:

| Patch | What it is | Measured |
|-------|------------|----------|
| `sampler_melody.wav` | plucked C4 tone, one-shot, melody 0 4 7 12 7 4 0 | note pitches within 0.09 cents (DFT peak); peak 0.71 |
| `sampler_sustain.wav` | 375 Hz tone, 128-cycle sustain loop, held 1.2 s, then the recorded decay | **375.0000 Hz**; level identical at four points while held (seamless loop); decays 0.34 -> 0.11 -> 0.04 after the release |
| `sampler_pingpong.wav` | beating pad looped back and forth, two notes a fifth apart | peak 0.57, no clipping |

Identical levels in float, Q15 and Q31. (A quick autocorrelation estimate first suggested 1-3 cents
sharp in every build; zero-crossing and DFT measurements show that was the estimator, not the sampler.)

`play_sample <in.wav> <out.wav> [root_note]` plays a melody from any 16-bit PCM WAV (stereo is averaged),
in sustain mode if the file has a `smpl` loop. Host-only, as is its reader.

## Known limits

* **Aliasing:** interpolation does not band-limit, so a sample played more than about an octave up
  folds its top octave back down (decision 1).
* M33 cycles not measured; M0+ not at all.
* A third retrigger within one declick time cuts the oldest fading tail.
* Mono only; stereo means two voices fed with the same events.

## References

* J. O. Smith, *Physical Audio Signal Processing*, CCRMA, 2010 (interpolated table lookup).
* T. I. Laakso et al., "Splitting the unit delay," IEEE Signal Processing Magazine 13(1), 1996.
* D. C. Massie, "Wavetable sampling synthesis," in M. Kahrs and K. Brandenburg (eds.), *Applications
  of Digital Signal Processing to Audio and Acoustics*, Kluwer, 1998.

All citations are from memory and have not been checked against the sources.
