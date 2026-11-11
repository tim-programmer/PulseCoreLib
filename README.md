# PulseCoreLib

A portable DSP library for music and sound synthesis. The same C99 source builds on desktop
(Windows/macOS/Linux) and on microcontrollers, including parts with no FPU. Design rules and
conventions are in [CLAUDE.md](CLAUDE.md).

## Building

Pick a sample type at configure time: `FLOAT`, `Q15` or `Q31`.

```sh
cmake -B build-float -DPCL_SAMPLE_TYPE=FLOAT && cmake --build build-float && ctest --test-dir build-float
cmake -B build-q15   -DPCL_SAMPLE_TYPE=Q15   && cmake --build build-q15   && ctest --test-dir build-q15
cmake -B build-q31   -DPCL_SAMPLE_TYPE=Q31   && cmake --build build-q31   && ctest --test-dir build-q31
```

Embedded compile checks (need `arm-none-eabi-gcc`). They build the library only. The Cortex-M0+
presets also fail if a fixed-point build references the soft-float runtime; the Cortex-M33
presets (hard-float FPU, e.g. NXP MCX N947) skip that check:

```sh
cmake --preset cm0plus-q15 && cmake --build --preset cm0plus-q15
cmake --preset cm0plus-q31 && cmake --build --preset cm0plus-q31
cmake --preset cm33-float  && cmake --build --preset cm33-float
cmake --preset cm33-q31    && cmake --build --preset cm33-q31
```

CMake options: `PCL_SAMPLE_TYPE` (default `FLOAT`), `PCL_BUILD_TESTS`, `PCL_BUILD_EXAMPLES` and
`PCL_BUILD_TOOLS` (all default `ON`; the embedded presets turn them off), `PCL_ALLOW_FLOAT_INIT`
(default `OFF`; lets init/config code use float in Q15/Q31 builds, never process code).

## Examples

`examples/sweep_wav.c` renders a slow exponential 20 Hz to 20 kHz sweep (48 kHz, mono 16-bit) with
the oscillator's pitch input and a VCA fade, in whichever sample type the build uses:

```sh
build-float/examples/sweep_wav sweep.wav saw 20     # sine | triangle | saw | pulse, seconds
```

It is also run as a short smoke test by `ctest`. Turn the volume down before playing the top end.

`tools/render` writes listening-test files. `render noise <outdir> [seconds] [seed]` produces 5 s each of
white, pink and brown noise (equal loudness) and two sample-and-hold-driven pitch files; `render lfo
<outdir>` writes five patches (tremolo, vibrato, PWM, wobble, random pitch) that use the LFO; `render adsr
<outdir>` writes five envelope patches (pluck, pad, drum from 1-sample triggers, curves, retrigger);
`render svf <outdir>` writes four filter patches (an ADSR-swept saw, a noise wah, a self-oscillating
filter glided from 440 to 880 Hz, and a complete voice: oscillator, filter and VCA with two envelopes);
`render biquad <outdir>` four EQ patches (a swept peaking EQ, a shelf pair, a swept notch, a 4th-order
Butterworth from two instances); `render ladder <outdir>` three ladder-filter patches (a saturated
24 dB low-pass swept by an ADSR, self-oscillation started from a click at 440 then 880 Hz, and a saw
through all eight modes in turn); `render delay <outdir>` three delay patches (an echo with feedback and
tone, a chorus and a flanger); `render sampler <outdir>` three sampler patches from synthesized samples (a
one-shot melody, a sustain loop with release, a ping-pong pad); `render events <outdir>` three patches that run the clock, the
arpeggiator and the step sequencer into a voice (an arpeggiator over four chords in four modes, a 16-step
acid line with ties, rests and swing, and a ping-pong run that turns random):

```sh
mkdir -p out && build-float/tools/render noise out && build-float/tools/render lfo out \
    && build-float/tools/render adsr out && build-float/tools/render svf out \
    && build-float/tools/render biquad out && build-float/tools/render ladder out \
    && build-float/tools/render delay out && build-float/tools/render sampler out \
    && build-float/tools/render events out
```

`tools/play_sample <in.wav> <out.wav> [root_note]` plays a short melody from any 16-bit PCM WAV file through
the sampler (in sustain mode if the file has a `smpl` loop), for listening to the sampler with real
recordings. The WAV reader (`tools/wav_read.c`) is host-only, like every file-I/O path.

## Testing on a FRDM-MCXN947

Builds the library for the board's Cortex-M33, flashes it, runs the oscillator, noise,
sample-and-hold, VCA, LFO, ADSR, math, state-variable filter, biquad, ladder, delay and sampler benchmark cases on the chip, and checks the result against the host build - one
command, no manual steps.

```sh
scripts/setup_mcxn947.sh                      # once: pyOCD + pyserial venv, flash pack, NXP SDK
# plug in the board's MCU-Link (debug) USB port, then:
.venv-tools/bin/python scripts/mcxn947_bench.py --type q15     # or q31, float
```

What it does: builds `platform/mcxn947` (preset `mcxn947-<type>`, `-O2`) and the host reference
`tools/bench_ref`; flashes with pyOCD over the on-board MCU-Link; resets the board and reads its
streamed results from the MCU-Link virtual serial port (921600 baud); then reports, per case,
whether the CRC and every sample match the host build (bit-exact for Q15/Q31, 1e-6 for float)
together with measured cycles per sample from the core's DWT counter. The exit status is
non-zero on any mismatch, so it can run from a script or CI with a board attached.

Notes:

* The NXP SDK is fetched into `third_party/` (pinned commits, ~30 MB, gitignored, not vendored)
  and only used under `platform/`. The core library does not depend on it.
* This overwrites the board's flash. The MicroPython image that shipped on the board was saved
  to `backups/frdm-mcxn947_micropython_flash_*.bin` (gitignored); restore it with
  `.venv-tools/bin/pyocd flash -t mcxn947vdf --format bin --base-address 0 backups/<file>.bin`.
* Audio output is not wired up yet: this stage streams samples to the PC instead of a DAC/I2S.
* Options: `--no-build`, `--no-flash`, `--port`, `--baud`; `-DPCL_MCXN947_DISABLE_LPCAC=ON`
  at configure time turns the flash cache off to see its effect on the cycle counts.

## Regenerating the lookup tables

`src/core/pcl_tables.c` is generated (C99 cannot evaluate `sin`/`exp2` at compile time) and checked in:

```sh
build-float/tools/gen_tables > src/core/pcl_tables.c
```

## Using the library

```c
#include "pcl/pcl.h"

pcl_vca_t vca;                       /* caller-owned state, no heap */
pcl_vca_config_t cfg = { 48000u, PCL_COEF(0.5) };
pcl_vca_init(&vca, &cfg);

pcl_vca_process(&vca, in, out, NULL /* no modulation */, frames);
```

Link against the `pcl::pcl` target; it carries the include path and the sample-type definition.

## Modules

| Module | Header | Rate | Float | Q15 | Q31 | Design note |
|--------|--------|------|:-----:|:---:|:---:|-------------|
| Oscillator (sine, triangle, saw, pulse; pitch/PWM/sync) | `pcl_osc.h` | audio (pitch/PWM control rate) | yes | yes | yes | [oscillator.md](docs/modules/oscillator.md) |
| Unison oscillator (up to 8 detuned `pcl_osc` voices, seeded start phases, mono or constant-power stereo spread) | `pcl_unison.h` | audio (pitch/PWM control rate) | yes | yes | yes | [unison.md](docs/modules/unison.md) |
| ADSR envelope (exponential/linear stages, gate buffer or events, `run_through`) | `pcl_adsr.h` | control (interpolated to audio) | yes | yes | yes | [adsr.md](docs/modules/adsr.md) |
| LFO (sine, triangle, saw up/down, square, random step/smooth; rate input) | `pcl_lfo.h` | control (interpolated to audio) | yes | yes | yes | [lfo.md](docs/modules/lfo.md) |
| Noise: white, pink (Voss-McCartney), brown | `pcl_noise.h` | audio | yes | yes | yes | [noise.md](docs/modules/noise.md) |
| Random sample-and-hold (clock / events / gate trigger, slew) | `pcl_noise.h` | sample-accurate triggers | yes | yes | yes | [noise.md](docs/modules/noise.md) |
| VCA (gain + modulation) | `pcl_vca.h` | audio | yes | yes | yes | [vca.md](docs/modules/vca.md) |
| Ladder filter (4-pole zero-delay-feedback Moog-style VCF: LP24/18/12/6, BP12/24, HP12/24 by pole mixing; resonance to self-oscillation, optional tanh drive, bass compensation) | `pcl_ladder.h` | audio (coefficients at control rate) | yes | yes | yes | [ladder.md](docs/modules/ladder.md) |
| State-variable filter (VCF: LP/BP/HP + notch/peak/all-pass, resonance to self-oscillation, exponential cutoff modulation) | `pcl_svf.h` | audio (coefficients at control rate) | yes | yes | yes | [svf.md](docs/modules/svf.md) |
| Biquad filter (RBJ cookbook: LP, HP, BP, notch, all-pass, peaking EQ, low/high shelf; error-feedback fixed point) | `pcl_biquad.h` | audio (coefficients at control rate) | yes | yes | yes | [biquad.md](docs/modules/biquad.md) |
| Clock (tempo, 1..96 pulses per quarter, sample-accurate pulses with no drift, START/STOP) | `pcl_clock.h` | event | yes | yes | yes | [clock.md](docs/modules/clock.md) |
| Arpeggiator (up, down, up-down, random, as played, chord; 1..4 octaves, latch, gate/division in pulses) | `pcl_arp.h` | event | yes | yes | yes | [arp.md](docs/modules/arp.md) |
| Step sequencer (up to 64 steps; rests, ties, per-step gate, swing, transpose; forward/backward/ping-pong/random) | `pcl_seq.h` | event | yes | yes | yes | [sequencer.md](docs/modules/sequencer.md) |
| Sampler (one voice: caller-owned mono sample in flash or RAM, MIDI note pitch, octave pitch modulation, one-shot / forward / ping-pong / sustain-with-release loops, reverse, sample-accurate note events, declick) | `pcl_sampler.h` | audio (pitch modulation at control rate) | yes | yes | yes | [sampler.md](docs/modules/sampler.md) |
| Delay (echo, chorus, flanger, vibrato: tape-style glide, time modulation, feedback with tone and saturation, dry/wet mix; linear or Hermite interpolation; caller memory, optional 16-bit storage) | `pcl_delay.h` | audio (modulation at control rate) | yes | yes | yes | [delay.md](docs/modules/delay.md) |
| Math: pitch and dB conversions, sin/cos, filter pre-warp `tan`, `tanh` | `pcl_math.h` | setters / audio | yes | yes | yes | [math.md](docs/modules/math.md) |

Shared pieces: `pcl_types.h` (sample types, status codes, saturating fixed-point helpers),
`pcl_platform.h` (compiler/CPU detection, `PCL_INLINE`, `PCL_RESTRICT`, `PCL_ALIGNED`),
`pcl_smooth.h` (linear parameter smoothing), `pcl_event.h` (`pcl_event_t` for event-driven modules). Frequencies and octave amounts use `pcl_hz_t` /
`pcl_oct_t` (float, or Q16.16 / Q4.28 in fixed-point builds); build constants with `PCL_HZ(x)` and
`PCL_OCT(x)`. Levels in decibels and signals that may exceed full scale use `pcl_db_t` (Q8.24) and
`pcl_wide_t` (Q4.28), with `PCL_DB(x)` / `PCL_WIDE(x)`; a filter's quality factor uses `pcl_quality_t`
(Q16.16) with `PCL_QUALITY(x)`.
