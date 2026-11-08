# Clock

Files: `include/pcl/pcl_clock.h`, `src/event/pcl_clock.c`, `tests/test_clock.c`, `tests/test_event_chain.c`; the event types
`PCL_EVT_START` / `PCL_EVT_STOP` in `include/pcl/pcl_event.h`; the `events` subcommand of
`tools/render.c`.

**Status: implemented; tests pass in float, Q15 and Q31** (Clang ASan + UBSan, the `cm0plus-*` and
`cm33-*` compile checks). The module is pure integer code, so the three builds produce **identical
events**. Cortex-M0+ cost is unmeasured (no hardware).

> **Written without review.** The user asked for the event modules and went to bed, so the design
> choices below were made by the author and are listed for review in the morning. They are marked
> **[D]**. Everything here is easy to change; none of it is used by anything but the arpeggiator,
> the sequencer and their tests so far.

## Summary

A tempo clock for the event modules. Each `process(frames)` call returns the `PCL_EVT_CLOCK` pulses
that fall in that block at their **sample-accurate offsets**, optionally preceded by `PCL_EVT_START` or
`PCL_EVT_STOP` at offset 0. The arpeggiator and the sequencer count these pulses; everything they do
lands on the offset of a pulse, so the whole event chain shares one timeline.

* **Event rate.** `process()` costs one loop iteration per pulse, not per sample.
* No state outside the struct, no heap, no libm, no float in `process` (in any build).

## Parameters

| Parameter | Type | Range | Notes |
|-----------|------|-------|-------|
| `sample_rate_hz` | `uint32_t` | `PCL_SAMPLE_RATE_MIN_HZ` .. `MAX_HZ` | |
| `bpm` | `pcl_hz_t` | 10 .. 999 | beats (quarter notes) per minute, `PCL_HZ(120.0)`. **[D]** A `pcl_hz_t` is a plain number here (float, or Q16.16), not a frequency: it gives 1/65536-bpm resolution in every build without a new type. |
| `ppqn` | `uint32_t` | 1 .. 96 | pulses per quarter note; 24 is the MIDI clock, 96 resolves a 16th note into 24 pulses |
| `running` | `int` | | non-zero: the first block carries START |

Setters (`set_bpm`, `set_ppqn`) validate and leave the clock unchanged on error (NaN included).

## Algorithm

The period is `fs * 60 / (bpm * ppqn)` samples, stored as a **Q32.32 integer**, computed by a long
division (`clock_period`: the integer part and then two 16-bit steps of fraction; no floating point, no
`__int128`). Pulse `k` of a run lands in the sample `floor(k * period)`: the position is exact up to
the 2^-32 truncation of the period, so **nothing accumulates**. 120 bpm at 44.1 kHz and 24 ppqn is
918.75 samples per pulse, and an hour of it has exactly the right pulse count (`test_long_run_does_not_drift`).

`next` is the time from the start of the next block to the next pulse (Q32.32). `process` emits while
`next < frames`, adds the period, and finally subtracts the block length.

Transport:

| Call | Effect at the next `process()` |
|------|-------------------------------|
| `start()` | `START` and a pulse at offset 0; the phase restarts, so **the first pulse is at once** **[D]**. Also restarts a running clock. |
| `stop()` | `STOP` at offset 0 (only if it was running), then no events |
| both before one `process()` | the last call wins |

Tempo changes apply from the pulse after the next one: pulses already scheduled keep their time **[D]**
(tested: `test_tempo_change`).

## Event conventions (shared by all event modules)

These are in the header of `pcl_event.h` and were fixed here because the clock is the first producer:

* Events are **sorted by offset**; the same-offset order of a producer is `STOP, NOTE_OFF, START,
  CLOCK, NOTE_ON`, so a note is released before the next one starts and a transport event is seen
  before the pulse it belongs to.
* `NOTE_ON` with velocity 0 means `NOTE_OFF`.
* Blocks are at most **65536 samples** (the offset is 16 bit).
* Consumers ignore types they do not know and **pass nothing through** **[D]**: the arpeggiator and
  the sequencer produce new streams. A caller who wants the clock in the output merges the arrays.
* **Capacity policy [D]:** events that do not fit the caller's buffer are **dropped and counted**
  (`pcl_clock_dropped`), the module's state still advances, and the buffer is never written past its
  end. `pcl_clock_max_events(frames)` is an upper bound that is reached (`test_max_events_is_tight`),
  so a caller who sizes the buffer with it never loses an event.

## Memory and cost

`sizeof(pcl_clock_t)` is 40 bytes; the code is 0.6 KB (`-Os`) / 0.9 KB (`-O2`) on a Cortex-M0+ (measured with `arm-none-eabi-gcc`, not run). There are no tables. `process()` is a handful of 64-bit additions
and comparisons per pulse; the only 64-bit divisions are in the setters (`clock_period`) and
`pcl_clock_max_events`, which run on the control side. (On a Cortex-M0+ the setter's 64-bit division
is a library call of a few hundred cycles; it is not in the audio path.)

## Tests (`tests/test_clock.c`, 10 cases, ~226,000 checks)

* init/setter validation: every limit accepted, one beyond rejected, NaN, state untouched on error;
* **pulse positions against exact arithmetic** for eight sample rate / tempo / resolution cases (up to
  10 minutes of pulses): position within `[0, 1)` of `k * period`;
* exact spacing where the tempo divides the rate, and the 918.75 case; one hour without drift;
* **block-size independence** (1, 7, 64, 255, 1000, 65536 samples give the same pulses);
* transport: start, restart, stop, stop on a stopped clock, start-then-stop, stop-then-start, reset,
  `running` at init;
* a random stream of block sizes and transport calls: sorted, in range, START/STOP before CLOCK at one
  offset, note/velocity fields zero;
* tempo change spacing; `max_events` tight and sufficient; a too-small buffer (partial write, counted
  drops, untouched tail, state unaffected; zero capacity).

Mutation testing (operator and constant mutations of `pcl_clock.c`, float and Q15 builds): every
survivor is an equivalent mutant (boolean flags stored as 1/2, the redundant `next = 0` stores) or code
excluded from that build.

**Cross-module test** (`tests/test_event_chain.c`, shared by the three event modules): the clock feeds the
sequencer and the arpeggiator, whose events drive a real ADSR. The envelope must start within two control periods of the
sample the clock arithmetic predicts (step *n* at 6000*n* samples at 120 bpm, 48 kHz, division 6), reach
full level, release at the gate, be silent before the next step, and be **bit-identical** for a 256-sample
and for an irregular block split (1, 7, 97, 255, 300, 13, 512, 64). This also confirms the legato
convention against the ADSR (a `NOTE_ON` while the gate is high is ignored, so a tied step does not
retrigger the envelope).

## Limits

* The pulse period is at least 5 samples (8 kHz, 999 bpm, 96 ppqn); this is checked at compile time
  (`PCL_STATIC_ASSERT` in `pcl_clock.c`).
* A block longer than 65536 samples would overflow the 16-bit offset: `frames` is not checked.
* There is no external sync (MIDI clock input) and no swing in the clock itself (the sequencer has
  swing). A clock that follows an external source would be a different producer of the same events.
