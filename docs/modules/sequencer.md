# Step sequencer

Files: `include/pcl/pcl_seq.h`, `src/event/pcl_seq.c`, `src/core/pcl_evbuf.h` (internal),
`tests/test_seq.c`, `tests/test_event_chain.c`; the `events` subcommand of `tools/render.c`.

**Status: implemented; tests pass in float, Q15 and Q31** (Clang ASan + UBSan, the `cm0plus-*` and
`cm33-*` compile checks, no float runtime pulled in). Pure integer code: **the three builds produce
identical events**. Cortex-M0+ cost is unmeasured (no hardware).

> **Written without review.** Choices marked **[D]** were made by the author while the user was away;
> they are listed for review in the morning.

## Summary

A pattern of up to 64 steps played from clock pulses. It consumes `CLOCK`, `START` and `STOP` events
and produces `NOTE_ON` / `NOTE_OFF` events at the offsets of the pulses that caused them.

* **Event rate**, no audio. Everything is counted in **pulses** **[D]** (as in the arpeggiator): the
  step length (`division`), the gate and the swing. A step of 6 pulses at 24 ppqn is a sixteenth note.
* Per step: `note` (0..127), `velocity` (1..127; **0 is a rest**), an optional `gate` (0 = the
  sequence's default) and a `TIE` flag.
* Note events and other event types are ignored; nothing is passed through **[D]**. (A transposing
  "follow the keyboard" input is not part of this module; `set_transpose` is the control-side
  equivalent.)

## Parameters

| Parameter | Range | Notes |
|-----------|-------|-------|
| `length` | 1 .. 64 | steps in the loop |
| `division` | 1 .. 96 pulses | pulses per step |
| `gate` | 1 .. 96 pulses | default note length; a step's own `gate` overrides it |
| `swing` | 0 .. `division` - 1 | pulses by which every second step is late |
| `direction` | `FORWARD`, `BACKWARD`, `PINGPONG`, `RANDOM` | |
| `transpose` | semitones | added to every note; the result is clamped to 0..127, and the setting itself to ±127 |
| `seed` | any | RANDOM only (xorshift32; `0` = the default seed) |

All steps start as rests (note 60). Setters validate and leave the module unchanged on error. A change
takes effect from the next step; lowering the division lowers the swing to `division - 1`.

## Behaviour

* **Pulse 0 of a run plays the first step** (the last when playing backward; a random one in RANDOM).
  A run starts at init, `reset`, and `START`.
* **Step times.** Step `n` is due at pulse `n * division`, plus `swing` when `n` is odd. (So with swing,
  the late step is `swing` pulses after the grid and the next one is on the grid again; the average
  tempo does not change.)
* **Directions.** `BACKWARD`: last to first. `PINGPONG`: forward then backward, the end steps played
  once per turn (`0 1 2 3 2 1 0 1 ...`). `RANDOM`: uniform over the loop. A loop of one step plays that
  step every time in every direction.
* **Rests** start nothing. A note that is still sounding when a rest comes is released there.
* **Gates.** A note is released `gate` pulses after it starts, or at the next step if that is sooner;
  with `gate >= division` it is cut by the next step: `NOTE_OFF`, then `NOTE_ON`, at the same offset
  (the voice retriggers).
* **Ties [D].** A step with `PCL_SEQ_TIE` keeps its note through the end of the step: no `NOTE_OFF`,
  and if the next step has a note, its `NOTE_ON` comes **while the tied one still sounds**. That is a
  *legato*: a monophonic voice with an envelope reads it as "change pitch, do not retrigger" (the
  ADSR ignores note numbers). The tied note never receives its own `NOTE_OFF` in that case; the
  following note's release ends the sound. If the next step is a rest, the tied note ends there. A tie
  on the last step wraps into the first. **A polyphonic consumer must treat a `NOTE_ON` that arrives
  while the previous one sounds as a replacement** (this is how the property test models the voice).
* **Transpose** is applied when a note starts; the matching `NOTE_OFF` uses the note that was started,
  so changing the transpose while a note sounds cannot leave it hanging.
* **START** releases a sounding note and rewinds; **STOP** releases a sounding note and leaves the
  position; `reset` rewinds without events (send `STOP` first if a voice may hold a note) **[D]**.
* **Shortening the loop under the playhead**: the next step is the start of the (new) loop.
* **Counters** (pulses, step count) are 32-bit and compared as signed differences, so a run that crosses
  2^32 pulses (at 24 ppqn and 120 bpm, 2.8 years) continues correctly (`test_pulse_counter_wrap`).

## Capacity

A pulse produces at most two events (release and start), a `START`/`STOP` at most one. A buffer of
`2 * n_clock_events + n_transport_events` can never overflow; events that do not fit are **dropped and
counted** (`pcl_seq_dropped`) **[D]**, the state still advances, and `out` is never written past
`capacity`.

## Memory and cost

`sizeof(pcl_seq_t)` is 296 bytes (64 steps of 4 bytes plus counters). Code: 1.2 KB (`-Os`) / 1.6 KB
(`-O2`) on a Cortex-M0+ (measured with `arm-none-eabi-gcc`, not run). No tables. Per pulse: a few
compares; a step start adds one modulo (forward) and the pattern lookup.

## Tests (`tests/test_seq.c`, 17 cases, ~150,000 checks)

* validation of every field and setter; default steps; the limits accepted, one beyond rejected;
* forward timing with exact pulses and velocities, shorter loop than steps set, `gate > division` cut,
  per-step gate, rests (with the gate release still due), overlong gate ending in a rest;
* every direction's order, loops of 1, 2, 3 and 4 in ping-pong, RANDOM determinism / seeds / coverage and
  the first step of a random run;
* ties: through a step, into a rest, wrapping from the last step into the first;
* swing: exact step pulses, maximum swing, swing following a lowered division;
* transpose: positive, clamped at both ends, extreme settings (no signed overflow), release uses the
  started note;
* transport: STOP, START, START while sounding, backward restart, `reset`, ignored events;
* settings changed while running: shortening the loop under the playhead (forward at and past the new
  end, backward, ping-pong), division change takes effect from the next step;
* **a 32-bit counter wrap**: a run whose counters start 16 pulses below 2^32 gives the same events as a run
  from 0;
* output capacity (partial write is the leading part of the full stream, counted drops, zero capacity);
* **an independent reference model** (300 random patterns with rests, gates, swing, forward/backward):
  the event list is compared with one computed from the closed form `start(n) = n * division (+ swing)`,
  `release = min(start + gate, start(n+1))`;
* **random streams** (60 runs: random 64-step patterns with ties, rests and gates, random START/STOP,
  length, division, swing and transpose changes): sorted output, every `NOTE_OFF` releases the note a
  monophonic voice model says is sounding, nothing left sounding after `STOP`, no drops.

Mutation testing of `pcl_seq.c` (164 operator and constant mutants): 149 killed; the 15 survivors are
equivalent - clamp comparisons whose boundary value maps to itself, the redundant resets of `way`,
`first`, `tied` and `off_at` in `seq_rewind`/`seq_off` (each is overwritten before it is read),
`sounding` written as 2, `fired++` against `fired--` (only its parity is used), a loop bound that
writes one entry past the array of a local, and the initialisation of `cur_note` (there for
determinism of the state, invisible to the output).

**Cross-module test** (`tests/test_event_chain.c`, shared by the three event modules): the clock feeds the
sequencer, whose events drive a real ADSR. The envelope must start within two control periods of the
sample the clock arithmetic predicts (step *n* at 6000*n* samples at 120 bpm, 48 kHz, division 6), reach
full level, release at the gate, be silent before the next step, and be **bit-identical** for a 256-sample
and for an irregular block split (1, 7, 97, 255, 300, 13, 512, 64). This also confirms the legato
convention against the ADSR (a `NOTE_ON` while the gate is high is ignored, so a tied step does not
retrigger the envelope).

## Limits and not done

* No per-step probability, ratchets / repeats, accent, slide time or step-level transpose: those are
  natural additions to the step struct (`flags`) without changing the process loop.
* No song mode (pattern chaining), no recording, no MIDI file import (that is `platform/` or
  `examples/` territory, not the core).
* RANDOM can repeat a step; there is no "no repeat" or "shuffle" mode.
