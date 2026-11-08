# Arpeggiator

Files: `include/pcl/pcl_arp.h`, `src/event/pcl_arp.c`, `src/core/pcl_evbuf.h` (internal),
`tests/test_arp.c`, `tests/test_event_chain.c`; the `events` subcommand of `tools/render.c`.

**Status: implemented; tests pass in float, Q15 and Q31** (Clang ASan + UBSan, the `cm0plus-*` and
`cm33-*` compile checks, no float runtime pulled in). Pure integer code: **the three builds produce
identical events**. Cortex-M0+ cost is unmeasured (no hardware).

> **Written without review.** Choices marked **[D]** were made by the author while the user was away;
> they are listed for review in the morning.

## Summary

Turns the keys that are held and the clock pulses into a stream of notes. It consumes `NOTE_ON`,
`NOTE_OFF`, `CLOCK`, `START` and `STOP` events (sorted, with sample offsets) and produces `NOTE_ON` and
`NOTE_OFF` events at the offsets of the clock events that caused them.

* **Event rate**, no audio. One call per block with the block's events.
* Notes start on every `division`-th pulse and are released `gate` pulses later. Both are counted in
  **pulses** **[D]**, not samples or milliseconds, so every note time is the offset of a CLOCK event and
  the arpeggiator follows tempo changes and clock jitter for free. (A step of 6 pulses at 24 ppqn is a
  sixteenth note.)
* Up to 16 held keys (`PCL_ARP_MAX_NOTES`; further keys are ignored), 1 to 4 octaves.

## Modes

With the held notes sorted by pitch `n0 < n1 < ...` (m of them) and `octaves` = o, the pattern is the
`m*o` notes `n_i + 12*j` in ascending order (`e = j*m + i`); `pos` counts the steps played:

| Mode | Note index `e` of step `pos` | Period |
|------|-------------------------------|--------|
| `UP` | `pos mod m*o` | `m*o` |
| `DOWN` | `m*o - 1 - pos mod m*o` | `m*o` |
| `UPDOWN` | up, then down; the top and bottom notes are played once per turn | `2*m*o - 2` (1 if `m*o` = 1) |
| `UPDOWN_INCLUSIVE` | up, then down; the end notes are played twice | `2*m*o` |
| `RANDOM` | uniform over the pattern (xorshift32 from the config `seed`; `0` = the default seed) | |
| `AS_PLAYED` | the pattern in key-press order (the octaves repeat that order) | `m*o` |
| `CHORD` | all held notes together on every step; octaves do not apply **[D]** | |

Notes above 127 (a high key plus octaves) are clamped to 127 **[D]**. A note's velocity is the velocity
of the key it came from.

## Behaviour

* **Keys.** `NOTE_ON` adds a key (a key that is already held just updates its velocity).
  `NOTE_ON` with velocity 0 is a `NOTE_OFF`. `NOTE_OFF` removes it. When the last key goes, the sounding
  note is released **at the offset of that key-up** (the note does not wait for the next pulse).
* **Step.** On a pulse where `pulse mod division == 0` the previous note (if it still sounds) is
  released, then the next note starts: `NOTE_OFF` before `NOTE_ON` at the same offset, always. With
  `gate >= division` a note therefore lasts until the next step, and the voice is retriggered.
* **Gate.** `pulse mod division == gate` releases the note (for `gate < division`).
* **Pattern position.** `pos` increments per step and the note is looked up in the *current* held set,
  so keys can come and go while it plays; the pattern keeps its place and wraps by modulo **[D]**, it
  does not restart when the set changes.
* **Latch.** With `latch` on, a released key stays in the set (`down` = 0). Pressing a key while none
  of the set is physically down starts a new chord: the latched one is dropped **[D]** (the usual
  hardware behaviour). `set_latch(0)` drops the keys that are not physically down.
* **START** (from the clock): the pulse count and `pos` go back to 0 and sounding notes are released.
  **STOP** releases sounding notes and changes nothing else.
* `reset` empties the set and rewinds, but does **not** release sounding notes (no event is produced);
  send `STOP` through `process` first if a voice may be holding one **[D]**.
* Events of other types are ignored and nothing is passed through **[D]**.

## Parameters

| Parameter | Range | Notes |
|-----------|-------|-------|
| `mode` | the seven above | |
| `octaves` | 1 .. 4 | |
| `division` | 1 .. 96 pulses | |
| `gate` | 1 .. 96 pulses | `>= division`: legato |
| `latch` | 0 / 1 | |
| `seed` | any | RANDOM only |

Setters validate and leave the module unchanged on error; they take effect at the next pulse.

## Capacity

`process(arp, in, n_in, out, capacity)` returns the number of events written. A step produces at most
`2 * PCL_ARP_MAX_NOTES` events (CHORD: 16 releases and 16 starts), a key release at most 16. Events that
do not fit are **dropped and counted** (`pcl_arp_dropped`) **[D]**, the state still advances, and `out`
is never written past `capacity`. For a buffer that cannot lose events, size it at
`n_clock_events * 2 * PCL_ARP_MAX_NOTES + n_key_events * PCL_ARP_MAX_NOTES`; in practice a non-CHORD arp
needs `2 * n_clock_events + n_key_events`.

## Memory and cost

`sizeof(pcl_arp_t)` is 88 bytes (arrays of 16 for note, velocity, key state and sounding note, plus the
counters); the code is 1.3 KB (`-Os`) / 2.1 KB (`-O2`) on a Cortex-M0+ (measured with `arm-none-eabi-gcc`, not run). No tables. Per pulse: one modulo, the set sorted by insertion sort (at most 16 entries), one
pattern lookup. The loops are bounded by `PCL_ARP_MAX_NOTES`. The two integer divisions per pulse
(`pulse mod division`, `pos mod ...`) are library calls on a Cortex-M0+, a few tens of cycles each.

## Tests (`tests/test_arp.c`, 20 cases, ~1.5 million checks)

* validation of every field and setter, state untouched on error;
* **the note sequence of every mode** with keys pressed out of order, 1 and 2 octaves; sets of one and
  two notes (UPDOWN with two notes, single note in every mode, four octaves of one note);
* step and gate timing with exact offsets (division 4 / gate 1, gate 3, gate >= division);
  OFF always immediately before ON at one offset; velocity of the key a note belongs to; AS_PLAYED
  velocities across octaves;
* CHORD (order, velocities, octaves ignored); clamping at 127; more than 16 keys; the same key twice;
* key-up that empties the set releases at the key-up offset; velocity-0 NOTE_ON; unknown events;
  empty-set pulses still count;
* keys added and removed while the pattern runs (position kept, wraps by modulo); velocity and key
  state kept when a middle key is removed;
* latch: held, replaced by a new chord, joined while another key is down, a latched key pressed again,
  latch off;
* START/STOP: release offsets, restart from the first note, STOP changes nothing else;
* RANDOM: same seed gives the same sequence, a different seed a different one, all notes and octaves
  reached;
* output capacity: a tight buffer gets exactly the leading events of the full stream, the rest is
  counted, zero capacity;
* **random streams** (60 runs over every mode and several settings): output sorted, every `NOTE_OFF`
  releases a note that sounds, nothing left sounding after `STOP`, no drops, bounded counts.

Mutation testing of `pcl_arp.c` (operator and constant mutations): the 10 survivors are equivalent
mutants (boolean flags written as 2, reads of a stale vacated slot, a loop bound that reads one entry
past the set which is zero by construction).

**Cross-module test** (`tests/test_event_chain.c`, shared by the three event modules): the clock feeds the
arpeggiator (a key held and released mid-run), whose events drive a real ADSR. The envelope must start within two control periods of the
sample the clock arithmetic predicts (step *n* at 6000*n* samples at 120 bpm, 48 kHz, division 6), reach
full level, release at the gate, be silent before the next step, and be **bit-identical** for a 256-sample
and for an irregular block split (1, 7, 97, 255, 300, 13, 512, 64). This also confirms the legato
convention against the ADSR (a `NOTE_ON` while the gate is high is ignored, so a tied step does not
retrigger the envelope).

## Limits and not done

* No pass-through of the input events (see above), no note-length randomisation, no velocity
  accent pattern, no "hold" pedal (latch is the equivalent) and no chord memory.
* The held set is the first 16 keys; a 17th is ignored until one is released.
* No ratcheting or per-step probability; those would sit naturally in the step sequencer.
