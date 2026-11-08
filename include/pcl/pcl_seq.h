/*
 * pcl_seq.h - step sequencer: a pattern of notes played from clock pulses.
 *
 * Design note: docs/modules/sequencer.md.
 *
 * Input events (sorted by offset, as in pcl_event.h):
 *   CLOCK - one pulse; a step starts every `division` pulses (every other step is delayed by
 *           `swing` pulses), and its note is released `gate` pulses after it starts;
 *   START - back to the first step (the last one when playing backward), pulse count 0; a
 *           sounding note is released;
 *   STOP  - a sounding note is released.
 * Note events are ignored (a transposing "follow" input is not part of this module), as are other
 * types; nothing is passed through.
 *
 * A step is a note and a velocity; velocity 0 is a rest. A step with PCL_SEQ_TIE holds its note
 * through the end of the step into the next one: no NOTE_OFF at the end of the step, and if the
 * next step has a note, its NOTE_ON arrives while the tied one still sounds (a legato, which a
 * voice with an envelope reads as "change pitch, do not retrigger"); if the next step is a rest,
 * the tied note ends there. A NOTE_OFF always comes before the NOTE_ON that follows it at the same
 * offset.
 *
 * Rate: event rate. Memory: the caller's pcl_seq_t (about 300 bytes), no heap.
 */
#ifndef PCL_SEQ_H
#define PCL_SEQ_H

#include "pcl/pcl_event.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

#define PCL_SEQ_MAX_STEPS 64
#define PCL_SEQ_MAX_DIVISION 96

/* pcl_seq_step_t.flags */
#define PCL_SEQ_TIE 0x01u

typedef struct
{
    uint8_t note;     /* MIDI note, 0 .. 127                                                     */
    uint8_t velocity; /* 1 .. 127 plays the note; 0 is a rest                                    */
    uint8_t gate;     /* pulses the note lasts; 0 means the sequence's default gate              */
    uint8_t flags;    /* PCL_SEQ_TIE                                                             */
} pcl_seq_step_t;

typedef enum
{
    PCL_SEQ_FORWARD,
    PCL_SEQ_BACKWARD,
    PCL_SEQ_PINGPONG, /* forward then backward, the end steps played once per turn               */
    PCL_SEQ_RANDOM    /* any step, uniformly                                                     */
} pcl_seq_direction_t;

typedef struct
{
    uint32_t            length;   /* steps in the loop, 1 .. PCL_SEQ_MAX_STEPS                   */
    uint32_t            division; /* pulses per step, 1 .. 96 (6 at 24 ppqn is a sixteenth)     */
    uint32_t            gate;     /* default gate in pulses, 1 .. 96                             */
    uint32_t            swing;    /* pulses by which every second step is late, 0 .. division-1 */
    pcl_seq_direction_t direction;
    int transpose; /* semitones added to every note; the result is clamped to 0 .. 127, and
                      the setting itself to -127 .. 127                            */
    uint32_t seed; /* RANDOM direction; 0 means PCL_NOISE_DEFAULT_SEED            */
} pcl_seq_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_seq_step_t step[PCL_SEQ_MAX_STEPS];
    uint32_t       pulse; /* clock pulses since START (or init/reset)                          */
    uint32_t next_base;   /* the pulse at which the next step is due, before its swing         */
    uint32_t fired;       /* steps started so far; its parity says which are swung             */
    uint32_t off_at;      /* the pulse at which the sounding note is released                  */
    uint32_t rng;
    uint32_t dropped;
    int32_t  transpose;
    uint8_t  length;
    uint8_t  division;
    uint8_t  gate;
    uint8_t  swing;
    uint8_t  direction;
    uint8_t  pos;   /* the step played last                                              */
    int8_t   way;   /* +1 or -1 (ping-pong)                                              */
    uint8_t  first; /* the next step is the first of the run                             */
    uint8_t  sounding;
    uint8_t  tied; /* the sounding note is held into the next step                      */
    uint8_t  cur_note;
} pcl_seq_t;

/*
 * Initializes `seq` from `cfg` with every step a rest (note 60). Returns PCL_ERR_NULL_POINTER for
 * NULL arguments and PCL_ERR_OUT_OF_RANGE for a field outside its range; `seq` is untouched on
 * error. Control-thread call.
 */
pcl_status_t pcl_seq_init(pcl_seq_t *seq, const pcl_seq_config_t *cfg);

/*
 * Goes back to the first step. The pattern and settings are kept. A sounding note is NOT released
 * (no event is produced): send STOP through pcl_seq_process first if a voice may be holding one.
 */
void pcl_seq_reset(pcl_seq_t *seq);

/* Step access. PCL_ERR_OUT_OF_RANGE for an index at or above PCL_SEQ_MAX_STEPS or a note above 127.
 */
pcl_status_t pcl_seq_set_step(pcl_seq_t *seq, size_t index, const pcl_seq_step_t *step);
pcl_status_t pcl_seq_get_step(const pcl_seq_t *seq, size_t index, pcl_seq_step_t *step);

/*
 * Settings: the same ranges and errors as the config; unchanged on error. Control-thread calls. A
 * change takes effect from the next step. Lowering the division also lowers the swing to
 * division-1.
 */
pcl_status_t pcl_seq_set_length(pcl_seq_t *seq, uint32_t length);
pcl_status_t pcl_seq_set_division(pcl_seq_t *seq, uint32_t division);
pcl_status_t pcl_seq_set_gate(pcl_seq_t *seq, uint32_t gate);
pcl_status_t pcl_seq_set_swing(pcl_seq_t *seq, uint32_t swing);
pcl_status_t pcl_seq_set_direction(pcl_seq_t *seq, pcl_seq_direction_t direction);
void         pcl_seq_set_transpose(pcl_seq_t *seq, int semitones);

/*
 * Consumes `n_in` input events (sorted; `in` may be NULL if 0) and writes the notes they cause to
 * `out` (room for `capacity`), returning how many. `in` and `out` must not overlap. Events that do
 * not fit are dropped and counted (pcl_seq_dropped); a pulse produces at most 2 events and a STOP
 * or START at most 1. Does not allocate or block, and does not check its arguments.
 */
size_t pcl_seq_process(
    pcl_seq_t *seq, const pcl_event_t *in, size_t n_in, pcl_event_t *out, size_t capacity);

/* The index of the step played last (0 before the first step of a run). */
size_t pcl_seq_position(const pcl_seq_t *seq);

/* Events dropped so far for lack of room in the caller's buffer. */
uint32_t pcl_seq_dropped(const pcl_seq_t *seq);

PCL_END_DECLS

#endif /* PCL_SEQ_H */
