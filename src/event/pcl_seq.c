/*
 * pcl_seq.c - see include/pcl/pcl_seq.h and docs/modules/sequencer.md.
 */
#include "pcl/pcl_seq.h"

#include "core/pcl_evbuf.h"
#include "core/pcl_rng.h"

#define SEQ_NOTE_MAX 127
#define SEQ_GATE_MAX 96u

static int seq_direction_valid(pcl_seq_direction_t d)
{
    return (unsigned)d <= (unsigned)PCL_SEQ_RANDOM;
}

/* Beyond the MIDI range every note clamps anyway; limiting the setting keeps note + transpose in
 * range. */
static int32_t seq_transpose_clamp(int semitones)
{
    return semitones < -SEQ_NOTE_MAX ? -SEQ_NOTE_MAX
                                     : (semitones > SEQ_NOTE_MAX ? SEQ_NOTE_MAX : semitones);
}

static int seq_length_valid(uint32_t l) { return l >= 1u && l <= PCL_SEQ_MAX_STEPS; }

static int seq_division_valid(uint32_t d) { return d >= 1u && d <= PCL_SEQ_MAX_DIVISION; }

static int seq_gate_valid(uint32_t g) { return g >= 1u && g <= SEQ_GATE_MAX; }

static int seq_step_valid(const pcl_seq_step_t *s)
{
    return s->note <= SEQ_NOTE_MAX && s->velocity <= 127u && s->gate <= SEQ_GATE_MAX
        && (s->flags & ~PCL_SEQ_TIE) == 0u;
}

/* The run goes back to its start; the pattern and the settings stay. */
static void seq_rewind(pcl_seq_t *seq)
{
    seq->pulse = 0;
    seq->next_base = 0;
    seq->fired = 0;
    seq->off_at = 0;
    seq->pos = 0;
    seq->way = 1;
    seq->first = 1;
    seq->sounding = 0;
    seq->tied = 0;
}

pcl_status_t pcl_seq_init(pcl_seq_t *seq, const pcl_seq_config_t *cfg)
{
    pcl_seq_t tmp;
    unsigned  i;

    if (seq == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!seq_length_valid(cfg->length) || !seq_division_valid(cfg->division)
        || !seq_gate_valid(cfg->gate) || cfg->swing >= cfg->division
        || !seq_direction_valid(cfg->direction))
        return PCL_ERR_OUT_OF_RANGE;
    for (i = 0; i < PCL_SEQ_MAX_STEPS; i++)
    {
        tmp.step[i].note = 60;
        tmp.step[i].velocity = 0;
        tmp.step[i].gate = 0;
        tmp.step[i].flags = 0;
    }
    tmp.length = (uint8_t)cfg->length;
    tmp.division = (uint8_t)cfg->division;
    tmp.gate = (uint8_t)cfg->gate;
    tmp.swing = (uint8_t)cfg->swing;
    tmp.direction = (uint8_t)cfg->direction;
    tmp.transpose = seq_transpose_clamp(cfg->transpose);
    tmp.rng = pcl_rng_seed(cfg->seed);
    tmp.dropped = 0;
    tmp.cur_note = 0;
    seq_rewind(&tmp);
    *seq = tmp;
    return PCL_OK;
}

void pcl_seq_reset(pcl_seq_t *seq) { seq_rewind(seq); }

pcl_status_t pcl_seq_set_step(pcl_seq_t *seq, size_t index, const pcl_seq_step_t *step)
{
    if (seq == NULL || step == NULL)
        return PCL_ERR_NULL_POINTER;
    if (index >= PCL_SEQ_MAX_STEPS || !seq_step_valid(step))
        return PCL_ERR_OUT_OF_RANGE;
    seq->step[index] = *step;
    return PCL_OK;
}

pcl_status_t pcl_seq_get_step(const pcl_seq_t *seq, size_t index, pcl_seq_step_t *step)
{
    if (seq == NULL || step == NULL)
        return PCL_ERR_NULL_POINTER;
    if (index >= PCL_SEQ_MAX_STEPS)
        return PCL_ERR_OUT_OF_RANGE;
    *step = seq->step[index];
    return PCL_OK;
}

pcl_status_t pcl_seq_set_length(pcl_seq_t *seq, uint32_t length)
{
    if (seq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!seq_length_valid(length))
        return PCL_ERR_OUT_OF_RANGE;
    seq->length = (uint8_t)length;
    return PCL_OK;
}

pcl_status_t pcl_seq_set_division(pcl_seq_t *seq, uint32_t division)
{
    if (seq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!seq_division_valid(division))
        return PCL_ERR_OUT_OF_RANGE;
    seq->division = (uint8_t)division;
    if (seq->swing >= seq->division)
        seq->swing = (uint8_t)(seq->division - 1u);
    return PCL_OK;
}

pcl_status_t pcl_seq_set_gate(pcl_seq_t *seq, uint32_t gate)
{
    if (seq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!seq_gate_valid(gate))
        return PCL_ERR_OUT_OF_RANGE;
    seq->gate = (uint8_t)gate;
    return PCL_OK;
}

pcl_status_t pcl_seq_set_swing(pcl_seq_t *seq, uint32_t swing)
{
    if (seq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (swing >= seq->division)
        return PCL_ERR_OUT_OF_RANGE;
    seq->swing = (uint8_t)swing;
    return PCL_OK;
}

pcl_status_t pcl_seq_set_direction(pcl_seq_t *seq, pcl_seq_direction_t direction)
{
    if (seq == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!seq_direction_valid(direction))
        return PCL_ERR_OUT_OF_RANGE;
    seq->direction = (uint8_t)direction;
    return PCL_OK;
}

void pcl_seq_set_transpose(pcl_seq_t *seq, int semitones)
{
    seq->transpose = seq_transpose_clamp(semitones);
}

size_t pcl_seq_position(const pcl_seq_t *seq) { return seq->pos; }

uint32_t pcl_seq_dropped(const pcl_seq_t *seq) { return seq->dropped; }

/* Releases the sounding note, if any. */
static void seq_off(pcl_seq_t *seq, pcl_evbuf_t *b, uint16_t offset)
{
    if (seq->sounding)
        pcl_evbuf_emit(b, offset, PCL_EVT_NOTE_OFF, seq->cur_note, 0);
    seq->sounding = 0;
    seq->tied = 0;
}

/* Moves to the next step of the pattern. */
static void seq_advance(pcl_seq_t *seq)
{
    const unsigned len = seq->length;

    if (seq->first)
    {
        seq->first = 0;
        seq->way = 1;
        if (seq->direction == PCL_SEQ_BACKWARD)
            seq->pos = (uint8_t)(len - 1u);
        else if (seq->direction == PCL_SEQ_RANDOM)
            seq->pos = (uint8_t)(pcl_rng_next(&seq->rng) % len);
        else
            seq->pos = 0;
        return;
    }
    if (seq->pos >= len) /* the pattern was shortened under us: the next step is the loop's start */
        seq->pos = (uint8_t)(seq->direction == PCL_SEQ_BACKWARD ? 0u : len - 1u);
    switch ((pcl_seq_direction_t)seq->direction)
    {
    case PCL_SEQ_BACKWARD:
        seq->pos = (uint8_t)(seq->pos == 0u ? len - 1u : seq->pos - 1u);
        break;
    case PCL_SEQ_PINGPONG:
        if (len == 1u)
        {
            seq->pos = 0;
        }
        else
        {
            int next = (int)seq->pos + seq->way;

            if (next >= (int)len)
            {
                next = (int)len - 2;
                seq->way = -1;
            }
            else if (next < 0)
            {
                next = 1;
                seq->way = 1;
            }
            seq->pos = (uint8_t)next;
        }
        break;
    case PCL_SEQ_RANDOM:
        seq->pos = (uint8_t)(pcl_rng_next(&seq->rng) % len);
        break;
    default: /* PCL_SEQ_FORWARD */
        seq->pos = (uint8_t)((seq->pos + 1u) % len);
        break;
    }
}

static void seq_fire(pcl_seq_t *seq, pcl_evbuf_t *b, uint16_t offset)
{
    const pcl_seq_step_t *s;

    seq_advance(seq);
    s = &seq->step[seq->pos];
    seq->next_base += seq->division;
    seq->fired++;
    if (s->velocity == 0u)
    {
        seq_off(seq, b, offset); /* a rest ends a tied or overlong note */
        return;
    }
    {
        int note = (int)s->note + seq->transpose;

        note = note < 0 ? 0 : (note > SEQ_NOTE_MAX ? SEQ_NOTE_MAX : note);
        if (seq->sounding && !seq->tied)
            seq_off(seq, b, offset); /* retrigger */
        pcl_evbuf_emit(b, offset, PCL_EVT_NOTE_ON, (uint8_t)note, s->velocity);
        seq->sounding = 1;
        seq->cur_note = (uint8_t)note;
        seq->tied = (s->flags & PCL_SEQ_TIE) != 0u;
        seq->off_at = seq->pulse + (s->gate != 0u ? s->gate : seq->gate);
    }
}

static void seq_pulse(pcl_seq_t *seq, pcl_evbuf_t *b, uint16_t offset)
{
    /* differences of pulse numbers are compared as signed, so the counters may wrap */
    const uint32_t due = seq->next_base + (((seq->fired & 1u) != 0u) ? seq->swing : 0u);

    if (seq->sounding && !seq->tied && (int32_t)(seq->pulse - seq->off_at) >= 0)
        seq_off(seq, b, offset);
    if ((int32_t)(seq->pulse - due) >= 0)
        seq_fire(seq, b, offset);
    seq->pulse++;
}

size_t pcl_seq_process(
    pcl_seq_t *seq, const pcl_event_t *in, size_t n_in, pcl_event_t *out, size_t capacity)
{
    pcl_evbuf_t b;
    size_t      i;

    b.out = out;
    b.capacity = capacity;
    b.count = 0;
    b.dropped = &seq->dropped;
    for (i = 0; i < n_in; i++)
    {
        const pcl_event_t *e = &in[i];

        switch (e->type)
        {
        case PCL_EVT_CLOCK:
            seq_pulse(seq, &b, e->offset);
            break;
        case PCL_EVT_START:
            seq_off(seq, &b, e->offset);
            seq_rewind(seq);
            break;
        case PCL_EVT_STOP:
            seq_off(seq, &b, e->offset);
            break;
        default:
            break;
        }
    }
    return b.count;
}
