/*
 * pcl_arp.c - see include/pcl/pcl_arp.h and docs/modules/arp.md.
 */
#include "pcl/pcl_arp.h"

#include "core/pcl_evbuf.h"
#include "core/pcl_rng.h"

#define ARP_OCTAVE 12u
#define ARP_NOTE_MAX 127u
#define ARP_GATE_MAX 96u

static int arp_mode_valid(pcl_arp_mode_t mode) { return (unsigned)mode <= (unsigned)PCL_ARP_CHORD; }

static int arp_octaves_valid(uint32_t o) { return o >= 1u && o <= PCL_ARP_MAX_OCTAVES; }

static int arp_division_valid(uint32_t d) { return d >= 1u && d <= PCL_ARP_MAX_DIVISION; }

static int arp_gate_valid(uint32_t g) { return g >= 1u && g <= ARP_GATE_MAX; }

static void arp_clear(pcl_arp_t *arp)
{
    unsigned i;

    /* the arrays are zeroed too, so equal histories give equal (bit-identical) states */
    for (i = 0; i < PCL_ARP_MAX_NOTES; i++)
    {
        arp->note[i] = 0;
        arp->vel[i] = 0;
        arp->down[i] = 0;
        arp->on_note[i] = 0;
    }
    arp->n = 0;
    arp->on_count = 0;
    arp->pulse = 0;
    arp->pos = 0;
}

pcl_status_t pcl_arp_init(pcl_arp_t *arp, const pcl_arp_config_t *cfg)
{
    pcl_arp_t tmp;

    if (arp == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!arp_mode_valid(cfg->mode) || !arp_octaves_valid(cfg->octaves)
        || !arp_division_valid(cfg->division) || !arp_gate_valid(cfg->gate))
        return PCL_ERR_OUT_OF_RANGE;
    arp_clear(&tmp);
    tmp.mode = (uint8_t)cfg->mode;
    tmp.octaves = (uint8_t)cfg->octaves;
    tmp.division = (uint8_t)cfg->division;
    tmp.gate = (uint8_t)cfg->gate;
    tmp.latch = cfg->latch != 0;
    tmp.rng = pcl_rng_seed(cfg->seed);
    tmp.dropped = 0;
    *arp = tmp;
    return PCL_OK;
}

void pcl_arp_reset(pcl_arp_t *arp) { arp_clear(arp); }

pcl_status_t pcl_arp_set_mode(pcl_arp_t *arp, pcl_arp_mode_t mode)
{
    if (arp == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!arp_mode_valid(mode))
        return PCL_ERR_OUT_OF_RANGE;
    arp->mode = (uint8_t)mode;
    return PCL_OK;
}

pcl_status_t pcl_arp_set_octaves(pcl_arp_t *arp, uint32_t octaves)
{
    if (arp == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!arp_octaves_valid(octaves))
        return PCL_ERR_OUT_OF_RANGE;
    arp->octaves = (uint8_t)octaves;
    return PCL_OK;
}

pcl_status_t pcl_arp_set_division(pcl_arp_t *arp, uint32_t division)
{
    if (arp == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!arp_division_valid(division))
        return PCL_ERR_OUT_OF_RANGE;
    arp->division = (uint8_t)division;
    return PCL_OK;
}

pcl_status_t pcl_arp_set_gate(pcl_arp_t *arp, uint32_t gate)
{
    if (arp == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!arp_gate_valid(gate))
        return PCL_ERR_OUT_OF_RANGE;
    arp->gate = (uint8_t)gate;
    return PCL_OK;
}

/* Removes held entry `i`, keeping the order of the others. */
static void arp_remove(pcl_arp_t *arp, unsigned i)
{
    for (; i + 1u < arp->n; i++)
    {
        arp->note[i] = arp->note[i + 1u];
        arp->vel[i] = arp->vel[i + 1u];
        arp->down[i] = arp->down[i + 1u];
    }
    arp->n--;
}

void pcl_arp_set_latch(pcl_arp_t *arp, int latch)
{
    unsigned i = 0;

    arp->latch = latch != 0;
    if (!arp->latch)
    {
        while (i < arp->n)
        {
            if (arp->down[i])
                i++;
            else
                arp_remove(arp, i);
        }
    }
}

size_t pcl_arp_held(const pcl_arp_t *arp) { return arp->n; }

uint32_t pcl_arp_dropped(const pcl_arp_t *arp) { return arp->dropped; }

/* Releases everything that sounds, at `offset`. */
static void arp_all_off(pcl_arp_t *arp, pcl_evbuf_t *b, uint16_t offset)
{
    unsigned i;

    for (i = 0; i < arp->on_count; i++)
        pcl_evbuf_emit(b, offset, PCL_EVT_NOTE_OFF, arp->on_note[i], 0);
    arp->on_count = 0;
}

static int arp_find(const pcl_arp_t *arp, uint8_t note)
{
    unsigned i;

    for (i = 0; i < arp->n; i++)
        if (arp->note[i] == note)
            return (int)i;
    return -1;
}

static int arp_any_down(const pcl_arp_t *arp)
{
    unsigned i;

    for (i = 0; i < arp->n; i++)
        if (arp->down[i])
            return 1;
    return 0;
}

static void arp_note_on(pcl_arp_t *arp, uint8_t note, uint8_t velocity)
{
    const int found = arp_find(arp, note);

    if (found >= 0)
    {
        arp->vel[found] = velocity;
        arp->down[found] = 1;
        return;
    }
    /* a new chord after every key was released while latched replaces the latched one */
    if (arp->latch && !arp_any_down(arp))
        arp->n = 0;
    if (arp->n >= PCL_ARP_MAX_NOTES)
        return;
    arp->note[arp->n] = note;
    arp->vel[arp->n] = velocity;
    arp->down[arp->n] = 1;
    arp->n++;
}

static void arp_note_off(pcl_arp_t *arp, uint8_t note, pcl_evbuf_t *b, uint16_t offset)
{
    const int found = arp_find(arp, note);

    if (found < 0)
        return;
    if (arp->latch)
        arp->down[found] = 0;
    else
        arp_remove(arp, (unsigned)found);
    if (arp->n == 0)
        arp_all_off(arp, b, offset);
}

/* The held set sorted by pitch (insertion sort; at most PCL_ARP_MAX_NOTES entries). */
static void arp_sorted(const pcl_arp_t *arp, uint8_t *note, uint8_t *vel)
{
    unsigned i, j;

    for (i = 0; i < arp->n; i++)
    {
        const uint8_t nn = arp->note[i];
        const uint8_t vv = arp->vel[i];

        for (j = i; j > 0 && note[j - 1u] > nn; j--)
        {
            note[j] = note[j - 1u];
            vel[j] = vel[j - 1u];
        }
        note[j] = nn;
        vel[j] = vv;
    }
}

static void arp_sound(pcl_arp_t *arp, pcl_evbuf_t *b, uint16_t offset, unsigned note, uint8_t vel)
{
    const uint8_t n = (uint8_t)(note > ARP_NOTE_MAX ? ARP_NOTE_MAX : note);

    pcl_evbuf_emit(b, offset, PCL_EVT_NOTE_ON, n, vel);
    if (arp->on_count >= PCL_ARP_MAX_NOTES) { 
        return;   /* or drop the oldest */
    }
    arp->on_note[arp->on_count++] = n;
}

/* Starts the step: the notes the mode picks from the held set. */
static void arp_step(pcl_arp_t *arp, pcl_evbuf_t *b, uint16_t offset)
{
    uint8_t        snote[PCL_ARP_MAX_NOTES];
    uint8_t        svel[PCL_ARP_MAX_NOTES];
    const uint32_t n = arp->n;
    const uint32_t total = n * arp->octaves; /* notes in one pass of the pattern */
    uint32_t       period;
    uint32_t       e;
    unsigned       i;

    arp_sorted(arp, snote, svel);
    if (arp->mode == PCL_ARP_CHORD)
    {
        for (i = 0; i < n; i++)
            arp_sound(arp, b, offset, snote[i], svel[i]);
        arp->pos++;
        return;
    }
    switch ((pcl_arp_mode_t)arp->mode)
    {
    case PCL_ARP_DOWN:
        period = total;
        e = total - 1u - arp->pos % period;
        break;
    case PCL_ARP_UPDOWN:
    {
        uint32_t idx;

        period = total < 2u ? 1u : 2u * total - 2u;
        idx = arp->pos % period;
        e = idx < total ? idx : 2u * total - 2u - idx;
        break;
    }
    case PCL_ARP_UPDOWN_INCLUSIVE:
    {
        uint32_t idx;

        period = 2u * total;
        idx = arp->pos % period;
        e = idx < total ? idx : 2u * total - 1u - idx;
        break;
    }
    case PCL_ARP_RANDOM:
        e = pcl_rng_next(&arp->rng) % total;
        break;
    default: /* UP and AS_PLAYED walk the pattern in order */
        e = arp->pos % total;
        break;
    }
    if (arp->mode == PCL_ARP_AS_PLAYED)
        arp_sound(arp, b, offset, arp->note[e % n] + ARP_OCTAVE * (e / n), arp->vel[e % n]);
    else
        arp_sound(arp, b, offset, snote[e % n] + ARP_OCTAVE * (e / n), svel[e % n]);
    arp->pos++;
}

static void arp_pulse(pcl_arp_t *arp, pcl_evbuf_t *b, uint16_t offset)
{
    const uint32_t phase = arp->pulse % arp->division;

    if (phase == 0u)
    {
        arp_all_off(arp, b, offset); /* the previous step's notes, if they still sound */
        if (arp->n > 0u)
            arp_step(arp, b, offset);
    }
    else if (arp->on_count > 0u && phase == arp->gate)
    {
        arp_all_off(arp, b, offset);
    }
    arp->pulse++;
}

size_t pcl_arp_process(
    pcl_arp_t *arp, const pcl_event_t *in, size_t n_in, pcl_event_t *out, size_t capacity)
{
    pcl_evbuf_t b;
    size_t      i;

    b.out = out;
    b.capacity = capacity;
    b.count = 0;
    b.dropped = &arp->dropped;
    for (i = 0; i < n_in; i++)
    {
        const pcl_event_t *e = &in[i];

        switch (e->type)
        {
        case PCL_EVT_NOTE_ON:
            if (e->velocity == 0u)
                arp_note_off(arp, e->note, &b, e->offset);
            else
                arp_note_on(arp, e->note, e->velocity);
            break;
        case PCL_EVT_NOTE_OFF:
            arp_note_off(arp, e->note, &b, e->offset);
            break;
        case PCL_EVT_CLOCK:
            arp_pulse(arp, &b, e->offset);
            break;
        case PCL_EVT_START:
            arp_all_off(arp, &b, e->offset);
            arp->pulse = 0;
            arp->pos = 0;
            break;
        case PCL_EVT_STOP:
            arp_all_off(arp, &b, e->offset);
            break;
        default:
            break;
        }
    }
    return b.count;
}
