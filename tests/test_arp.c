/*
 * pcl_arp tests: the note sequence of every mode, step and gate timing, latch, transport,
 * capacity limits, and a random-stream property test (sorted, no stuck notes, bounded).
 */
#include <string.h>

#include "pcl_test.h"

#define MAXEV 1024

typedef struct
{
    pcl_event_t e[MAXEV];
    size_t      n;
} evl_t;

static void put(evl_t *l, uint16_t offset, uint8_t type, uint8_t note, uint8_t vel)
{
    PCL_CHECK(l->n < MAXEV);
    l->e[l->n].offset = offset;
    l->e[l->n].type = type;
    l->e[l->n].note = note;
    l->e[l->n].velocity = vel;
    l->n++;
}

static pcl_arp_config_t cfg_of(
    pcl_arp_mode_t mode, uint32_t octaves, uint32_t division, uint32_t gate)
{
    pcl_arp_config_t c;

    c.mode = mode;
    c.octaves = octaves;
    c.division = division;
    c.gate = gate;
    c.latch = 0;
    c.seed = 0;
    return c;
}

static void arp_new(
    pcl_arp_t *a, pcl_arp_mode_t mode, uint32_t octaves, uint32_t division, uint32_t gate)
{
    const pcl_arp_config_t c = cfg_of(mode, octaves, division, gate);

    PCL_CHECK_EQ_INT(pcl_arp_init(a, &c), PCL_OK);
}

/* Presses the notes at offset 0 then sends `pulses` clock events 10 samples apart, starting at 10.
 */
static size_t play(pcl_arp_t *a, const uint8_t *keys, size_t nkeys, unsigned pulses, evl_t *out)
{
    evl_t    in;
    size_t   i;
    unsigned p;

    in.n = 0;
    for (i = 0; i < nkeys; i++)
        put(&in, 0, PCL_EVT_NOTE_ON, keys[i], (uint8_t)(100 + i));
    for (p = 0; p < pulses; p++)
        put(&in, (uint16_t)(10 + 10 * p), PCL_EVT_CLOCK, 0, 0);
    out->n = pcl_arp_process(a, in.e, in.n, out->e, MAXEV);
    return out->n;
}

/* The notes of the NOTE_ON events, in order. */
static size_t ons(const evl_t *l, uint8_t *notes, size_t max)
{
    size_t i, n = 0;

    for (i = 0; i < l->n && n < max; i++)
        if (l->e[i].type == PCL_EVT_NOTE_ON)
            notes[n++] = l->e[i].note;
    return n;
}

static void check_seq(pcl_arp_mode_t mode, uint32_t octaves, const uint8_t *keys, size_t nkeys,
    const uint8_t *want, size_t nwant)
{
    pcl_arp_t a;
    evl_t     out;
    uint8_t   got[64];
    size_t    n;

    arp_new(&a, mode, octaves, 2, 1);
    (void)play(&a, keys, nkeys, (unsigned)(2 * nwant), &out);
    n = ons(&out, got, 64);
    PCL_CHECK_EQ_INT(n, nwant);
    if (n == nwant)
        PCL_CHECK(memcmp(got, want, nwant) == 0);
}

static void test_init_validation(void)
{
    pcl_arp_t        a;
    pcl_arp_config_t c = cfg_of(PCL_ARP_UP, 1, 6, 3);

    PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_arp_init(NULL, &c), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_arp_init(&a, NULL), PCL_ERR_NULL_POINTER);
    {
        pcl_arp_t keep = a;

        c = cfg_of((pcl_arp_mode_t)7, 1, 6, 3);
        PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(PCL_ARP_UP, 0, 6, 3);
        PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(PCL_ARP_UP, 5, 6, 3);
        PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(PCL_ARP_UP, 1, 0, 3);
        PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(PCL_ARP_UP, 1, 97, 3);
        PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(PCL_ARP_UP, 1, 6, 0);
        PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(PCL_ARP_UP, 1, 6, 97);
        PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_ERR_OUT_OF_RANGE);
        PCL_CHECK(memcmp(&a, &keep, sizeof a) == 0);
    }
    c = cfg_of(PCL_ARP_CHORD, 4, 96, 96);
    PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_OK);

    PCL_CHECK_EQ_INT(pcl_arp_set_mode(&a, PCL_ARP_DOWN), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_arp_set_mode(&a, (pcl_arp_mode_t)9), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_arp_set_mode(NULL, PCL_ARP_UP), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_arp_set_octaves(&a, 2), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_arp_set_octaves(&a, 0), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_arp_set_octaves(&a, 5), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_arp_set_division(&a, 12), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_arp_set_division(&a, 0), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_arp_set_division(&a, 97), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_arp_set_gate(&a, 5), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_arp_set_gate(&a, 0), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_arp_set_gate(&a, 97), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_arp_set_gate(NULL, 5), PCL_ERR_NULL_POINTER);
}

static void test_mode_sequences(void)
{
    /* keys pressed out of order: 67, 60, 64 */
    static const uint8_t keys[] = { 67, 60, 64 };
    static const uint8_t up[] = { 60, 64, 67, 60, 64, 67, 60 };
    static const uint8_t down[] = { 67, 64, 60, 67, 64, 60, 67 };
    static const uint8_t updown[] = { 60, 64, 67, 64, 60, 64, 67, 64, 60 };
    static const uint8_t incl[] = { 60, 64, 67, 67, 64, 60, 60, 64, 67, 67 };
    static const uint8_t played[] = { 67, 60, 64, 67, 60 };
    static const uint8_t up2[] = { 60, 64, 67, 72, 76, 79, 60, 64 };
    static const uint8_t down2[] = { 79, 76, 72, 67, 64, 60, 79 };
    static const uint8_t updown2[] = { 60, 64, 67, 72, 76, 79, 76, 72, 67, 64, 60, 64 };
    static const uint8_t played2[] = { 67, 60, 64, 79, 72, 76, 67 };

    check_seq(PCL_ARP_UP, 1, keys, 3, up, sizeof up);
    check_seq(PCL_ARP_DOWN, 1, keys, 3, down, sizeof down);
    check_seq(PCL_ARP_UPDOWN, 1, keys, 3, updown, sizeof updown);
    check_seq(PCL_ARP_UPDOWN_INCLUSIVE, 1, keys, 3, incl, sizeof incl);
    check_seq(PCL_ARP_AS_PLAYED, 1, keys, 3, played, sizeof played);
    check_seq(PCL_ARP_UP, 2, keys, 3, up2, sizeof up2);
    check_seq(PCL_ARP_DOWN, 2, keys, 3, down2, sizeof down2);
    check_seq(PCL_ARP_UPDOWN, 2, keys, 3, updown2, sizeof updown2);
    check_seq(PCL_ARP_AS_PLAYED, 2, keys, 3, played2, sizeof played2);
}

static void test_small_sets(void)
{
    static const uint8_t one[] = { 60 };
    static const uint8_t two[] = { 64, 60 };
    static const uint8_t same[] = { 60, 60, 60, 60, 60 };
    static const uint8_t ud2[] = { 60, 64, 60, 64, 60 };
    static const uint8_t udi2[] = { 60, 64, 64, 60, 60, 64 };
    static const uint8_t oct1[] = { 60, 72, 84, 96, 60, 72 };
    static const uint8_t udoct1[] = { 60, 72, 84, 96, 84, 72, 60, 72 };
    pcl_arp_mode_t       m;

    for (m = PCL_ARP_UP; m <= PCL_ARP_AS_PLAYED; m = (pcl_arp_mode_t)(m + 1))
        if (m != PCL_ARP_RANDOM)
            check_seq(m, 1, one, 1, same, 5); /* one key: that key, every step */
    check_seq(PCL_ARP_UPDOWN, 1, two, 2, ud2, sizeof ud2);
    check_seq(PCL_ARP_UPDOWN_INCLUSIVE, 1, two, 2, udi2, sizeof udi2);
    check_seq(PCL_ARP_UP, 4, one, 1, oct1, sizeof oct1);
    check_seq(PCL_ARP_UPDOWN, 4, one, 1, udoct1, sizeof udoct1);
}

static void test_timing(void)
{
    static const uint8_t keys[] = { 60, 64 };
    pcl_arp_t            a;
    evl_t                out;
    size_t               i;

    /* division 4, gate 1: the first pulse (offset 10) starts a note, the second (offset 20)
       releases it; the pulses 3 and 4 are silent; the fifth (offset 50) starts the next one */
    arp_new(&a, PCL_ARP_UP, 1, 4, 1);
    (void)play(&a, keys, 2, 10, &out);
    PCL_CHECK_EQ_INT(out.n, 6);
    PCL_CHECK(out.e[0].type == PCL_EVT_NOTE_ON && out.e[0].offset == 10 && out.e[0].note == 60);
    PCL_CHECK(out.e[1].type == PCL_EVT_NOTE_OFF && out.e[1].offset == 20 && out.e[1].note == 60);
    PCL_CHECK(out.e[2].type == PCL_EVT_NOTE_ON && out.e[2].offset == 50 && out.e[2].note == 64);
    PCL_CHECK(out.e[3].type == PCL_EVT_NOTE_OFF && out.e[3].offset == 60 && out.e[3].note == 64);
    PCL_CHECK(out.e[4].type == PCL_EVT_NOTE_ON && out.e[4].offset == 90 && out.e[4].note == 60);
    PCL_CHECK(out.e[5].type == PCL_EVT_NOTE_OFF && out.e[5].offset == 100 && out.e[5].note == 60);

    /* gate 3 of division 4: released at the fourth pulse of the step */
    arp_new(&a, PCL_ARP_UP, 1, 4, 3);
    (void)play(&a, keys, 2, 5, &out);
    PCL_CHECK_EQ_INT(out.n, 3);
    PCL_CHECK(out.e[0].type == PCL_EVT_NOTE_ON && out.e[0].offset == 10);
    PCL_CHECK(out.e[1].type == PCL_EVT_NOTE_OFF && out.e[1].offset == 40);
    PCL_CHECK(out.e[2].type == PCL_EVT_NOTE_ON && out.e[2].offset == 50);

    /* gate >= division: legato in the arp sense - the old note is released at the same offset
       as the new one starts, OFF first */
    arp_new(&a, PCL_ARP_UP, 1, 2, 2);
    (void)play(&a, keys, 2, 6, &out);
    PCL_CHECK_EQ_INT(out.n, 5);
    for (i = 0; i < out.n; i++)
    {
        if (out.e[i].type == PCL_EVT_NOTE_OFF)
            PCL_CHECK(i + 1 < out.n && out.e[i + 1].type == PCL_EVT_NOTE_ON
                && out.e[i + 1].offset == out.e[i].offset);
        if (i > 0)
            PCL_CHECK(out.e[i].offset >= out.e[i - 1].offset);
    }

    /* velocities come from the key the note belongs to */
    arp_new(&a, PCL_ARP_UP, 2, 2, 1);
    (void)play(&a, keys, 2, 8, &out);
    {
        uint8_t j = 0;

        for (i = 0; i < out.n; i++)
            if (out.e[i].type == PCL_EVT_NOTE_ON)
            {
                /* UP over 2 octaves of {60 (vel 100), 64 (vel 101)}: 60 64 72 76 */
                PCL_CHECK_EQ_INT(out.e[i].velocity, 100 + (j % 2));
                j++;
            }
    }
}

static void test_division_one(void)
{
    static const uint8_t keys[] = { 60, 64 };
    pcl_arp_t            a;
    evl_t                out;

    /* every pulse is a step; gate is moot, the previous note is released as the next starts */
    arp_new(&a, PCL_ARP_UP, 1, 1, 1);
    (void)play(&a, keys, 2, 3, &out);
    PCL_CHECK_EQ_INT(out.n, 5); /* ON, OFF ON, OFF ON */
    PCL_CHECK(out.e[0].type == PCL_EVT_NOTE_ON && out.e[0].note == 60);
    PCL_CHECK(out.e[1].type == PCL_EVT_NOTE_OFF && out.e[1].note == 60 && out.e[1].offset == 20);
    PCL_CHECK(out.e[2].type == PCL_EVT_NOTE_ON && out.e[2].note == 64 && out.e[2].offset == 20);
}

static void test_chord(void)
{
    static const uint8_t keys[] = { 67, 60, 64 };
    pcl_arp_t            a;
    evl_t                out;

    arp_new(&a, PCL_ARP_CHORD, 3, 2, 1); /* octaves do not apply */
    (void)play(&a, keys, 3, 4, &out);
    /* pulse 1: ON x3 at 10; pulse 2: OFF x3 at 20; pulse 3: ON x3 at 30; pulse 4: OFF x3 */
    PCL_CHECK_EQ_INT(out.n, 12);
    PCL_CHECK(out.e[0].note == 60 && out.e[1].note == 64 && out.e[2].note == 67);
    PCL_CHECK(out.e[0].type == PCL_EVT_NOTE_ON && out.e[3].type == PCL_EVT_NOTE_OFF);
    PCL_CHECK(out.e[0].velocity == 101 && out.e[1].velocity == 102 && out.e[2].velocity == 100);
}

static void test_note_clamp(void)
{
    static const uint8_t keys[] = { 120 };
    pcl_arp_t            a;
    evl_t                out;
    uint8_t              got[8];
    size_t               n;

    arp_new(&a, PCL_ARP_UP, 4, 2, 1);
    (void)play(&a, keys, 1, 8, &out);
    n = ons(&out, got, 8);
    PCL_CHECK_EQ_INT(n, 4);
    PCL_CHECK(got[0] == 120 && got[1] == 127 && got[2] == 127 && got[3] == 127);
}

static void test_note_offs_and_empty_set(void)
{
    pcl_arp_t a;
    evl_t     in, out;

    arp_new(&a, PCL_ARP_UP, 1, 4, 4);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 60, 100);
    put(&in, 0, PCL_EVT_NOTE_ON, 64, 100);
    put(&in, 5, PCL_EVT_CLOCK, 0, 0); /* 60 sounds */
    put(&in, 8, PCL_EVT_NOTE_OFF, 64, 0);
    put(&in, 9, PCL_EVT_CLOCK, 0, 0);
    put(&in, 20, PCL_EVT_NOTE_ON, 60, 0); /* velocity 0 is a note off: the last key goes */
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 0);
    /* ON 60 at 5; the empty set releases it at 20 (at the key-up that emptied it) */
    PCL_CHECK_EQ_INT(out.n, 2);
    PCL_CHECK(out.e[0].type == PCL_EVT_NOTE_ON && out.e[0].note == 60 && out.e[0].offset == 5);
    PCL_CHECK(out.e[1].type == PCL_EVT_NOTE_OFF && out.e[1].note == 60 && out.e[1].offset == 20);

    /* a step with an empty set is silent, and pulses still count */
    in.n = 0;
    put(&in, 0, PCL_EVT_CLOCK, 0, 0);
    put(&in, 1, PCL_EVT_CLOCK, 0, 0);
    PCL_CHECK_EQ_INT(pcl_arp_process(&a, in.e, in.n, out.e, MAXEV), 0);

    /* unknown note offs and unknown event types do nothing */
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_OFF, 99, 0);
    put(&in, 1, 0xF0, 1, 1);
    PCL_CHECK_EQ_INT(pcl_arp_process(&a, in.e, in.n, out.e, MAXEV), 0);
    PCL_CHECK_EQ_INT(pcl_arp_process(&a, NULL, 0, out.e, MAXEV), 0);
}

static void test_key_changes_mid_pattern(void)
{
    pcl_arp_t a;
    evl_t     in, out;
    uint8_t   got[16];
    size_t    n;

    arp_new(&a, PCL_ARP_UP, 1, 2, 1);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 60, 100);
    put(&in, 0, PCL_EVT_NOTE_ON, 67, 100);
    put(&in, 1, PCL_EVT_CLOCK, 0, 0); /* 60 */
    put(&in, 2, PCL_EVT_CLOCK, 0, 0);
    put(&in, 3, PCL_EVT_CLOCK, 0, 0); /* 67 */
    put(&in, 4, PCL_EVT_CLOCK, 0, 0);
    put(&in, 4, PCL_EVT_NOTE_ON, 64, 100); /* joins: 60 64 67 */
    put(&in, 5, PCL_EVT_CLOCK, 0, 0);      /* position 2 of 3: 67 */
    put(&in, 6, PCL_EVT_CLOCK, 0, 0);
    put(&in, 7, PCL_EVT_CLOCK, 0, 0); /* position 3 -> 60 */
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    n = ons(&out, got, 16);
    PCL_CHECK_EQ_INT(n, 4);
    PCL_CHECK(got[0] == 60 && got[1] == 67 && got[2] == 67 && got[3] == 60);

    /* a held note of a growing set is never skipped or doubled by the position: with the set
       shrinking below the position the pattern wraps by modulo */
    arp_new(&a, PCL_ARP_UP, 1, 1, 1);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 60, 100);
    put(&in, 0, PCL_EVT_NOTE_ON, 62, 100);
    put(&in, 0, PCL_EVT_NOTE_ON, 64, 100);
    put(&in, 1, PCL_EVT_CLOCK, 0, 0);
    put(&in, 2, PCL_EVT_CLOCK, 0, 0);
    put(&in, 3, PCL_EVT_CLOCK, 0, 0);
    put(&in, 3, PCL_EVT_NOTE_OFF, 64, 0);
    put(&in, 3, PCL_EVT_NOTE_OFF, 62, 0);
    put(&in, 4, PCL_EVT_CLOCK, 0, 0);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    n = ons(&out, got, 16);
    PCL_CHECK_EQ_INT(n, 4);
    PCL_CHECK(got[3] == 60);
}

static void test_latch(void)
{
    pcl_arp_t        a;
    pcl_arp_config_t c = cfg_of(PCL_ARP_UP, 1, 2, 1);
    evl_t            in, out;
    uint8_t          got[16];
    size_t           n;

    c.latch = 1;
    PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_OK);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 60, 100);
    put(&in, 0, PCL_EVT_NOTE_ON, 64, 100);
    put(&in, 1, PCL_EVT_NOTE_OFF, 60, 0); /* released but latched */
    put(&in, 1, PCL_EVT_NOTE_OFF, 64, 0);
    put(&in, 2, PCL_EVT_CLOCK, 0, 0);
    put(&in, 3, PCL_EVT_CLOCK, 0, 0);
    put(&in, 4, PCL_EVT_CLOCK, 0, 0);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 2);
    n = ons(&out, got, 16);
    PCL_CHECK_EQ_INT(n, 2);
    PCL_CHECK(got[0] == 60 && got[1] == 64);

    /* a new chord replaces the latched one */
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 70, 100);
    put(&in, 0, PCL_EVT_NOTE_ON, 74, 100);
    put(&in, 1, PCL_EVT_NOTE_OFF, 70, 0);
    put(&in, 1, PCL_EVT_NOTE_OFF, 74, 0);
    PCL_CHECK_EQ_INT(pcl_arp_process(&a, in.e, in.n, out.e, MAXEV) <= 3, 1);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 2);
    in.n = 0;
    put(&in, 0, PCL_EVT_CLOCK, 0, 0); /* the pulse count is odd here: no step */
    put(&in, 1, PCL_EVT_CLOCK, 0, 0);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    n = ons(&out, got, 16);
    PCL_CHECK(n == 1 && got[0] == 70);

    /* a key added while another is still down joins the chord */
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 50, 100);
    put(&in, 1, PCL_EVT_NOTE_ON, 52, 100);
    (void)pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 2);

    /* latch off drops the keys that are up */
    pcl_arp_set_latch(&a, 0);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 2); /* 50 and 52 are physically down */
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_OFF, 50, 0);
    (void)pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 1);
    pcl_arp_set_latch(&a, 1);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_OFF, 52, 0);
    (void)pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 1); /* latched */
    pcl_arp_set_latch(&a, 0);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 0);
}

static void test_transport(void)
{
    static const uint8_t keys[] = { 60, 64, 67 };
    pcl_arp_t            a;
    evl_t                in, out;
    uint8_t              got[16];
    size_t               n;

    arp_new(&a, PCL_ARP_UP, 1, 4, 4);
    (void)play(&a, keys, 3, 2, &out); /* 60 sounds */
    in.n = 0;
    put(&in, 3, PCL_EVT_STOP, 0, 0);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(out.n, 1);
    PCL_CHECK(out.e[0].type == PCL_EVT_NOTE_OFF && out.e[0].note == 60 && out.e[0].offset == 3);
    /* STOP with nothing sounding says nothing */
    PCL_CHECK_EQ_INT(pcl_arp_process(&a, in.e, in.n, out.e, MAXEV), 0);

    /* START restarts the pattern from the first note at the next pulse */
    in.n = 0;
    put(&in, 0, PCL_EVT_START, 0, 0);
    put(&in, 0, PCL_EVT_CLOCK, 0, 0);
    put(&in, 4, PCL_EVT_CLOCK, 0, 0);
    put(&in, 5, PCL_EVT_CLOCK, 0, 0);
    put(&in, 6, PCL_EVT_CLOCK, 0, 0);
    put(&in, 7, PCL_EVT_CLOCK, 0, 0);
    put(&in, 8, PCL_EVT_CLOCK, 0, 0); /* second step of the new run */
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    n = ons(&out, got, 16);
    PCL_CHECK_EQ_INT(n, 2);
    PCL_CHECK(got[0] == 60 && got[1] == 64);

    /* START while a note sounds releases it, OFF at the START offset */
    in.n = 0;
    put(&in, 1, PCL_EVT_START, 0, 0);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK(out.n == 1 && out.e[0].type == PCL_EVT_NOTE_OFF && out.e[0].offset == 1);
}

static void test_random(void)
{
    static const uint8_t keys[] = { 60, 64, 67 };
    pcl_arp_t            a, b;
    evl_t                o1, o2;
    uint8_t              g1[200], g2[200];
    size_t               n1, n2, i;
    unsigned             seen[3] = { 0, 0, 0 }, oct[2] = { 0, 0 };

    arp_new(&a, PCL_ARP_RANDOM, 2, 2, 1);
    arp_new(&b, PCL_ARP_RANDOM, 2, 2, 1);
    {
        pcl_arp_t a3 = a;

        (void)a3;
    }
    (void)play(&a, keys, 3, 200, &o1);
    n1 = ons(&o1, g1, 200);
    (void)play(&b, keys, 3, 200, &o2);
    n2 = ons(&o2, g2, 200);
    PCL_CHECK_EQ_INT(n1, 100);
    PCL_CHECK_EQ_INT(n2, 100);
    PCL_CHECK(memcmp(g1, g2, 100) == 0); /* same seed, same sequence */
    for (i = 0; i < n1; i++)
    {
        const uint8_t  g = g1[i];
        const unsigned o = g >= 72 ? 1u : 0u;
        const uint8_t  base = (uint8_t)(g - 12 * o);

        PCL_CHECK(base == 60 || base == 64 || base == 67);
        seen[base == 60 ? 0 : base == 64 ? 1 : 2]++;
        oct[o]++;
    }
    for (i = 0; i < 3; i++)
        PCL_CHECK(seen[i] > 10); /* roughly uniform over 6 notes: ~16 each */
    PCL_CHECK(oct[0] > 25 && oct[1] > 25);

    /* a different seed gives a different sequence */
    {
        pcl_arp_config_t c = cfg_of(PCL_ARP_RANDOM, 2, 2, 1);
        pcl_arp_t        d;
        evl_t            o3;
        uint8_t          g3[200];

        c.seed = 0xDEADBEEFu;
        PCL_CHECK_EQ_INT(pcl_arp_init(&d, &c), PCL_OK);
        (void)play(&d, keys, 3, 200, &o3);
        (void)ons(&o3, g3, 200);
        PCL_CHECK(memcmp(g1, g3, 100) != 0);
    }
}

static void test_too_many_keys(void)
{
    pcl_arp_t a;
    evl_t     in, out;
    int       k;

    arp_new(&a, PCL_ARP_UP, 1, 2, 1);
    in.n = 0;
    for (k = 0; k < 20; k++)
        put(&in, 0, PCL_EVT_NOTE_ON, (uint8_t)(40 + k), 100);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), PCL_ARP_MAX_NOTES);
    /* a key already held that is pressed again is not a new key */
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 41, 90);
    (void)pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), PCL_ARP_MAX_NOTES);
}

static void test_capacity(void)
{
    static const uint8_t keys[] = { 60, 62, 64, 65, 67, 69, 71, 72 };
    pcl_arp_t            a, b;
    evl_t                in, full, part;
    size_t               i, n, j;

    /* chord: 8 notes on/off per step. Same input into a roomy and a tight buffer: the tight one
       gets the first events of the full stream, the rest are counted */
    arp_new(&a, PCL_ARP_CHORD, 1, 2, 1);
    arp_new(&b, PCL_ARP_CHORD, 1, 2, 1);
    in.n = 0;
    for (i = 0; i < 8; i++)
        put(&in, 0, PCL_EVT_NOTE_ON, keys[i], 100);
    for (i = 0; i < 4; i++)
        put(&in, (uint16_t)(1 + i), PCL_EVT_CLOCK, 0, 0);
    full.n = pcl_arp_process(&a, in.e, in.n, full.e, MAXEV);
    PCL_CHECK_EQ_INT(full.n, 32);
    PCL_CHECK_EQ_INT(pcl_arp_dropped(&a), 0);
    n = pcl_arp_process(&b, in.e, in.n, part.e, 20);
    PCL_CHECK_EQ_INT(n, 20);
    PCL_CHECK_EQ_INT(pcl_arp_dropped(&b), 12);
    for (j = 0; j < n; j++)
        PCL_CHECK(part.e[j].offset == full.e[j].offset && part.e[j].type == full.e[j].type
            && part.e[j].note == full.e[j].note);
    /* zero capacity */
    arp_new(&b, PCL_ARP_CHORD, 1, 2, 1);
    PCL_CHECK_EQ_INT(pcl_arp_process(&b, in.e, in.n, NULL, 0), 0);
    PCL_CHECK_EQ_INT(pcl_arp_dropped(&b), 32);
}

/* Removing a key from the middle keeps the others' velocities and key states. */
static void test_removal_keeps_attributes(void)
{
    pcl_arp_t        a;
    pcl_arp_config_t c = cfg_of(PCL_ARP_UP, 1, 2, 1);
    evl_t            in, out;
    size_t           i;
    uint8_t          vel[8];
    size_t           n = 0;

    PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_OK);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 60, 100);
    put(&in, 0, PCL_EVT_NOTE_ON, 64, 101);
    put(&in, 0, PCL_EVT_NOTE_ON, 67, 102);
    put(&in, 0, PCL_EVT_NOTE_ON, 71, 103);
    put(&in, 1, PCL_EVT_NOTE_OFF, 64, 0);
    put(&in, 2, PCL_EVT_CLOCK, 0, 0);
    put(&in, 3, PCL_EVT_CLOCK, 0, 0);
    put(&in, 4, PCL_EVT_CLOCK, 0, 0);
    put(&in, 5, PCL_EVT_CLOCK, 0, 0);
    put(&in, 6, PCL_EVT_CLOCK, 0, 0);
    put(&in, 7, PCL_EVT_CLOCK, 0, 0);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    for (i = 0; i < out.n; i++)
        if (out.e[i].type == PCL_EVT_NOTE_ON && n < 8)
            vel[n++] = out.e[i].velocity;
    PCL_CHECK_EQ_INT(n, 3);
    PCL_CHECK(vel[0] == 100 && vel[1] == 102 && vel[2] == 103);

    /* latched: a released key in the middle is dropped by latch-off, the keys still down stay */
    c.latch = 1;
    PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_OK);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 60, 100);
    put(&in, 0, PCL_EVT_NOTE_ON, 64, 101);
    put(&in, 0, PCL_EVT_NOTE_ON, 67, 102);
    put(&in, 1, PCL_EVT_NOTE_OFF, 64, 0);
    (void)pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 3);
    pcl_arp_set_latch(&a, 0);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 2);
    in.n = 0;
    put(&in, 0, PCL_EVT_CLOCK, 0, 0);
    put(&in, 1, PCL_EVT_CLOCK, 0, 0);
    put(&in, 2, PCL_EVT_CLOCK, 0, 0);
    put(&in, 3, PCL_EVT_CLOCK, 0, 0);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    {
        uint8_t got[8];

        PCL_CHECK_EQ_INT(ons(&out, got, 8), 2);
        PCL_CHECK(got[0] == 60 && got[1] == 67);
    }

    /* the same key pressed again updates its velocity and is not a second entry */
    PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_OK);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 60, 100);
    put(&in, 1, PCL_EVT_NOTE_ON, 60, 90);
    (void)pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 1);
    in.n = 0;
    put(&in, 0, PCL_EVT_CLOCK, 0, 0);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK(out.n == 1 && out.e[0].velocity == 90);
}

/* Latch: the chord is replaced only when no key of it is still down. */
static void test_latch_chord_replacement(void)
{
    pcl_arp_t        a;
    pcl_arp_config_t c = cfg_of(PCL_ARP_UP, 1, 2, 1);
    evl_t            in, out;

    c.latch = 1;
    PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_OK);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 50, 100);
    put(&in, 0, PCL_EVT_NOTE_ON, 52, 100);
    put(&in, 1, PCL_EVT_NOTE_OFF, 50, 0); /* 52 is still down */
    put(&in, 2, PCL_EVT_NOTE_ON, 54, 100);
    (void)pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 3); /* 50 (latched), 52, 54 */

    /* a latched key pressed again counts as down */
    PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_OK);
    in.n = 0;
    put(&in, 0, PCL_EVT_NOTE_ON, 60, 100);
    put(&in, 1, PCL_EVT_NOTE_OFF, 60, 0);
    put(&in, 2, PCL_EVT_NOTE_ON, 60, 100);
    put(&in, 3, PCL_EVT_NOTE_ON, 64, 100);
    (void)pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(pcl_arp_held(&a), 2);
}

/* The AS_PLAYED pattern takes each note's own velocity across octaves. */
static void test_as_played_velocity(void)
{
    static const uint8_t keys[] = { 67, 60, 64 }; /* velocities 100, 101, 102 */
    pcl_arp_t            a;
    evl_t                out;
    size_t               i, n = 0;
    uint8_t              vel[8];

    arp_new(&a, PCL_ARP_AS_PLAYED, 2, 2, 1);
    (void)play(&a, keys, 3, 12, &out);
    for (i = 0; i < out.n; i++)
        if (out.e[i].type == PCL_EVT_NOTE_ON && n < 8)
            vel[n++] = out.e[i].velocity;
    PCL_CHECK_EQ_INT(n, 6);
    PCL_CHECK(vel[0] == 100 && vel[1] == 101 && vel[2] == 102);
    PCL_CHECK(vel[3] == 100 && vel[4] == 101 && vel[5] == 102);
}

/* CHORD steps count: switching to UP afterwards continues from the step count. */
static void test_chord_counts_steps(void)
{
    static const uint8_t keys[] = { 60, 64, 67 };
    pcl_arp_t            a;
    evl_t                out;
    uint8_t              got[4];

    arp_new(&a, PCL_ARP_CHORD, 1, 2, 1);
    (void)play(&a, keys, 3, 1, &out); /* one chord step */
    PCL_CHECK_EQ_INT(pcl_arp_set_mode(&a, PCL_ARP_UP), PCL_OK);
    {
        evl_t in;

        in.n = 0;
        put(&in, 0, PCL_EVT_CLOCK, 0, 0);
        put(&in, 1, PCL_EVT_CLOCK, 0, 0);
        out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    }
    PCL_CHECK_EQ_INT(ons(&out, got, 4), 1);
    PCL_CHECK_EQ_INT(got[0], 64);
}

/* STOP releases and nothing else; START restarts the count and the position. */
static void test_stop_and_start_state(void)
{
    static const uint8_t keys[] = { 60, 64, 67 };
    pcl_arp_t            a;
    evl_t                in, out;
    uint8_t              got[8];

    arp_new(&a, PCL_ARP_UP, 1, 2, 1);
    (void)play(&a, keys, 3, 3, &out); /* steps at pulses 0 and 2: 60, 64; pulse count 3 */
    in.n = 0;
    put(&in, 0, PCL_EVT_STOP, 0, 0);
    put(&in, 1, PCL_EVT_CLOCK, 0, 0); /* pulse 3, odd: nothing */
    put(&in, 2, PCL_EVT_CLOCK, 0, 0); /* pulse 4: third step, 67 */
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(ons(&out, got, 8), 1);
    PCL_CHECK_EQ_INT(got[0], 67);

    /* START without a STOP before it: back to the first note and an even pulse */
    in.n = 0;
    put(&in, 0, PCL_EVT_START, 0, 0);
    put(&in, 1, PCL_EVT_CLOCK, 0, 0);
    put(&in, 2, PCL_EVT_CLOCK, 0, 0);
    put(&in, 3, PCL_EVT_CLOCK, 0, 0);
    out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
    PCL_CHECK_EQ_INT(ons(&out, got, 8), 2);
    PCL_CHECK(got[0] == 60 && got[1] == 64);
}

/* A random stream of keys, clocks and transport: the output is sorted, every ON is eventually
   released, a released note was sounding, and the counts stay within the documented bounds. */
static void property_run(
    pcl_arp_mode_t mode, uint32_t oct, uint32_t div, uint32_t gate, int latch, uint32_t seed)
{
    pcl_arp_t        a;
    pcl_arp_config_t c = cfg_of(mode, oct, div, gate);
    unsigned         sounding[128];
    uint32_t         s = seed;
    int              blk;
    int              k;

    c.latch = latch;
    c.seed = seed;
    PCL_CHECK_EQ_INT(pcl_arp_init(&a, &c), PCL_OK);
    memset(sounding, 0, sizeof sounding);
    for (blk = 0; blk < 300; blk++)
    {
        evl_t    in, out;
        unsigned off = 0;
        size_t   i;

        in.n = 0;
        for (k = 0; k < 24; k++)
        {
            s = s * 1664525u + 1013904223u;
            off += (s >> 24) % 20u;
            switch ((s >> 8) % 8u)
            {
            case 0:
            case 1:
                put(&in, (uint16_t)off, PCL_EVT_NOTE_ON, (uint8_t)(36 + (s >> 12) % 40u),
                    (uint8_t)((s >> 20) % 128u)); /* velocity 0 is an off */
                break;
            case 2:
                put(&in, (uint16_t)off, PCL_EVT_NOTE_OFF, (uint8_t)(36 + (s >> 12) % 40u), 0);
                break;
            case 3:
                if ((s >> 16) % 16u == 0u)
                    put(&in, (uint16_t)off, (s >> 20) & 1u ? PCL_EVT_START : PCL_EVT_STOP, 0, 0);
                break;
            default:
                put(&in, (uint16_t)off, PCL_EVT_CLOCK, 0, 0);
                break;
            }
        }
        out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
        PCL_CHECK(out.n < MAXEV);
        for (i = 0; i < out.n; i++)
        {
            PCL_CHECK(out.e[i].note < 128);
            if (i > 0)
                PCL_CHECK(out.e[i].offset >= out.e[i - 1].offset);
            if (out.e[i].type == PCL_EVT_NOTE_ON)
            {
                PCL_CHECK(out.e[i].velocity > 0);
                sounding[out.e[i].note]++;
            }
            else
            {
                PCL_CHECK(out.e[i].type == PCL_EVT_NOTE_OFF);
                PCL_CHECK(out.e[i].velocity == 0);
                PCL_CHECK(sounding[out.e[i].note] > 0);
                if (sounding[out.e[i].note] > 0)
                    sounding[out.e[i].note]--;
            }
        }
    }
    PCL_CHECK_EQ_INT(pcl_arp_dropped(&a), 0);
    /* STOP releases whatever is left */
    {
        evl_t in, out;
        int   n;

        in.n = 0;
        put(&in, 0, PCL_EVT_STOP, 0, 0);
        out.n = pcl_arp_process(&a, in.e, in.n, out.e, MAXEV);
        for (k = 0; k < (int)out.n; k++)
            if (sounding[out.e[k].note] > 0)
                sounding[out.e[k].note]--;
        for (n = 0; n < 128; n++)
            PCL_CHECK_EQ_INT(sounding[n], 0);
    }
}

static void test_property_streams(void)
{
    pcl_arp_mode_t m;
    uint32_t       seed = 1;

    for (m = PCL_ARP_UP; m <= PCL_ARP_CHORD; m = (pcl_arp_mode_t)(m + 1))
    {
        property_run(m, 1, 1, 1, 0, seed++);
        property_run(m, 2, 3, 2, 0, seed++);
        property_run(m, 4, 6, 6, 1, seed++);
        property_run(m, 3, 2, 96, 0, seed++);
        property_run(m, 1, 96, 1, 1, seed++);
    }
}

int main(void)
{
    PCL_RUN_TEST(test_init_validation);
    PCL_RUN_TEST(test_mode_sequences);
    PCL_RUN_TEST(test_small_sets);
    PCL_RUN_TEST(test_timing);
    PCL_RUN_TEST(test_division_one);
    PCL_RUN_TEST(test_chord);
    PCL_RUN_TEST(test_note_clamp);
    PCL_RUN_TEST(test_note_offs_and_empty_set);
    PCL_RUN_TEST(test_key_changes_mid_pattern);
    PCL_RUN_TEST(test_latch);
    PCL_RUN_TEST(test_transport);
    PCL_RUN_TEST(test_random);
    PCL_RUN_TEST(test_too_many_keys);
    PCL_RUN_TEST(test_capacity);
    PCL_RUN_TEST(test_removal_keeps_attributes);
    PCL_RUN_TEST(test_latch_chord_replacement);
    PCL_RUN_TEST(test_as_played_velocity);
    PCL_RUN_TEST(test_chord_counts_steps);
    PCL_RUN_TEST(test_stop_and_start_state);
    PCL_RUN_TEST(test_property_streams);
    return pcl_test_summary("arp");
}
