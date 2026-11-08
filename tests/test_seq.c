/*
 * pcl_seq tests: step order for each direction, gates, rests, ties, swing, transpose, transport,
 * counter wraparound, capacity limits, an independent reference model, and a random-stream
 * property test against a monophonic voice model.
 */
#include <string.h>

#include "pcl_test.h"

#define MAXREC 16384
#define PULSE_SPACING 10u
#define CALL_PULSES 400u /* pulses per process() call: offsets stay below 65536 */

typedef struct
{
    unsigned pulse;
    uint8_t  type;
    uint8_t  note;
    uint8_t  vel;
} rec_t;

typedef struct
{
    rec_t  r[MAXREC];
    size_t n;
} recs_t;

static recs_t   g_recs;
static uint32_t g_rng;

static unsigned rnd(unsigned m)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return (g_rng >> 10) % m;
}

static void add_rec(recs_t *l, const pcl_event_t *e, unsigned base)
{
    PCL_CHECK(l->n < MAXREC);
    if (l->n >= MAXREC)
        return;
    l->r[l->n].pulse = base + e->offset / PULSE_SPACING;
    l->r[l->n].type = e->type;
    l->r[l->n].note = e->note;
    l->r[l->n].vel = e->velocity;
    l->n++;
}

/* Sends `count` clock pulses starting at pulse number `first` (offsets are pulse * 10 within a
   call). Records of every event produced are appended to `l`. */
static void clocks(pcl_seq_t *s, unsigned first, unsigned count, recs_t *l)
{
    unsigned done = 0;

    while (done < count)
    {
        pcl_event_t    in[CALL_PULSES];
        pcl_event_t    out[2 * CALL_PULSES + 4];
        const unsigned chunk = (count - done < CALL_PULSES) ? count - done : CALL_PULSES;
        unsigned       i;
        size_t         n;

        for (i = 0; i < chunk; i++)
        {
            in[i].offset = (uint16_t)(i * PULSE_SPACING + 1u);
            in[i].type = PCL_EVT_CLOCK;
            in[i].note = 0;
            in[i].velocity = 0;
        }
        n = pcl_seq_process(s, in, chunk, out, sizeof out / sizeof out[0]);
        for (i = 0; i < n; i++)
            add_rec(l, &out[i], first + done);
        done += chunk;
    }
}

static pcl_seq_config_t cfg_of(
    uint32_t length, uint32_t division, uint32_t gate, uint32_t swing, pcl_seq_direction_t dir)
{
    pcl_seq_config_t c;

    c.length = length;
    c.division = division;
    c.gate = gate;
    c.swing = swing;
    c.direction = dir;
    c.transpose = 0;
    c.seed = 0;
    return c;
}

static void step(pcl_seq_t *s, size_t i, uint8_t note, uint8_t vel, uint8_t gate, uint8_t flags)
{
    pcl_seq_step_t st;

    st.note = note;
    st.velocity = vel;
    st.gate = gate;
    st.flags = flags;
    PCL_CHECK_EQ_INT(pcl_seq_set_step(s, i, &st), PCL_OK);
}

static void seq_new(pcl_seq_t *s, uint32_t length, uint32_t division, uint32_t gate, uint32_t swing,
    pcl_seq_direction_t dir)
{
    const pcl_seq_config_t c = cfg_of(length, division, gate, swing, dir);

    PCL_CHECK_EQ_INT(pcl_seq_init(s, &c), PCL_OK);
}

/* Fills steps 0.. with the notes, velocity 100, no gate override, no flags. */
static void notes(pcl_seq_t *s, const uint8_t *n, size_t count)
{
    size_t i;

    for (i = 0; i < count; i++)
        step(s, i, n[i], 100, 0, 0);
}

static int is(const recs_t *l, size_t i, unsigned pulse, uint8_t type, uint8_t note)
{
    return i < l->n && l->r[i].pulse == pulse && l->r[i].type == type && l->r[i].note == note;
}

static void test_init_and_setters(void)
{
    pcl_seq_t        s;
    pcl_seq_config_t c = cfg_of(8, 6, 3, 2, PCL_SEQ_FORWARD);
    pcl_seq_step_t   st;

    PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_seq_init(NULL, &c), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_seq_init(&s, NULL), PCL_ERR_NULL_POINTER);
    {
        pcl_seq_t keep = s;

        c = cfg_of(0, 6, 3, 2, PCL_SEQ_FORWARD);
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(65, 6, 3, 2, PCL_SEQ_FORWARD);
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(8, 0, 3, 0, PCL_SEQ_FORWARD);
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(8, 97, 3, 0, PCL_SEQ_FORWARD);
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(8, 6, 0, 0, PCL_SEQ_FORWARD);
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(8, 6, 97, 0, PCL_SEQ_FORWARD);
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(8, 6, 3, 6, PCL_SEQ_FORWARD); /* swing must stay below the division */
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_ERR_OUT_OF_RANGE);
        c = cfg_of(8, 6, 3, 2, (pcl_seq_direction_t)4);
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_ERR_OUT_OF_RANGE);
        PCL_CHECK(memcmp(&s, &keep, sizeof s) == 0);
    }
    c = cfg_of(64, 96, 96, 95, PCL_SEQ_RANDOM);
    PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_OK);

    /* steps start as rests */
    {
        size_t i;

        for (i = 0; i < PCL_SEQ_MAX_STEPS; i++)
        {
            PCL_CHECK_EQ_INT(pcl_seq_get_step(&s, i, &st), PCL_OK);
            PCL_CHECK(st.note == 60 && st.velocity == 0 && st.gate == 0 && st.flags == 0);
        }
    }
    PCL_CHECK_EQ_INT(pcl_seq_get_step(&s, 63, &st), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_seq_get_step(&s, 64, &st), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_get_step(NULL, 0, &st), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_seq_get_step(&s, 0, NULL), PCL_ERR_NULL_POINTER);

    st.note = 60, st.velocity = 100, st.gate = 3, st.flags = PCL_SEQ_TIE;
    PCL_CHECK_EQ_INT(pcl_seq_set_step(&s, 5, &st), PCL_OK);
    {
        pcl_seq_step_t back;

        PCL_CHECK_EQ_INT(pcl_seq_get_step(&s, 5, &back), PCL_OK);
        PCL_CHECK(
            back.note == 60 && back.velocity == 100 && back.gate == 3 && back.flags == PCL_SEQ_TIE);
    }
    PCL_CHECK_EQ_INT(pcl_seq_set_step(&s, 64, &st), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_step(NULL, 0, &st), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_seq_set_step(&s, 0, NULL), PCL_ERR_NULL_POINTER);
    st.note = 127, st.velocity = 127, st.gate = 96, st.flags = 0; /* the limits are accepted */
    PCL_CHECK_EQ_INT(pcl_seq_set_step(&s, 63, &st), PCL_OK);
    st.note = 128;
    PCL_CHECK_EQ_INT(pcl_seq_set_step(&s, 0, &st), PCL_ERR_OUT_OF_RANGE);
    st.note = 60, st.velocity = 128;
    PCL_CHECK_EQ_INT(pcl_seq_set_step(&s, 0, &st), PCL_ERR_OUT_OF_RANGE);
    st.velocity = 100, st.gate = 97;
    PCL_CHECK_EQ_INT(pcl_seq_set_step(&s, 0, &st), PCL_ERR_OUT_OF_RANGE);
    st.gate = 0, st.flags = 2;
    PCL_CHECK_EQ_INT(pcl_seq_set_step(&s, 0, &st), PCL_ERR_OUT_OF_RANGE);

    PCL_CHECK_EQ_INT(pcl_seq_set_length(&s, 16), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_seq_set_length(&s, 0), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_length(&s, 65), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_gate(&s, 4), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_seq_set_gate(&s, 0), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_gate(&s, 97), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_direction(&s, PCL_SEQ_PINGPONG), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_seq_set_direction(&s, (pcl_seq_direction_t)4), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_division(&s, 6), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_seq_set_swing(&s, 5), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_seq_set_swing(&s, 6), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_division(&s, 3), PCL_OK); /* the swing follows it down */
    PCL_CHECK_EQ_INT(pcl_seq_set_swing(&s, 2), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_seq_set_swing(&s, 3), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_division(&s, 0), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_division(&s, 97), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_seq_set_length(NULL, 4), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_seq_set_swing(NULL, 1), PCL_ERR_NULL_POINTER);
}

static void test_all_rests_is_silent(void)
{
    pcl_seq_t s;

    seq_new(&s, 8, 6, 3, 0, PCL_SEQ_FORWARD);
    g_recs.n = 0;
    clocks(&s, 0, 200, &g_recs);
    PCL_CHECK_EQ_INT(g_recs.n, 0);
}

static void test_forward_timing(void)
{
    static const uint8_t pat[] = { 60, 62, 64, 65 };
    pcl_seq_t            s;
    size_t               i;

    seq_new(&s, 4, 2, 1, 0, PCL_SEQ_FORWARD);
    notes(&s, pat, 4);
    g_recs.n = 0;
    clocks(&s, 0, 12, &g_recs);
    /* ON at pulses 0,2,4,..., OFF one pulse later; step velocity 100 */
    PCL_CHECK_EQ_INT(g_recs.n, 12);
    for (i = 0; i < 6; i++)
    {
        PCL_CHECK(is(&g_recs, 2 * i, (unsigned)(2 * i), PCL_EVT_NOTE_ON, pat[i % 4]));
        PCL_CHECK(g_recs.r[2 * i].vel == 100);
        PCL_CHECK(is(&g_recs, 2 * i + 1, (unsigned)(2 * i + 1), PCL_EVT_NOTE_OFF, pat[i % 4]));
    }
    PCL_CHECK_EQ_INT(pcl_seq_position(&s), 1); /* 6th step: 6 % 4 = 2 steps in; played last = 1 */

    /* length shorter than the steps set: the rest are not played */
    seq_new(&s, 3, 1, 1, 0, PCL_SEQ_FORWARD);
    notes(&s, pat, 4);
    g_recs.n = 0;
    clocks(&s, 0, 7, &g_recs);
    {
        uint8_t got[16];
        size_t  n = 0;

        for (i = 0; i < g_recs.n; i++)
            if (g_recs.r[i].type == PCL_EVT_NOTE_ON)
                got[n++] = g_recs.r[i].note;
        PCL_CHECK_EQ_INT(n, 7);
        PCL_CHECK(got[0] == 60 && got[1] == 62 && got[2] == 64 && got[3] == 60 && got[6] == 60);
    }

    /* gate longer than division with no tie: the note is cut by the next step, OFF then ON */
    seq_new(&s, 2, 2, 5, 0, PCL_SEQ_FORWARD);
    notes(&s, pat, 2);
    g_recs.n = 0;
    clocks(&s, 0, 6, &g_recs);
    PCL_CHECK_EQ_INT(g_recs.n, 5);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 1, 2, PCL_EVT_NOTE_OFF, 60));
    PCL_CHECK(is(&g_recs, 2, 2, PCL_EVT_NOTE_ON, 62));
    PCL_CHECK(is(&g_recs, 3, 4, PCL_EVT_NOTE_OFF, 62));
    PCL_CHECK(is(&g_recs, 4, 4, PCL_EVT_NOTE_ON, 60));

    /* a per-step gate overrides the default */
    seq_new(&s, 2, 6, 1, 0, PCL_SEQ_FORWARD);
    step(&s, 0, 60, 90, 4, 0);
    step(&s, 1, 62, 80, 0, 0);
    g_recs.n = 0;
    clocks(&s, 0, 13, &g_recs);
    PCL_CHECK_EQ_INT(g_recs.n, 5);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60) && g_recs.r[0].vel == 90);
    PCL_CHECK(is(&g_recs, 1, 4, PCL_EVT_NOTE_OFF, 60));
    PCL_CHECK(is(&g_recs, 2, 6, PCL_EVT_NOTE_ON, 62) && g_recs.r[2].vel == 80);
    PCL_CHECK(is(&g_recs, 3, 7, PCL_EVT_NOTE_OFF, 62));
    PCL_CHECK(is(&g_recs, 4, 12, PCL_EVT_NOTE_ON, 60));
}

static void check_order(pcl_seq_direction_t dir, uint32_t length, const uint8_t *want, size_t nwant)
{
    pcl_seq_t s;
    uint8_t   pat[8];
    size_t    i, n = 0;
    uint8_t   got[64];

    for (i = 0; i < 8; i++)
        pat[i] = (uint8_t)(60 + i);
    seq_new(&s, length, 1, 1, 0, dir);
    notes(&s, pat, 8);
    g_recs.n = 0;
    clocks(&s, 0, (unsigned)nwant, &g_recs);
    for (i = 0; i < g_recs.n && n < 64; i++)
        if (g_recs.r[i].type == PCL_EVT_NOTE_ON)
            got[n++] = g_recs.r[i].note;
    PCL_CHECK_EQ_INT(n, nwant);
    if (n == nwant)
        PCL_CHECK(memcmp(got, want, nwant) == 0);
}

static void test_directions(void)
{
    static const uint8_t bw4[] = { 63, 62, 61, 60, 63, 62 };
    static const uint8_t pp4[] = { 60, 61, 62, 63, 62, 61, 60, 61, 62, 63, 62 };
    static const uint8_t pp2[] = { 60, 61, 60, 61, 60 };
    static const uint8_t pp3[] = { 60, 61, 62, 61, 60, 61, 62 };
    static const uint8_t one[] = { 60, 60, 60, 60 };

    check_order(PCL_SEQ_BACKWARD, 4, bw4, sizeof bw4);
    check_order(PCL_SEQ_PINGPONG, 4, pp4, sizeof pp4);
    check_order(PCL_SEQ_PINGPONG, 2, pp2, sizeof pp2);
    check_order(PCL_SEQ_PINGPONG, 3, pp3, sizeof pp3);
    check_order(PCL_SEQ_PINGPONG, 1, one, sizeof one);
    check_order(PCL_SEQ_BACKWARD, 1, one, sizeof one);
    check_order(PCL_SEQ_FORWARD, 1, one, sizeof one);
}

static void test_random_direction(void)
{
    pcl_seq_t        s1, s2, s3;
    pcl_seq_config_t c = cfg_of(8, 1, 1, 0, PCL_SEQ_RANDOM);
    uint8_t          pat[8];
    size_t           i;
    unsigned         seen[8] = { 0 };
    recs_t          *a = &g_recs;
    static recs_t    b;

    for (i = 0; i < 8; i++)
        pat[i] = (uint8_t)(60 + i);
    PCL_CHECK_EQ_INT(pcl_seq_init(&s1, &c), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_seq_init(&s2, &c), PCL_OK);
    c.seed = 777;
    PCL_CHECK_EQ_INT(pcl_seq_init(&s3, &c), PCL_OK);
    notes(&s1, pat, 8);
    notes(&s2, pat, 8);
    notes(&s3, pat, 8);
    a->n = 0;
    b.n = 0;
    clocks(&s1, 0, 400, a);
    clocks(&s2, 0, 400, &b);
    PCL_CHECK_EQ_INT(a->n, b.n);
    PCL_CHECK(memcmp(a->r, b.r, a->n * sizeof(rec_t)) == 0); /* same seed, same run */
    for (i = 0; i < a->n; i++)
        if (a->r[i].type == PCL_EVT_NOTE_ON)
        {
            PCL_CHECK(a->r[i].note >= 60 && a->r[i].note < 68);
            seen[a->r[i].note - 60]++;
        }
    for (i = 0; i < 8; i++)
        PCL_CHECK(seen[i] > 20); /* 400 draws over 8 steps: ~50 each */
    b.n = 0;
    clocks(&s3, 0, 400, &b);
    PCL_CHECK(memcmp(a->r, b.r, a->n * sizeof(rec_t)) != 0);
}

static void test_rests_and_ties(void)
{
    pcl_seq_t s;

    /* step 1 is a rest: nothing starts there, the gate release still comes */
    seq_new(&s, 3, 4, 2, 0, PCL_SEQ_FORWARD);
    step(&s, 0, 60, 100, 0, 0);
    step(&s, 1, 62, 0, 0, 0);
    step(&s, 2, 64, 100, 0, 0);
    g_recs.n = 0;
    clocks(&s, 0, 13, &g_recs);
    PCL_CHECK_EQ_INT(g_recs.n, 5); /* the loop comes round to ON 60 at pulse 12 */
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 1, 2, PCL_EVT_NOTE_OFF, 60));
    PCL_CHECK(is(&g_recs, 2, 8, PCL_EVT_NOTE_ON, 64));
    PCL_CHECK(is(&g_recs, 3, 10, PCL_EVT_NOTE_OFF, 64));

    /* a gate longer than the step ending in a rest: the rest cuts the note */
    seq_new(&s, 2, 2, 9, 0, PCL_SEQ_FORWARD);
    step(&s, 0, 60, 100, 0, 0);
    step(&s, 1, 62, 0, 0, 0);
    g_recs.n = 0;
    clocks(&s, 0, 3, &g_recs);
    PCL_CHECK_EQ_INT(g_recs.n, 2);
    PCL_CHECK(is(&g_recs, 1, 2, PCL_EVT_NOTE_OFF, 60));

    /* tie: the note runs on into the next step; the next ON comes with no OFF before it */
    seq_new(&s, 3, 4, 1, 0, PCL_SEQ_FORWARD);
    step(&s, 0, 60, 100, 0, PCL_SEQ_TIE);
    step(&s, 1, 62, 100, 0, 0);
    step(&s, 2, 64, 100, 0, 0);
    g_recs.n = 0;
    clocks(&s, 0, 10, &g_recs);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 1, 4, PCL_EVT_NOTE_ON, 62)); /* no OFF 60 in between: legato */
    PCL_CHECK(is(&g_recs, 2, 5, PCL_EVT_NOTE_OFF, 62));
    PCL_CHECK(is(&g_recs, 3, 8, PCL_EVT_NOTE_ON, 64));
    PCL_CHECK(is(&g_recs, 4, 9, PCL_EVT_NOTE_OFF, 64));

    /* tie into a rest: the note ends at the rest */
    seq_new(&s, 2, 4, 1, 0, PCL_SEQ_FORWARD);
    step(&s, 0, 60, 100, 0, PCL_SEQ_TIE);
    step(&s, 1, 62, 0, 0, 0);
    g_recs.n = 0;
    clocks(&s, 0, 6, &g_recs);
    PCL_CHECK_EQ_INT(g_recs.n, 2);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 1, 4, PCL_EVT_NOTE_OFF, 60));

    /* a tied last step wraps into the first */
    seq_new(&s, 2, 2, 1, 0, PCL_SEQ_FORWARD);
    step(&s, 0, 60, 100, 0, 0);
    step(&s, 1, 67, 100, 0, PCL_SEQ_TIE);
    g_recs.n = 0;
    clocks(&s, 0, 6, &g_recs);
    /* ON 60, OFF 60 (gate 1), ON 67 at 2 (tied), ON 60 at 4 (legato), OFF 60 at 5 */
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 1, 1, PCL_EVT_NOTE_OFF, 60));
    PCL_CHECK(is(&g_recs, 2, 2, PCL_EVT_NOTE_ON, 67));
    PCL_CHECK(is(&g_recs, 3, 4, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 4, 5, PCL_EVT_NOTE_OFF, 60));
    PCL_CHECK_EQ_INT(g_recs.n, 5);
}

static void test_swing(void)
{
    static const uint8_t pat[] = { 60, 62, 64, 65 };
    pcl_seq_t            s;
    unsigned             k;

    seq_new(&s, 4, 6, 1, 2, PCL_SEQ_FORWARD);
    notes(&s, pat, 4);
    g_recs.n = 0;
    clocks(&s, 0, 50, &g_recs);
    /* steps at 0, 6+2, 12, 18+2, 24, 32, 36, 44, 48 */
    {
        static const unsigned at[] = { 0, 8, 12, 20, 24, 32, 36, 44, 48 };

        for (k = 0; k < 9; k++)
            PCL_CHECK(is(&g_recs, 2 * k, at[k], PCL_EVT_NOTE_ON, pat[k % 4]));
    }
    /* max swing: the late step is one pulse short of the next */
    seq_new(&s, 2, 4, 1, 3, PCL_SEQ_FORWARD);
    notes(&s, pat, 2);
    g_recs.n = 0;
    clocks(&s, 0, 14, &g_recs);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 2, 7, PCL_EVT_NOTE_ON, 62));
    PCL_CHECK(is(&g_recs, 4, 8, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 5, 9, PCL_EVT_NOTE_OFF, 60));
    PCL_CHECK_EQ_INT(g_recs.n, 6);
}

static void test_transpose(void)
{
    pcl_seq_t s;
    size_t    i;

    {
        pcl_seq_config_t c = cfg_of(1, 1, 1, 0, PCL_SEQ_FORWARD);

        c.transpose = 2147483647; /* the config is clamped too */
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_OK);
        step(&s, 0, 1, 100, 0, 0);
        g_recs.n = 0;
        clocks(&s, 0, 1, &g_recs);
        PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 127));
    }
    seq_new(&s, 2, 2, 1, 0, PCL_SEQ_FORWARD);
    step(&s, 0, 10, 100, 0, 0);
    step(&s, 1, 120, 100, 0, 0);
    pcl_seq_set_transpose(&s, 7);
    g_recs.n = 0;
    clocks(&s, 0, 4, &g_recs);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 17));
    PCL_CHECK(is(&g_recs, 1, 1, PCL_EVT_NOTE_OFF, 17));
    PCL_CHECK(is(&g_recs, 2, 2, PCL_EVT_NOTE_ON, 127)); /* clamped */
    PCL_CHECK(is(&g_recs, 3, 3, PCL_EVT_NOTE_OFF, 127));
    /* the note released is the note started, even if transpose changes in between */
    seq_new(&s, 1, 4, 3, 0, PCL_SEQ_FORWARD);
    step(&s, 0, 60, 100, 0, 0);
    g_recs.n = 0;
    clocks(&s, 0, 1, &g_recs);
    pcl_seq_set_transpose(&s, 12);
    clocks(&s, 1, 3, &g_recs);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 1, 3, PCL_EVT_NOTE_OFF, 60));
    /* extreme transposes clamp, never wrap */
    pcl_seq_set_transpose(&s, -1000000);
    g_recs.n = 0;
    clocks(&s, 4, 4, &g_recs);
    PCL_CHECK(is(&g_recs, 0, 4, PCL_EVT_NOTE_ON, 0));
    pcl_seq_set_transpose(&s, 2147483647);
    clocks(&s, 8, 4, &g_recs);
    for (i = 0; i < g_recs.n; i++)
        if (g_recs.r[i].type == PCL_EVT_NOTE_ON && g_recs.r[i].pulse == 8)
            PCL_CHECK(g_recs.r[i].note == 127);
}

/* Lowering the division pulls the swing down with it, and the pulled-down swing is the one used. */
static void test_swing_follows_division(void)
{
    static const uint8_t pat[] = { 60, 62 };
    pcl_seq_t            s;

    seq_new(&s, 2, 6, 1, 5, PCL_SEQ_FORWARD);
    notes(&s, pat, 2);
    PCL_CHECK_EQ_INT(pcl_seq_set_division(&s, 4), PCL_OK); /* swing 5 -> 3 */
    g_recs.n = 0;
    clocks(&s, 0, 20, &g_recs);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 2, 7, PCL_EVT_NOTE_ON, 62)); /* 4 + 3 */
    PCL_CHECK(is(&g_recs, 4, 8, PCL_EVT_NOTE_ON, 60));
    /* swing equal to the new division also comes down, to division - 1 */
    seq_new(&s, 2, 6, 1, 3, PCL_SEQ_FORWARD);
    notes(&s, pat, 2);
    PCL_CHECK_EQ_INT(pcl_seq_set_division(&s, 3), PCL_OK);
    g_recs.n = 0;
    clocks(&s, 0, 8, &g_recs);
    PCL_CHECK(is(&g_recs, 2, 5, PCL_EVT_NOTE_ON, 62)); /* 3 + 2 */
}

/* The first step of a random run is drawn from the pattern, and different seeds start differently.
 */
static void test_random_first_step(void)
{
    pcl_seq_t s;
    uint32_t  seed;
    unsigned  seen[8] = { 0 };
    unsigned  distinct = 0;
    size_t    i;
    uint8_t   pat[8];

    for (i = 0; i < 8; i++)
        pat[i] = (uint8_t)(60 + i);
    for (seed = 1; seed <= 200; seed++)
    {
        pcl_seq_config_t c = cfg_of(8, 1, 1, 0, PCL_SEQ_RANDOM);

        c.seed = seed;
        PCL_CHECK_EQ_INT(pcl_seq_init(&s, &c), PCL_OK);
        notes(&s, pat, 8);
        g_recs.n = 0;
        clocks(&s, 0, 1, &g_recs);
        PCL_CHECK(g_recs.n >= 1 && g_recs.r[0].type == PCL_EVT_NOTE_ON);
        if (g_recs.n >= 1 && g_recs.r[0].note >= 60 && g_recs.r[0].note < 68)
            seen[g_recs.r[0].note - 60]++;
        else
            PCL_CHECK(0);
    }
    for (i = 0; i < 8; i++)
        distinct += seen[i] > 0;
    PCL_CHECK(distinct >= 7);
}

/* Shortening the loop to or below the position: the next step is the start of the loop. */
static void test_shorten_edges(void)
{
    static const uint8_t pat[] = { 60, 62, 64, 65, 67, 69 };
    pcl_seq_t            s;
    size_t               i;
    uint8_t              got[8];
    size_t               n;

    /* forward, position exactly at the new length */
    seq_new(&s, 6, 1, 1, 0, PCL_SEQ_FORWARD);
    notes(&s, pat, 6);
    g_recs.n = 0;
    clocks(&s, 0, 4, &g_recs); /* 60 62 64 65: position 3 */
    PCL_CHECK_EQ_INT(pcl_seq_set_length(&s, 3), PCL_OK);
    g_recs.n = 0;
    clocks(&s, 4, 3, &g_recs);
    for (i = 0, n = 0; i < g_recs.n; i++)
        if (g_recs.r[i].type == PCL_EVT_NOTE_ON)
            got[n++] = g_recs.r[i].note;
    PCL_CHECK(n == 3 && got[0] == 60 && got[1] == 62 && got[2] == 64);

    /* backward: from the new last step */
    seq_new(&s, 6, 1, 1, 0, PCL_SEQ_BACKWARD);
    notes(&s, pat, 6);
    g_recs.n = 0;
    clocks(&s, 0, 3, &g_recs); /* 69 67 65: position 3 */
    PCL_CHECK_EQ_INT(pcl_seq_set_length(&s, 3), PCL_OK);
    g_recs.n = 0;
    clocks(&s, 3, 3, &g_recs);
    for (i = 0, n = 0; i < g_recs.n; i++)
        if (g_recs.r[i].type == PCL_EVT_NOTE_ON)
            got[n++] = g_recs.r[i].note;
    PCL_CHECK(n == 3 && got[0] == 64 && got[1] == 62 && got[2] == 60);

    /* ping-pong: stays inside the new loop */
    seq_new(&s, 6, 1, 1, 0, PCL_SEQ_PINGPONG);
    notes(&s, pat, 6);
    g_recs.n = 0;
    clocks(&s, 0, 6, &g_recs); /* up to position 5 */
    PCL_CHECK_EQ_INT(pcl_seq_set_length(&s, 3), PCL_OK);
    g_recs.n = 0;
    clocks(&s, 6, 8, &g_recs);
    for (i = 0; i < g_recs.n; i++)
        if (g_recs.r[i].type == PCL_EVT_NOTE_ON)
            PCL_CHECK(g_recs.r[i].note <= 64);
}

static void send(pcl_seq_t *s, uint8_t type, uint16_t offset, recs_t *l)
{
    pcl_event_t in, out[4];
    size_t      n, i;

    in.offset = offset;
    in.type = type;
    in.note = 0;
    in.velocity = 0;
    n = pcl_seq_process(s, &in, 1, out, 4);
    for (i = 0; i < n; i++)
        add_rec(l, &out[i], 0);
}

static void test_transport(void)
{
    static const uint8_t pat[] = { 60, 62, 64, 65 };
    pcl_seq_t            s;

    seq_new(&s, 4, 4, 3, 0, PCL_SEQ_FORWARD);
    notes(&s, pat, 4);
    g_recs.n = 0;
    clocks(&s, 0, 6, &g_recs); /* ON 60 at 0, OFF at 3, ON 62 at 4 (sounding) */
    PCL_CHECK_EQ_INT(g_recs.n, 3);
    send(&s, PCL_EVT_STOP, 30, &g_recs);
    PCL_CHECK_EQ_INT(g_recs.n, 4);
    PCL_CHECK(
        g_recs.r[3].type == PCL_EVT_NOTE_OFF && g_recs.r[3].note == 62 && g_recs.r[3].pulse == 3);
    send(&s, PCL_EVT_STOP, 30, &g_recs); /* nothing sounds: nothing said */
    PCL_CHECK_EQ_INT(g_recs.n, 4);

    /* START: back to the first step, pulse 0 plays it */
    send(&s, PCL_EVT_START, 0, &g_recs);
    PCL_CHECK_EQ_INT(g_recs.n, 4);
    g_recs.n = 0;
    clocks(&s, 0, 6, &g_recs);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 60));
    PCL_CHECK(is(&g_recs, 2, 4, PCL_EVT_NOTE_ON, 62));

    /* START while a note sounds releases it first */
    g_recs.n = 0;
    send(&s, PCL_EVT_START, 20, &g_recs);
    PCL_CHECK_EQ_INT(g_recs.n, 1);
    PCL_CHECK(g_recs.r[0].type == PCL_EVT_NOTE_OFF && g_recs.r[0].note == 62);
    PCL_CHECK_EQ_INT(pcl_seq_position(&s), 0);

    /* backward restarts at the last step */
    PCL_CHECK_EQ_INT(pcl_seq_set_direction(&s, PCL_SEQ_BACKWARD), PCL_OK);
    send(&s, PCL_EVT_START, 0, &g_recs);
    g_recs.n = 0;
    clocks(&s, 0, 1, &g_recs);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 65));

    /* reset rewinds without events and keeps the pattern */
    pcl_seq_reset(&s);
    g_recs.n = 0;
    clocks(&s, 0, 1, &g_recs);
    PCL_CHECK(is(&g_recs, 0, 0, PCL_EVT_NOTE_ON, 65));
    {
        pcl_event_t ign[3];
        pcl_event_t out[4];

        ign[0].offset = 0, ign[0].type = PCL_EVT_NOTE_ON, ign[0].note = 90, ign[0].velocity = 100;
        ign[1].offset = 1, ign[1].type = PCL_EVT_NOTE_OFF, ign[1].note = 90, ign[1].velocity = 0;
        ign[2].offset = 2, ign[2].type = 0xF0, ign[2].note = 0, ign[2].velocity = 0;
        PCL_CHECK_EQ_INT(pcl_seq_process(&s, ign, 3, out, 4), 0);
        PCL_CHECK_EQ_INT(pcl_seq_process(&s, NULL, 0, out, 4), 0);
    }
}

static void test_live_changes(void)
{
    static const uint8_t pat[] = { 60, 62, 64, 65, 67, 69 };
    pcl_seq_t            s;
    size_t               i;

    /* shorten the loop below the current position: wraps to the start, no out-of-range step */
    seq_new(&s, 6, 1, 1, 0, PCL_SEQ_FORWARD);
    notes(&s, pat, 6);
    g_recs.n = 0;
    clocks(&s, 0, 5, &g_recs); /* position 4 */
    PCL_CHECK_EQ_INT(pcl_seq_set_length(&s, 3), PCL_OK);
    clocks(&s, 5, 6, &g_recs);
    for (i = 0; i < g_recs.n; i++)
        PCL_CHECK(g_recs.r[i].note <= 69);
    PCL_CHECK(pcl_seq_position(&s) < 3);
    {
        uint8_t last[3];
        size_t  n = 0;

        for (i = 0; i < g_recs.n; i++)
            if (g_recs.r[i].pulse >= 5 && g_recs.r[i].type == PCL_EVT_NOTE_ON && n < 3)
                last[n++] = g_recs.r[i].note;
        PCL_CHECK(n == 3 && last[0] == 60 && last[1] == 62 && last[2] == 64);
    }

    /* division change takes effect from the next step */
    seq_new(&s, 2, 4, 1, 0, PCL_SEQ_FORWARD);
    notes(&s, pat, 2);
    g_recs.n = 0;
    clocks(&s, 0, 2, &g_recs); /* step 0 at 0; next due at 4 */
    PCL_CHECK_EQ_INT(pcl_seq_set_division(&s, 2), PCL_OK);
    clocks(&s, 2, 8, &g_recs);
    {
        static const unsigned at[] = { 0, 4, 6, 8 };
        size_t                n = 0;

        for (i = 0; i < g_recs.n; i++)
            if (g_recs.r[i].type == PCL_EVT_NOTE_ON)
            {
                if (n < 4)
                    PCL_CHECK_EQ_INT(g_recs.r[i].pulse, at[n]);
                n++;
            }
        PCL_CHECK(n >= 4);
    }
}

/* The counters wrap at 2^32; the signed differences must keep a run identical across the wrap. */
static void test_pulse_counter_wrap(void)
{
    static const uint8_t pat[] = { 60, 62, 64, 65, 67 };
    pcl_seq_t            a, b;
    static recs_t        ra, rb;
    size_t               i;
    const uint32_t       start = 0xFFFFFFF0u;

    seq_new(&a, 5, 3, 2, 1, PCL_SEQ_FORWARD);
    notes(&a, pat, 5);
    b = a;
    /* shift every pulse counter of b so that it crosses 2^32 after 16 pulses */
    b.pulse = start;
    b.next_base = start;
    b.off_at = start;
    ra.n = 0;
    rb.n = 0;
    clocks(&a, 0, 100, &ra);
    clocks(&b, 0, 100, &rb);
    PCL_CHECK(ra.n > 40);
    PCL_CHECK_EQ_INT(ra.n, rb.n);
    PCL_CHECK(memcmp(ra.r, rb.r, ra.n * sizeof(rec_t)) == 0);
    PCL_CHECK(b.pulse < 100u); /* it did wrap */
    (void)i;
}

static void test_capacity(void)
{
    static const uint8_t pat[] = { 60, 62 };
    pcl_seq_t            a, b;
    pcl_event_t          in[8], full[16], part[16];
    size_t               i, nf, np;

    seq_new(&a, 2, 1, 1, 0, PCL_SEQ_FORWARD);
    seq_new(&b, 2, 1, 1, 0, PCL_SEQ_FORWARD);
    notes(&a, pat, 2);
    notes(&b, pat, 2);
    for (i = 0; i < 8; i++)
    {
        in[i].offset = (uint16_t)i;
        in[i].type = PCL_EVT_CLOCK;
        in[i].note = 0;
        in[i].velocity = 0;
    }
    nf = pcl_seq_process(&a, in, 8, full, 16);
    PCL_CHECK_EQ_INT(nf, 15); /* ON at pulse 0, then OFF+ON per pulse (division 1) */
    PCL_CHECK_EQ_INT(pcl_seq_dropped(&a), 0);
    np = pcl_seq_process(&b, in, 8, part, 5);
    PCL_CHECK_EQ_INT(np, 5);
    PCL_CHECK_EQ_INT(pcl_seq_dropped(&b), 10);
    for (i = 0; i < np; i++)
        PCL_CHECK(part[i].offset == full[i].offset && part[i].type == full[i].type
            && part[i].note == full[i].note);
    PCL_CHECK_EQ_INT(pcl_seq_process(&b, in, 8, NULL, 0), 0);
    PCL_CHECK_EQ_INT(pcl_seq_dropped(&b), 26); /* 10, then 8 pulses of OFF + ON */
}

/* Independent model for untied patterns: step n starts at n * division (+ swing when n is odd) and
   is released at min(start + gate, start of step n + 1). */
static void model_run(const pcl_seq_step_t *pat, unsigned length, unsigned division, unsigned gate,
    unsigned swing, int backward, unsigned pulses, recs_t *out)
{
    unsigned n;

    out->n = 0;
    for (n = 0;; n++)
    {
        const unsigned        t = n * division + ((n & 1u) ? swing : 0u);
        const unsigned        t_next = (n + 1u) * division + (((n + 1u) & 1u) ? swing : 0u);
        const unsigned        idx = backward ? length - 1u - n % length : n % length;
        const pcl_seq_step_t *st = &pat[idx];
        unsigned              g, off;

        if (t >= pulses)
            break;
        if (st->velocity == 0)
            continue;
        g = st->gate ? st->gate : gate;
        off = t + g < t_next ? t + g : t_next;
        /* OFF is listed before the ON that starts at the same pulse: handled by the sort below */
        out->r[out->n].pulse = t, out->r[out->n].type = PCL_EVT_NOTE_ON;
        out->r[out->n].note = st->note, out->r[out->n].vel = st->velocity;
        out->n++;
        if (off < pulses)
        {
            out->r[out->n].pulse = off, out->r[out->n].type = PCL_EVT_NOTE_OFF;
            out->r[out->n].note = st->note, out->r[out->n].vel = 0;
            out->n++;
        }
    }
    /* stable sort by pulse, OFF before ON at one pulse */
    {
        size_t i, j;

        for (i = 1; i < out->n; i++)
        {
            const rec_t key = out->r[i];

            for (j = i; j > 0
                && (out->r[j - 1].pulse > key.pulse
                    || (out->r[j - 1].pulse == key.pulse && out->r[j - 1].type == PCL_EVT_NOTE_ON
                        && key.type == PCL_EVT_NOTE_OFF));
                j--)
                out->r[j] = out->r[j - 1];
            out->r[j] = key;
        }
    }
}

static void test_reference_model(void)
{
    static recs_t want;
    int           trial;

    g_rng = 99u;

    for (trial = 0; trial < 300; trial++)
    {
        pcl_seq_t      s;
        pcl_seq_step_t pat[PCL_SEQ_MAX_STEPS];
        unsigned       length, division, gate, swing, i;
        int            backward;

        length = 1 + rnd(12);
        division = 1 + rnd(12);
        gate = 1 + rnd(16);
        swing = rnd(division);
        backward = (int)rnd(2);
        seq_new(&s, length, division, gate, swing, backward ? PCL_SEQ_BACKWARD : PCL_SEQ_FORWARD);
        for (i = 0; i < length; i++)
        {
            const unsigned note = 30 + rnd(80);
            const unsigned vel = rnd(4) == 0 ? 0 : 1 + rnd(127);
            const unsigned g = rnd(3) == 0 ? 1 + rnd(20) : 0;

            pat[i].note = (uint8_t)note;
            pat[i].velocity = (uint8_t)vel;
            pat[i].gate = (uint8_t)g;
            pat[i].flags = 0;
            PCL_CHECK_EQ_INT(pcl_seq_set_step(&s, i, &pat[i]), PCL_OK);
        }
        g_recs.n = 0;
        clocks(&s, 0, 150, &g_recs);
        model_run(pat, length, division, gate, swing, backward, 150, &want);
        PCL_CHECK_EQ_INT(g_recs.n, want.n);
        if (g_recs.n == want.n)
            PCL_CHECK(memcmp(g_recs.r, want.r, want.n * sizeof(rec_t)) == 0);
    }
}

/* Random patterns, ties, transport and settings changes: the output is sorted, consistent with a
   monophonic voice (a note released was the one sounding), and nothing hangs after STOP. */
static void test_property_stream(void)
{
    int trial;

    g_rng = 4242u;

    for (trial = 0; trial < 60; trial++)
    {
        pcl_seq_t s;
        int       voice = -1; /* the note sounding in the monophonic model */
        unsigned  pulse = 0;
        unsigned  blk, i;

        {
            const unsigned len = 1 + rnd(16);
            const unsigned div = 1 + rnd(8);
            const unsigned gate = 1 + rnd(10);
            const unsigned dir = rnd(4);

            seq_new(&s, len, div, gate, 0, (pcl_seq_direction_t)dir);
        }
        for (i = 0; i < PCL_SEQ_MAX_STEPS; i++)
        {
            const unsigned note = rnd(128);
            const unsigned vel = rnd(4) == 0 ? 0 : 1 + rnd(127);
            const unsigned gate = rnd(3) == 0 ? 1 + rnd(30) : 0;
            const unsigned tie = rnd(3) == 0;

            step(&s, i, (uint8_t)note, (uint8_t)vel, (uint8_t)gate, tie ? PCL_SEQ_TIE : 0);
        }
        for (blk = 0; blk < 40; blk++)
        {
            pcl_event_t in[64], out[200];
            size_t      n, j, k = 0;
            uint16_t    off = 0;

            for (j = 0; j < 40; j++)
            {
                off = (uint16_t)(off + rnd(50));
                in[k].offset = off;
                in[k].note = 0;
                in[k].velocity = 0;
                switch (rnd(24))
                {
                case 0:
                    in[k].type = PCL_EVT_START;
                    break;
                case 1:
                    in[k].type = PCL_EVT_STOP;
                    break;
                default:
                    in[k].type = PCL_EVT_CLOCK;
                    break;
                }
                k++;
            }
            if (rnd(5) == 0)
                (void)pcl_seq_set_length(&s, 1 + rnd(PCL_SEQ_MAX_STEPS));
            if (rnd(5) == 0)
                (void)pcl_seq_set_division(&s, 1 + rnd(12));
            if (rnd(5) == 0)
                (void)pcl_seq_set_swing(&s, rnd(4));
            if (rnd(5) == 0)
                pcl_seq_set_transpose(&s, (int)rnd(60) - 30);
            n = pcl_seq_process(&s, in, k, out, 200);
            PCL_CHECK(n <= 80);
            for (j = 0; j < n; j++)
            {
                if (j > 0)
                    PCL_CHECK(out[j].offset >= out[j - 1].offset);
                PCL_CHECK(out[j].note < 128);
                if (out[j].type == PCL_EVT_NOTE_ON)
                {
                    PCL_CHECK(out[j].velocity > 0);
                    voice = out[j].note; /* mono: the newest note wins, tied or not */
                }
                else
                {
                    PCL_CHECK(out[j].type == PCL_EVT_NOTE_OFF);
                    PCL_CHECK_EQ_INT(out[j].note, voice); /* it releases the note that sounds */
                    voice = -1;
                }
            }
            pulse++;
        }
        {
            pcl_event_t  stop, out[2];
            const size_t n = (stop.offset = 0, stop.type = PCL_EVT_STOP, stop.note = 0,
                stop.velocity = 0, pcl_seq_process(&s, &stop, 1, out, 2));

            PCL_CHECK_EQ_INT(n, voice >= 0 ? 1 : 0);
        }
        PCL_CHECK_EQ_INT(pcl_seq_dropped(&s), 0);
        (void)pulse;
    }
}

int main(void)
{
    PCL_RUN_TEST(test_init_and_setters);
    PCL_RUN_TEST(test_all_rests_is_silent);
    PCL_RUN_TEST(test_forward_timing);
    PCL_RUN_TEST(test_directions);
    PCL_RUN_TEST(test_random_direction);
    PCL_RUN_TEST(test_rests_and_ties);
    PCL_RUN_TEST(test_swing);
    PCL_RUN_TEST(test_transpose);
    PCL_RUN_TEST(test_swing_follows_division);
    PCL_RUN_TEST(test_random_first_step);
    PCL_RUN_TEST(test_shorten_edges);
    PCL_RUN_TEST(test_transport);
    PCL_RUN_TEST(test_live_changes);
    PCL_RUN_TEST(test_pulse_counter_wrap);
    PCL_RUN_TEST(test_capacity);
    PCL_RUN_TEST(test_reference_model);
    PCL_RUN_TEST(test_property_stream);
    return pcl_test_summary("seq");
}
