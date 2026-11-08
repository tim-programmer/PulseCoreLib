/*
 * Event modules wired together: clock -> sequencer / arpeggiator -> ADSR. The envelope must rise
 * and fall at the sample positions the clock arithmetic predicts (to one control period), and the
 * result must not depend on how the stream is cut into blocks.
 */
#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

#define FS 48000u
#define SPP 1000u /* samples per pulse: 120 bpm, 24 ppqn at 48 kHz */
#define DIV 6u    /* pulses per step                        */
#define GATE 3u   /* pulses per note                        */
#define STEP (SPP * DIV)
#define TOTAL (STEP * 8u)
#define ONSET_TOL (2u * PCL_CONTROL_DIV)

typedef enum
{
    VIA_SEQ,
    VIA_SEQ_TIES, /* steps 0 and 1 tied */
    VIA_ARP
} source_t;

static pcl_adsr_config_t adsr_cfg(void)
{
    pcl_adsr_config_t c;

    c.sample_rate_hz = FS;
    c.attack_ms = 2;
    c.decay_ms = 10;
    c.release_ms = 5;
    c.sustain = PCL_COEF(1.0);
    c.attack_curve = c.decay_curve = c.release_curve = PCL_COEF(0.0);
    c.run_through = 0;
    return c;
}

/* Renders TOTAL samples of envelope with blocks of the sizes in `sizes` (repeated). */
static void render(source_t src, const size_t *sizes, size_t nsizes, double *out)
{
    pcl_clock_config_t      cc;
    pcl_clock_t             clk;
    pcl_seq_t               seq;
    pcl_arp_t               arp;
    pcl_adsr_t              adsr;
    const pcl_adsr_config_t ac = adsr_cfg();
    size_t                  done = 0, bi = 0;
    int                     first = 1;

    cc.sample_rate_hz = FS;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cc.bpm = 120.0f;
#else
    cc.bpm = PCL_HZ(120.0);
#endif
    cc.ppqn = 24;
    cc.running = 1;
    PCL_CHECK_EQ_INT(pcl_clock_init(&clk, &cc), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_adsr_init(&adsr, &ac), PCL_OK);
    if (src != VIA_ARP)
    {
        pcl_seq_config_t sc;
        size_t           i;

        sc.length = 4;
        sc.division = DIV;
        sc.gate = GATE;
        sc.swing = 0;
        sc.direction = PCL_SEQ_FORWARD;
        sc.transpose = 0;
        sc.seed = 0;
        PCL_CHECK_EQ_INT(pcl_seq_init(&seq, &sc), PCL_OK);
        for (i = 0; i < 4; i++)
        {
            pcl_seq_step_t st;

            st.note = (uint8_t)(60 + 2 * i);
            st.velocity = 100;
            st.gate = 0;
            st.flags = (src == VIA_SEQ_TIES && i < 2) ? PCL_SEQ_TIE : 0;
            PCL_CHECK_EQ_INT(pcl_seq_set_step(&seq, i, &st), PCL_OK);
        }
    }
    else
    {
        pcl_arp_config_t ac2;

        ac2.mode = PCL_ARP_UP;
        ac2.octaves = 1;
        ac2.division = DIV;
        ac2.gate = GATE;
        ac2.latch = 0;
        ac2.seed = 0;
        PCL_CHECK_EQ_INT(pcl_arp_init(&arp, &ac2), PCL_OK);
    }
    while (done < TOTAL)
    {
        size_t       frames = sizes[bi++ % nsizes];
        pcl_event_t  cev[64], nev[160], kev[8];
        pcl_sample_t buf[512];
        size_t       ncev, nnev = 0, i;

        if (frames > TOTAL - done)
            frames = TOTAL - done;
        if (frames > 512)
            frames = 512;
        ncev = pcl_clock_process(&clk, frames, cev, 64);
        if (src != VIA_ARP)
        {
            nnev = pcl_seq_process(&seq, cev, ncev, nev, 160);
        }
        else
        {
            /* a key held from the first sample; released at sample STEP * 5 + 100 */
            pcl_event_t merged[80];
            size_t      nk = 0, nm = 0, a = 0, b;

            if (first)
            {
                kev[nk].offset = 0, kev[nk].type = PCL_EVT_NOTE_ON, kev[nk].note = 60;
                kev[nk].velocity = 100, nk++;
            }
            if (done <= STEP * 5 + 100 && done + frames > STEP * 5 + 100)
            {
                kev[nk].offset = (uint16_t)(STEP * 5 + 100 - done), kev[nk].type = PCL_EVT_NOTE_OFF;
                kev[nk].note = 60, kev[nk].velocity = 0, nk++;
            }
            /* merge by offset; at one offset the keyboard's events go first, so a key pressed on
               the very sample of a pulse counts for that pulse (and a key released there does not).
               The caller picks this order: the producer order in pcl_event.h is only for events
               made by one module. */
            for (b = 0; a < ncev || b < nk;)
            {
                const int take_clock = b >= nk || (a < ncev && cev[a].offset < kev[b].offset);

                merged[nm++] = take_clock ? cev[a++] : kev[b++];
            }
            nnev = pcl_arp_process(&arp, merged, nm, nev, 160);
        }
        first = 0;
        pcl_adsr_process(&adsr, buf, NULL, nev, nnev, frames);
        for (i = 0; i < frames; i++)
            out[done + i] = pcl_test_to_double(buf[i]);
        done += frames;
    }
}

/* First sample in [from, to) at which the envelope exceeds `level`, or `to` if none. */
static size_t first_above(const double *e, size_t from, size_t to, double level)
{
    size_t i;

    for (i = from; i < to; i++)
        if (e[i] > level)
            return i;
    return to;
}

static void check_pattern(const double *e, unsigned nsteps)
{
    unsigned n;

    for (n = 0; n < nsteps; n++)
    {
        const size_t start = (size_t)STEP * n;
        const size_t off = start + (size_t)SPP * GATE;
        const size_t on = first_above(e, start, start + STEP, 0.01);

        /* the note starts at its pulse, to within a couple of control periods */
        PCL_CHECK(on >= start && on <= start + ONSET_TOL);
        /* full level well before the gate ends */
        PCL_CHECK(e[start + 1500] > 0.9);
        PCL_CHECK(e[off - 1] > 0.9);
        /* released at the gate: gone 5 ms (240 samples) + a control period later */
        PCL_CHECK(e[off + 240 + PCL_CONTROL_DIV] < 0.02);
        /* silent until the next step */
        PCL_CHECK(e[start + STEP - 1] < 0.02);
    }
}

static const size_t k256[] = { 256 };
static const size_t kodd[] = { 1, 7, 97, 255, 300, 13, 512, 64 };

static void test_sequencer_into_adsr(void)
{
    static double a[TOTAL], b[TOTAL];

    render(VIA_SEQ, k256, 1, a);
    check_pattern(a, 8);
    render(VIA_SEQ, kodd, sizeof kodd / sizeof kodd[0], b);
    PCL_CHECK(memcmp(a, b, sizeof a) == 0); /* block split does not matter */
}

/* A tied step runs into the next one: the envelope neither releases nor restarts in between. Steps
   0 and 1 are tied, so steps 0, 1 and 2 are one note, released at the gate of step 2. */
static void test_ties_do_not_retrigger(void)
{
    static double a[TOTAL];
    size_t        i, low = 0;

    render(VIA_SEQ_TIES, k256, 1, a);
    for (i = 600; i < (size_t)2 * STEP + SPP * GATE; i++)
        low += a[i] < 0.9;
    PCL_CHECK_EQ_INT(low, 0); /* no dip at the step edges */
    PCL_CHECK(a[2 * STEP + SPP * GATE + 240 + PCL_CONTROL_DIV] < 0.02);
    PCL_CHECK(a[3 * STEP - 1] < 0.02);
    /* step 3 is an ordinary note again */
    PCL_CHECK(first_above(a, 3 * STEP, 4 * STEP, 0.01) <= 3 * STEP + ONSET_TOL);
    PCL_CHECK(a[3 * STEP + SPP * GATE - 1] > 0.9);
}

static void test_arpeggiator_into_adsr(void)
{
    static double a[TOTAL], b[TOTAL];

    render(VIA_ARP, k256, 1, a);
    /* the held key sounds on steps 0 .. 4 and is released at STEP * 5 + 100 (inside step 5's
       first pulses, before its note starts?) - step 5 starts at STEP * 5, a note is playing there
       and is cut at +100 samples by the key-up */
    check_pattern(a, 5);
    PCL_CHECK(first_above(a, STEP * 5, STEP * 5 + 100, 0.01) <= STEP * 5 + ONSET_TOL);
    PCL_CHECK(a[STEP * 5 + 100 + 240 + PCL_CONTROL_DIV] < 0.02);     /* the key-up released it */
    PCL_CHECK(first_above(a, STEP * 5 + 400, TOTAL, 0.01) == TOTAL); /* and nothing after */
    render(VIA_ARP, kodd, sizeof kodd / sizeof kodd[0], b);
    PCL_CHECK(memcmp(a, b, sizeof a) == 0);
}

int main(void)
{
    PCL_RUN_TEST(test_sequencer_into_adsr);
    PCL_RUN_TEST(test_ties_do_not_retrigger);
    PCL_RUN_TEST(test_arpeggiator_into_adsr);
    return pcl_test_summary("event_chain");
}
