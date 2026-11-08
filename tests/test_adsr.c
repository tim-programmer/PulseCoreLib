/*
 * Tests for the ADSR envelope (include/pcl/pcl_adsr.h).
 *
 * The main check is an independent double-precision model of the algorithm written here from the
 * design note: gate edges seen in one control period are applied at the next control point, a stage
 * lasts a whole number of periods and lands exactly on its end level, and the output is a linear
 * ramp between control points. Everything else is a property check (timing, monotonicity, steps,
 * latching).
 */
#include <stdlib.h>
#include <string.h>

#include "pcl_test.h"

#define SAMPLE_RATE_HZ 48000u
#define DIV ((size_t)PCL_CONTROL_DIV)

/* The largest level the module can output in this build, as a fraction of full scale. */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define FULL_D 1.0
#define TOL_BASE 3.0e-6
#elif PCL_SAMPLE_TYPE == PCL_Q15
#define FULL_D (32767.0 / 32768.0)
#define TOL_BASE 7.0e-5 /* 2 LSB: the output shift truncates */
#else
#define FULL_D (2147483646.0 / 2147483648.0)
#define TOL_BASE 5.0e-8
#endif

/* ---- Independent model ---- */

typedef struct
{
    uint32_t fs;
    uint32_t ms[3]; /* attack, decay, release */
    double   curve[3];
    double   sus;
    int      run_through;
} mcfg_t;

typedef struct
{
    long at; /* sample the edge is seen at; < 0 = before the stream (applied at the first control
                point) */
    int on;
} medge_t;

/* Normalised decay curve; below 1/1024 it is exactly linear (the module's documented behaviour). */
static double model_decay(double curve, double x)
{
    if (curve < 1.0 / 1024.0)
        return 1.0 - x;
    {
        const double m = 16.0 * curve;
        return (pow(2.0, -m * x) - pow(2.0, -m)) / (1.0 - pow(2.0, -m));
    }
}

static long model_ticks(uint32_t ms, uint32_t fs)
{
    const long n = (long)floor((double)ms * (double)fs / (1000.0 * (double)DIV) + 0.5);
    return n > 0 ? n : 1;
}

/* Table error of the exponential (docs/modules/adsr.md), as an absolute tolerance for this curve.
 */
static double curve_tol(double curve)
{
    if (curve < 1.0 / 1024.0)
        return 0.0;
    return 2.5e-5 / (16.0 * curve) + 2.0e-5;
}

enum
{
    M_IDLE = 0,
    M_ATT,
    M_DEC,
    M_SUS,
    M_REL
};

/* Runs the model for n samples. stage_log (optional) receives the stage of every control period. */
static void model_adsr(
    const mcfg_t *c, const medge_t *edges, size_t ne, size_t n, double *out, int *stage_log)
{
    const double sus = c->sus < 1.0 ? c->sus : 1.0;
    int          stage = M_IDLE, gate = 0, rose = 0, fell = 0, pending = 0;
    double       v0 = 0.0, start = 0.0;
    long         left = 0, total = 0;
    size_t       ei = 0, tick, i;

    for (tick = 0; tick * DIV < n; tick++)
    {
        double v1 = 0.0;
        int    last = 0;

        while (ei < ne && edges[ei].at < (long)(tick * DIV))
        {
            if (edges[ei].on && !gate)
            {
                gate = 1;
                rose = 1;
            }
            else if (!edges[ei].on && gate)
            {
                gate = 0;
                fell = 1;
            }
            ei++;
        }

#define ENTER(s)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        stage = (s);                                                                               \
        start = v0;                                                                                \
        total = model_ticks(c->ms[(s) == M_ATT ? 0 : ((s) == M_DEC ? 1 : 2)], c->fs);              \
        left = total;                                                                              \
    } while (0)

        if ((stage == M_ATT || stage == M_DEC || stage == M_REL) && left == 0)
        {
            if (stage == M_ATT)
            {
                if (sus >= 1.0)
                {
                    stage = M_SUS;
                    if (pending)
                        ENTER(M_REL);
                }
                else
                {
                    ENTER(M_DEC);
                }
            }
            else if (stage == M_DEC)
            {
                stage = M_SUS;
                if (pending)
                    ENTER(M_REL);
            }
            else
            {
                stage = M_IDLE;
            }
        }
        if (rose)
        {
            pending = 0;
            if (stage == M_IDLE || stage == M_REL)
                ENTER(M_ATT);
        }
        if (!gate && fell)
        {
            if (stage == M_ATT || stage == M_DEC)
            {
                if (c->run_through)
                    pending = 1;
                else
                    ENTER(M_REL);
            }
            else if (stage == M_SUS)
            {
                ENTER(M_REL);
            }
        }
        rose = fell = 0;

        if (stage == M_ATT || stage == M_DEC || stage == M_REL)
        {
            left--;
            last = left == 0;
        }
        {
            const double x = (double)(total - left) / (double)total;
            switch (stage)
            {
            case M_ATT:
                v1 = last ? 1.0 : start + (1.0 - start) * (1.0 - model_decay(c->curve[0], x));
                break;
            case M_DEC:
                v1 = last ? sus : sus + (start - sus) * model_decay(c->curve[1], x);
                break;
            case M_SUS:
                v1 = sus;
                break;
            case M_REL:
                v1 = last ? 0.0 : start * model_decay(c->curve[2], x);
                break;
            default:
                v1 = 0.0;
                break;
            }
        }
        if (stage_log != NULL)
            stage_log[tick] = stage;
        for (i = 0; i < DIV && tick * DIV + i < n; i++)
            out[tick * DIV + i] = v0 + (v1 - v0) * (double)i / (double)DIV;
        v0 = v1;
    }
}

/* ---- Helpers ---- */

#define N_MAX 60000u
static pcl_sample_t g_a[N_MAX], g_b[N_MAX];
static double       g_model[N_MAX];
static pcl_sample_t g_gate[N_MAX];

static void make_adsr(pcl_adsr_t *a, const mcfg_t *c)
{
    pcl_adsr_config_t cfg;
    cfg.sample_rate_hz = c->fs;
    cfg.attack_ms = c->ms[0];
    cfg.decay_ms = c->ms[1];
    cfg.release_ms = c->ms[2];
    cfg.sustain = PCL_COEF(c->sus);
    cfg.attack_curve = PCL_COEF(c->curve[0]);
    cfg.decay_curve = PCL_COEF(c->curve[1]);
    cfg.release_curve = PCL_COEF(c->curve[2]);
    cfg.run_through = c->run_through;
    PCL_CHECK_EQ_INT(pcl_adsr_init(a, &cfg), PCL_OK);
}

static mcfg_t make_cfg(uint32_t a_ms, uint32_t d_ms, double sus, uint32_t r_ms, double ac,
    double dc, double rc, int run_through)
{
    mcfg_t c;
    c.fs = SAMPLE_RATE_HZ;
    c.ms[0] = a_ms;
    c.ms[1] = d_ms;
    c.ms[2] = r_ms;
    c.sus = pcl_test_coef_to_double(PCL_COEF(sus)); /* the value the module actually holds */
    c.curve[0] = pcl_test_coef_to_double(PCL_COEF(ac));
    c.curve[1] = pcl_test_coef_to_double(PCL_COEF(dc));
    c.curve[2] = pcl_test_coef_to_double(PCL_COEF(rc));
    c.run_through = run_through;
    return c;
}

/* Builds the events of the block [start, start+len) from the absolute edge list. */
static size_t events_for_block(
    const medge_t *edges, size_t ne, size_t start, size_t len, pcl_event_t *ev)
{
    size_t i, n = 0;
    for (i = 0; i < ne; i++)
    {
        if (edges[i].at >= (long)start && edges[i].at < (long)(start + len))
        {
            ev[n].offset = (uint16_t)(edges[i].at - (long)start);
            ev[n].type = edges[i].on ? PCL_EVT_NOTE_ON : PCL_EVT_NOTE_OFF;
            ev[n].note = 60;
            ev[n].velocity = 100;
            n++;
        }
    }
    return n;
}

/* Runs the module over n samples in blocks, driving the gate from the edge list as events. */
static void run_events(
    pcl_adsr_t *a, pcl_sample_t *out, size_t n, size_t block, const medge_t *edges, size_t ne)
{
    size_t done = 0;
    while (done < n)
    {
        pcl_event_t  ev[64];
        const size_t len = n - done < block ? n - done : block;
        const size_t cnt = events_for_block(edges, ne, done, len, ev);
        pcl_adsr_process(a, out + done, NULL, ev, cnt, len);
        done += len;
    }
}

static double max_err(const pcl_sample_t *out, const double *model, size_t n, size_t *where)
{
    double worst = 0.0;
    size_t i;
    for (i = 0; i < n; i++)
    {
        const double d = fabs(pcl_test_to_double(out[i]) - model[i]);
        if (d > worst)
        {
            worst = d;
            if (where != NULL)
                *where = i;
        }
    }
    return worst;
}

/* Largest step between consecutive samples. */
static double max_step(const pcl_sample_t *out, size_t n)
{
    double worst = 0.0;
    size_t i;
    for (i = 1; i < n; i++)
    {
        const double d = fabs(pcl_test_to_double(out[i]) - pcl_test_to_double(out[i - 1]));
        if (d > worst)
            worst = d;
    }
    return worst;
}

/* ---- Validation ---- */

static void test_init_validation(void)
{
    pcl_adsr_t        a, before;
    pcl_adsr_config_t cfg;

    cfg.sample_rate_hz = SAMPLE_RATE_HZ;
    cfg.attack_ms = 10;
    cfg.decay_ms = 20;
    cfg.release_ms = 30;
    cfg.sustain = PCL_COEF(0.5);
    cfg.attack_curve = PCL_COEF(0.0);
    cfg.decay_curve = PCL_COEF(0.5);
    cfg.release_curve = PCL_COEF(1.0);
    cfg.run_through = 0;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_adsr_init(NULL, &cfg), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, NULL), PCL_ERR_NULL_POINTER);

    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ - 1u;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ + 1u;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MIN_HZ;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_OK);
    cfg.sample_rate_hz = PCL_SAMPLE_RATE_MAX_HZ;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_OK);
    cfg.sample_rate_hz = SAMPLE_RATE_HZ;

    cfg.attack_ms = PCL_ADSR_TIME_MAX_MS;
    cfg.decay_ms = 0;
    cfg.release_ms = PCL_ADSR_TIME_MAX_MS;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_OK);
    cfg.attack_ms = PCL_ADSR_TIME_MAX_MS + 1u;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.attack_ms = 10;
    cfg.decay_ms = PCL_ADSR_TIME_MAX_MS + 1u;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.decay_ms = 20;
    cfg.release_ms = PCL_ADSR_TIME_MAX_MS + 1u;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.release_ms = 30;

    cfg.sustain = PCL_COEF(0.0);
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_OK);
    cfg.sustain = PCL_COEF(1.0);
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_OK);
    cfg.sustain = PCL_COEF(-0.01);
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sustain = PCL_COEF(0.5);
    cfg.decay_curve = PCL_COEF(-0.01);
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.decay_curve = PCL_COEF(0.5);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    cfg.sustain = 1.01f;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sustain = NAN;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sustain = 0.5f;
    cfg.attack_curve = 1.01f;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.attack_curve = NAN;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.attack_curve = INFINITY;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.attack_curve = 0.0f;
#else
    cfg.sustain = PCL_COEF(1.0) + 1;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    cfg.sustain = PCL_COEF(0.5);
#endif

    /* errors leave the module untouched */
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_OK);
    pcl_adsr_gate(&a, 1);
    pcl_adsr_process(&a, g_a, NULL, NULL, 0, 200);
    before = a;
    cfg.attack_ms = PCL_ADSR_TIME_MAX_MS + 1u;
    PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&a, &before, sizeof a) == 0);
    PCL_CHECK_EQ_INT(pcl_adsr_set_attack_ms(&a, PCL_ADSR_TIME_MAX_MS + 1u), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_adsr_set_decay_ms(&a, PCL_ADSR_TIME_MAX_MS + 1u), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_adsr_set_release_ms(&a, PCL_ADSR_TIME_MAX_MS + 1u), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_adsr_set_sustain(&a, PCL_COEF(-0.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_adsr_set_attack_curve(&a, PCL_COEF(-0.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_adsr_set_decay_curve(&a, PCL_COEF(-0.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK_EQ_INT(pcl_adsr_set_release_curve(&a, PCL_COEF(-0.5)), PCL_ERR_OUT_OF_RANGE);
    PCL_CHECK(memcmp(&a, &before, sizeof a) == 0);
    PCL_CHECK_EQ_INT(pcl_adsr_set_attack_ms(NULL, 1), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_adsr_set_decay_ms(NULL, 1), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_adsr_set_release_ms(NULL, 1), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_adsr_set_sustain(NULL, PCL_COEF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_adsr_set_attack_curve(NULL, PCL_COEF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_adsr_set_decay_curve(NULL, PCL_COEF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_adsr_set_release_curve(NULL, PCL_COEF(0.5)), PCL_ERR_NULL_POINTER);
    PCL_CHECK_EQ_INT(pcl_adsr_set_attack_ms(&a, 5), PCL_OK);
    PCL_CHECK_EQ_INT(pcl_adsr_set_decay_curve(&a, PCL_COEF(1.0)), PCL_OK);
}

/* ---- Agreement with the model ---- */

typedef struct
{
    const char *name;
    mcfg_t      cfg;
    medge_t     edges[12];
    size_t      ne;
    size_t      n;
} scenario_t;

static scenario_t g_scen[16];
static size_t     g_nscen;

static void add_scenario(const char *name, mcfg_t cfg, size_t n, const medge_t *edges, size_t ne)
{
    scenario_t *s = &g_scen[g_nscen++];
    s->name = name;
    s->cfg = cfg;
    s->n = n;
    s->ne = ne;
    memcpy(s->edges, edges, ne * sizeof edges[0]);
}

static void build_scenarios(void)
{
    static int built;
    if (built)
        return;
    built = 1;
    {
        const medge_t e[] = { { 100, 1 }, { 20000, 0 } };
        add_scenario("linear, full cycle", make_cfg(10, 50, 0.4, 80, 0, 0, 0, 0), 36000, e, 2);
        add_scenario(
            "curve 0.5, full cycle", make_cfg(10, 50, 0.4, 80, 0.5, 0.5, 0.5, 0), 36000, e, 2);
        add_scenario("mixed curves", make_cfg(20, 70, 0.3, 120, 1.0, 0.25, 0.75, 0), 36000, e, 2);
        add_scenario("small curves", make_cfg(20, 70, 0.3, 120, 0.002, 0.01, 0.1, 0), 36000, e, 2);
        add_scenario("sustain 0 and 1", make_cfg(5, 40, 0.0, 30, 0.5, 0.5, 0.5, 0), 20000, e, 2);
        add_scenario(
            "sustain 1 (no decay)", make_cfg(5, 40, 1.0, 30, 0.5, 0.5, 0.5, 0), 20000, e, 2);
    }
    {
        /* retrigger while releasing, a short gate, off during attack */
        const medge_t e[] = { { 100, 1 }, { 3000, 0 }, { 4000, 1 }, { 9000, 0 }, { 9100, 1 },
            { 9200, 0 }, { 9250, 1 }, { 30000, 0 } };
        add_scenario("retriggers", make_cfg(30, 60, 0.5, 200, 0.6, 0.4, 0.8, 0), 40000, e, 8);
    }
    {
        const medge_t e[] = { { 10, 1 }, { 1200, 0 }, { 6000, 1 }, { 6001, 0 } };
        add_scenario("run_through", make_cfg(3, 50, 0.3, 60, 0.5, 0.5, 0.5, 1), 30000, e, 4);
    }
    {
        /* run_through: off during the decay, then on again before sustain: the pending release is
         * void */
        const medge_t e[] = { { 10, 1 }, { 500, 0 }, { 700, 1 }, { 20000, 0 } };
        add_scenario(
            "run_through re-gated", make_cfg(3, 80, 0.3, 60, 0.5, 0.5, 0.5, 1), 30000, e, 4);
    }
}

static void test_matches_model(void)
{
    size_t s;
    build_scenarios();
    for (s = 0; s < g_nscen; s++)
    {
        const scenario_t *sc = &g_scen[s];
        pcl_adsr_t        a;
        double            tol = TOL_BASE, worst;
        size_t            where = 0, i;

        for (i = 0; i < 3; i++)
            if (curve_tol(sc->cfg.curve[i]) > tol - TOL_BASE)
                tol = TOL_BASE + curve_tol(sc->cfg.curve[i]);
        make_adsr(&a, &sc->cfg);
        run_events(&a, g_a, sc->n, 256, sc->edges, sc->ne);
        model_adsr(&sc->cfg, sc->edges, sc->ne, sc->n, g_model, NULL);
        worst = max_err(g_a, g_model, sc->n, &where);
        if (!(worst <= tol))
            printf("    %s: max error %.3g at sample %u (limit %.3g)\n", sc->name, worst,
                (unsigned)where, tol);
        PCL_CHECK(worst <= tol);
    }
}

/* The stage reported after every control period equals the model's. */
static void test_stage_sequence(void)
{
    size_t s;
    build_scenarios();
    for (s = 0; s < g_nscen; s++)
    {
        const scenario_t *sc = &g_scen[s];
        static int        log[N_MAX / 16 + 1];
        pcl_adsr_t        a;
        size_t            tick, bad = 0;

        make_adsr(&a, &sc->cfg);
        model_adsr(&sc->cfg, sc->edges, sc->ne, sc->n, g_model, log);
        for (tick = 0; tick * DIV < sc->n; tick++)
        {
            pcl_event_t  ev[64];
            const size_t cnt = events_for_block(sc->edges, sc->ne, tick * DIV, DIV, ev);
            pcl_adsr_process(&a, g_a, NULL, ev, cnt, DIV);
            if ((int)pcl_adsr_stage(&a) != log[tick])
                bad++;
        }
        if (bad != 0)
            printf("    %s: stage differs from the model in %u control periods\n", sc->name,
                (unsigned)bad);
        PCL_CHECK_EQ_INT(bad, 0);
    }
}

/* ---- Timing and shape properties, measured from the output alone ---- */

/* First index >= from at which the output is within tol of `level` and stays so for 200 samples. */
static size_t first_reach(const pcl_sample_t *out, size_t from, size_t n, double level, double tol)
{
    size_t i, j;
    for (i = from; i + 200 < n; i++)
    {
        int ok = 1;
        if (fabs(pcl_test_to_double(out[i]) - level) > tol)
            continue;
        for (j = 1; j < 200 && ok; j++)
            ok = fabs(pcl_test_to_double(out[i + j]) - level) <= tol;
        if (ok)
            return i;
    }
    return (size_t)-1;
}

static void test_stage_timing(void)
{
    static const uint32_t times[] = { 0, 1, 7, 100, 1000 };
    static const uint32_t rates[] = { 8000, 44100, 48000, 96000 };
    size_t                r, t;

    for (r = 0; r < sizeof rates / sizeof rates[0]; r++)
    {
        for (t = 0; t < sizeof times / sizeof times[0]; t++)
        {
            pcl_adsr_t    a;
            mcfg_t        c = make_cfg(times[t], times[t], 0.5, times[t], 0, 0, 0, 0);
            const double  period = (double)DIV;
            const double  want = (double)times[t] * (double)rates[r] / 1000.0;
            const long    on_at = 100, off_at = 100 + (long)(3.0 * want) + 4000;
            const medge_t e[] = { { on_at, 1 }, { off_at, 0 } };
            const size_t  n = (size_t)off_at + (size_t)(2.0 * want) + 2000;
            size_t        top, sus_reached, zero;
            const double  tol = 2.0 * TOL_BASE + 1e-4;
            const double  begin
                = (double)(((size_t)on_at / DIV + 1) * DIV); /* first ramp of the attack */
            size_t i;

            c.fs = rates[r];
            make_adsr(&a, &c);
            /* rebuild with the right rate */
            {
                pcl_adsr_config_t cfg;
                cfg.sample_rate_hz = rates[r];
                cfg.attack_ms = cfg.decay_ms = cfg.release_ms = times[t];
                cfg.sustain = PCL_COEF(0.5);
                cfg.attack_curve = cfg.decay_curve = cfg.release_curve = PCL_COEF(0.0);
                cfg.run_through = 0;
                PCL_CHECK_EQ_INT(pcl_adsr_init(&a, &cfg), PCL_OK);
            }
            if (n > N_MAX)
                continue;
            run_events(&a, g_a, n, 256, e, 2);

            /* attack: reaches full scale exactly after round(want / period) periods */
            top = (size_t)-1;
            for (i = 0; i < n; i++)
                if (pcl_test_to_double(g_a[i]) >= FULL_D - 1e-9)
                {
                    top = i;
                    break;
                }
            PCL_CHECK(top != (size_t)-1);
            PCL_CHECK(fabs((double)top - (begin + want)) <= period);
            /* decay: lands on the sustain level one decay time later */
            sus_reached = first_reach(g_a, top, n, 0.5, tol);
            PCL_CHECK(sus_reached != (size_t)-1);
            PCL_CHECK(fabs((double)sus_reached - ((double)top + want)) <= period + 1.0);
            /* release: the first exactly-zero sample after the gate-off */
            zero = (size_t)-1;
            for (i = (size_t)off_at; i < n; i++)
                if (g_a[i] == 0)
                {
                    zero = i;
                    break;
                }
            PCL_CHECK(zero != (size_t)-1);
            PCL_CHECK(fabs((double)zero - (((double)((size_t)off_at / DIV + 1) * period) + want))
                <= period);
        }
    }
}

static void test_shape_and_endpoints(void)
{
    static const double curves[] = { 0.0, 0.3, 1.0 };
    size_t              k;

    for (k = 0; k < 3; k++)
    {
        pcl_adsr_t    a;
        const mcfg_t  c = make_cfg(40, 120, 0.35, 200, curves[k], curves[k], curves[k], 0);
        const medge_t e[] = { { 50, 1 }, { 20000, 0 } };
        size_t        i, peak = 0;
        int           up = 1, down = 1, released = 0;

        make_adsr(&a, &c);
        run_events(&a, g_a, 40000, 256, e, 2);
        for (i = 1; i < 40000; i++)
            if (g_a[i] > g_a[peak])
                peak = i;
        PCL_CHECK_NEAR(
            pcl_test_to_double(g_a[peak]), FULL_D, 1e-12); /* exactly the largest level */
        for (i = 1; i <= peak; i++)
            if (g_a[i] < g_a[i - 1])
                up = 0;
        PCL_CHECK(up); /* attack never falls */
        for (i = peak + 1; i < 20000; i++)
            if (g_a[i] > g_a[i - 1])
                down = 0;
        PCL_CHECK(down); /* decay never rises */
        PCL_CHECK_NEAR(pcl_test_to_double(g_a[19990]), 0.35, TOL_BASE + 1e-9); /* sustain */
        down = 1;
        for (i = 20300; i < 40000; i++)
        {
            if (g_a[i] > g_a[i - 1])
                down = 0;
            if (g_a[i] == 0)
                released = 1;
        }
        PCL_CHECK(down); /* release never rises */
        PCL_CHECK(released);
        PCL_CHECK_EQ_INT(pcl_adsr_stage(&a), PCL_ADSR_IDLE);
        PCL_CHECK_EQ_INT(g_a[39999], 0);
        PCL_CHECK_EQ_INT(g_a[0], 0); /* exactly zero while idle */
    }
}

/* No step between consecutive samples is larger than one full-scale ramp (1/PCL_CONTROL_DIV). */
static void test_no_clicks_at_transitions(void)
{
    size_t s;
    build_scenarios();
    for (s = 0; s < g_nscen; s++)
    {
        const scenario_t *sc = &g_scen[s];
        pcl_adsr_t        a;
        make_adsr(&a, &sc->cfg);
        run_events(&a, g_a, sc->n, 256, sc->edges, sc->ne);
        PCL_CHECK(max_step(g_a, sc->n) <= 1.0 / (double)DIV + 1e-4);
    }
}

/* ---- Gate sources ---- */

static void test_events_and_gate_call(void)
{
    pcl_adsr_t   a;
    const mcfg_t c = make_cfg(10, 30, 0.5, 40, 0, 0, 0, 0);
    pcl_event_t  ev[6];

    /* NOTE_ON at 10, a CLOCK and a NOTE_OFF at a past-the-end offset are ignored; NOTE_OFF at 4000
     */
    make_adsr(&a, &c);
    memset(ev, 0, sizeof ev);
    ev[0].offset = 10;
    ev[0].type = PCL_EVT_NOTE_ON;
    ev[1].offset = 20;
    ev[1].type = PCL_EVT_CLOCK;
    ev[2].offset = 5000;
    ev[2].type = PCL_EVT_NOTE_OFF; /* == frames: ignored */
    pcl_adsr_process(&a, g_a, NULL, ev, 3, 5000);
    {
        const medge_t e[] = { { 10, 1 } };
        model_adsr(&c, e, 1, 5000, g_model, NULL);
        PCL_CHECK(max_err(g_a, g_model, 5000, NULL) <= TOL_BASE);
    }
    PCL_CHECK_EQ_INT(pcl_adsr_stage(&a), PCL_ADSR_SUSTAIN);

    /* ON then OFF at one offset ends low; OFF then ON ends high */
    make_adsr(&a, &c);
    memset(ev, 0, sizeof ev);
    ev[0].offset = 100;
    ev[0].type = PCL_EVT_NOTE_ON;
    ev[1].offset = 3000;
    ev[1].type = PCL_EVT_NOTE_ON; /* already high: nothing */
    ev[2].offset = 3000;
    ev[2].type = PCL_EVT_NOTE_OFF;
    pcl_adsr_process(&a, g_a, NULL, ev, 3, 6000);
    PCL_CHECK_EQ_INT(pcl_adsr_stage(&a), PCL_ADSR_IDLE);
    make_adsr(&a, &c);
    memset(ev, 0, sizeof ev);
    ev[0].offset = 100;
    ev[0].type = PCL_EVT_NOTE_ON;
    ev[1].offset = 3000;
    ev[1].type = PCL_EVT_NOTE_OFF;
    ev[2].offset = 3000;
    ev[2].type = PCL_EVT_NOTE_ON;
    pcl_adsr_process(&a, g_a, NULL, ev, 3, 6000);
    PCL_CHECK_EQ_INT(
        pcl_adsr_stage(&a), PCL_ADSR_SUSTAIN); /* a dip and back inside one period: stays high */

    /* pcl_adsr_gate() before processing starts the attack at the very first control point */
    make_adsr(&a, &c);
    pcl_adsr_gate(&a, 1);
    pcl_adsr_process(&a, g_a, NULL, NULL, 0, 4000);
    {
        const medge_t e[] = { { -1, 1 } };
        model_adsr(&c, e, 1, 4000, g_model, NULL);
        PCL_CHECK(max_err(g_a, g_model, 4000, NULL) <= TOL_BASE);
    }
    pcl_adsr_gate(&a, 0);
    pcl_adsr_process(&a, g_a, NULL, NULL, 0, 4000);
    PCL_CHECK_EQ_INT(pcl_adsr_stage(&a), PCL_ADSR_IDLE);
}

/* Reference Schmitt trigger on the quantized gate levels: edges at the sample they happen. */
static size_t gate_edges(const pcl_sample_t *g, size_t n, medge_t *edges, size_t max_edges)
{
    int    high = 0;
    size_t i, ne = 0;
    for (i = 0; i < n; i++)
    {
        const double v = pcl_test_to_double(g[i]);
        if (v >= 0.5 && !high)
        {
            high = 1;
            if (ne < max_edges)
            {
                edges[ne].at = (long)i;
                edges[ne].on = 1;
                ne++;
            }
        }
        else if (v < 0.25 && high)
        {
            high = 0;
            if (ne < max_edges)
            {
                edges[ne].at = (long)i;
                edges[ne].on = 0;
                ne++;
            }
        }
    }
    return ne;
}

static void test_gate_buffer(void)
{
    static const size_t blocks[] = { 1, 7, 16, 100, 256, 20000 };
    const mcfg_t        c = make_cfg(10, 40, 0.4, 60, 0.5, 0.5, 0.5, 0);
    medge_t             edges[40];
    pcl_adsr_t          a;
    size_t              i, b, ne;
    const size_t        n = 20000;

    /* starts high; a dip to 0.3 stays inside the hysteresis band; 0.1 releases; a 1-sample pulse;
       edges straddle block boundaries */
    for (i = 0; i < n; i++)
        g_gate[i] = 0;
    for (i = 0; i < 4000; i++)
        g_gate[i] = pcl_test_from_double(0.9);
    for (i = 1500; i < 1540; i++) /* 40 samples: longer than a control period, so it would matter */
        g_gate[i] = pcl_test_from_double(0.3);
    g_gate[2500] = pcl_test_from_double(0.1);
    g_gate[2501] = pcl_test_from_double(0.9);
    for (i = 6000; i < 6003; i++)
        g_gate[i] = pcl_test_from_double(1.0);
    g_gate[9000] = pcl_test_from_double(0.5); /* exactly the threshold fires */
    g_gate[9001] = pcl_test_from_double(0.49);
    g_gate[12000] = pcl_test_from_double(0.9); /* a single-sample pulse */
    ne = gate_edges(g_gate, n, edges, 40);
    PCL_CHECK(ne >= 8);
    PCL_CHECK_EQ_INT(edges[0].at, 0); /* high on the first sample fires */
    model_adsr(&c, edges, ne, n, g_model, NULL);

    for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
    {
        size_t done = 0;
        make_adsr(&a, &c);
        while (done < n)
        {
            const size_t len = n - done < blocks[b] ? n - done : blocks[b];
            pcl_adsr_process(&a, g_a + done, g_gate + done, NULL, 0, len);
            done += len;
        }
        PCL_CHECK(max_err(g_a, g_model, n, NULL) <= TOL_BASE + curve_tol(0.5));
    }

    /* the gate buffer may alias the output */
    make_adsr(&a, &c);
    memcpy(g_b, g_gate, n * sizeof g_gate[0]);
    pcl_adsr_process(&a, g_b, g_b, NULL, 0, n);
    PCL_CHECK(max_err(g_b, g_model, n, NULL) <= TOL_BASE + curve_tol(0.5));
}

static void test_run_through_short_triggers(void)
{
    const mcfg_t on = make_cfg(10, 40, 0.0, 60, 0, 0, 0, 1);
    const mcfg_t off = make_cfg(10, 40, 0.0, 60, 0, 0, 0, 0);
    pcl_adsr_t   a;
    size_t       i;
    double       peak_on = 0.0, peak_off = 0.0;
    const size_t n = 12000;

    memset(g_gate, 0, sizeof g_gate);
    g_gate[1000] = pcl_test_from_double(0.9); /* a one-sample pulse */

    make_adsr(&a, &on);
    pcl_adsr_process(&a, g_a, g_gate, NULL, 0, n);
    make_adsr(&a, &off);
    pcl_adsr_process(&a, g_b, g_gate, NULL, 0, n);
    for (i = 0; i < n; i++)
    {
        peak_on = fmax(peak_on, pcl_test_to_double(g_a[i]));
        peak_off = fmax(peak_off, pcl_test_to_double(g_b[i]));
    }
    PCL_CHECK_NEAR(peak_on, FULL_D, 1e-12); /* the whole attack plays */
    PCL_CHECK(peak_off < 0.1);              /* released one control period into a 10 ms attack */
    PCL_CHECK_EQ_INT(g_a[n - 1], 0);        /* sustain 0: decayed to nothing, then released */
}

/* ---- Determinism ---- */

static void test_block_size_independence(void)
{
    static const size_t blocks[] = { 1, 7, 16, 17, 256, 1000 };
    size_t              s, b;

    build_scenarios();
    for (s = 0; s < g_nscen; s++)
    {
        const scenario_t *sc = &g_scen[s];
        pcl_adsr_t        ref, a;

        make_adsr(&ref, &sc->cfg);
        run_events(&ref, g_a, sc->n, sc->n, sc->edges, sc->ne);
        for (b = 0; b < sizeof blocks / sizeof blocks[0]; b++)
        {
            make_adsr(&a, &sc->cfg);
            run_events(&a, g_b, sc->n, blocks[b], sc->edges, sc->ne);
            PCL_CHECK(memcmp(g_a, g_b, sc->n * sizeof g_a[0]) == 0);
            PCL_CHECK_EQ_INT(pcl_adsr_stage(&a), pcl_adsr_stage(&ref));
        }
    }
}

static void test_reset_determinism(void)
{
    pcl_adsr_t a;
    size_t     s;

    build_scenarios();
    for (s = 0; s < g_nscen; s++)
    {
        const scenario_t *sc = &g_scen[s];
        make_adsr(&a, &sc->cfg);
        run_events(&a, g_a, sc->n, 256, sc->edges, sc->ne);
        pcl_adsr_reset(&a);
        PCL_CHECK_EQ_INT(pcl_adsr_stage(&a), PCL_ADSR_IDLE);
        run_events(&a, g_b, sc->n, 256, sc->edges, sc->ne);
        PCL_CHECK(memcmp(g_a, g_b, sc->n * sizeof g_a[0]) == 0);
    }
}

/* ---- Parameters while running ---- */

static void test_sustain_glides(void)
{
    pcl_adsr_t   a;
    const mcfg_t c = make_cfg(1, 1, 0.8, 50, 0, 0, 0, 0);
    size_t       i;
    int          monotone = 1;

    make_adsr(&a, &c);
    pcl_adsr_gate(&a, 1);
    pcl_adsr_process(&a, g_a, NULL, NULL, 0, 4000);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[3999]), 0.8, TOL_BASE + 1e-9);
    PCL_CHECK_EQ_INT(pcl_adsr_set_sustain(&a, PCL_COEF(0.3)), PCL_OK);
    pcl_adsr_process(&a, g_a, NULL, NULL, 0, 4000);
    for (i = 1; i < 4000; i++)
        if (g_a[i] > g_a[i - 1])
            monotone = 0;
    PCL_CHECK(monotone);
    PCL_CHECK_NEAR(pcl_test_to_double(g_a[3999]), 0.3, TOL_BASE + 1e-9);
    PCL_CHECK(max_step(g_a, 4000) <= 0.5 / 200.0); /* the ~5 ms (240 sample) glide: no step */
    PCL_CHECK(pcl_test_to_double(g_a[100]) > 0.3); /* not instant */
    PCL_CHECK(pcl_test_to_double(g_a[500]) < 0.3 + 1e-3); /* but finished within about 10 ms */
}

static size_t first_zero_after(const pcl_sample_t *out, size_t from, size_t n)
{
    size_t i;
    for (i = from; i < n; i++)
        if (out[i] == 0)
            return i;
    return (size_t)-1;
}

static void test_times_are_latched_at_stage_start(void)
{
    pcl_adsr_t   a;
    const mcfg_t c = make_cfg(1, 1, 0.5, 100, 0, 0, 0, 0);
    const size_t first_off = 4000;
    size_t       z1, z2;

    make_adsr(&a, &c);
    pcl_adsr_gate(&a, 1);
    pcl_adsr_process(&a, g_a, NULL, NULL, 0, first_off);
    pcl_adsr_gate(&a, 0);
    pcl_adsr_process(&a, g_a, NULL, NULL, 0, 100); /* the release has begun */
    PCL_CHECK_EQ_INT(pcl_adsr_set_release_ms(&a, 1000), PCL_OK);
    pcl_adsr_process(&a, g_a + 100, NULL, NULL, 0, 20000);
    z1 = first_zero_after(g_a, 0, 20100);
    /* the running release stays 100 ms (4800 samples) long, give or take a period */
    PCL_CHECK(z1 != (size_t)-1 && z1 > 4500 && z1 < 5100);

    /* the next release uses the new time (1000 ms = 48000 samples) */
    pcl_adsr_gate(&a, 1);
    pcl_adsr_process(&a, g_b, NULL, NULL, 0, 4000);
    pcl_adsr_gate(&a, 0);
    pcl_adsr_process(&a, g_b, NULL, NULL, 0, N_MAX);
    z2 = first_zero_after(g_b, 0, N_MAX);
    PCL_CHECK(z2 != (size_t)-1 && z2 > 47000 && z2 < 49000);
}

/* ---- Extremes and long runs ---- */

static void check_level_range(const pcl_sample_t *buf, size_t n, size_t *bad)
{
    size_t i;
    for (i = 0; i < n; i++)
    {
        const double v = pcl_test_to_double(buf[i]);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        if (!isfinite(buf[i]) || (buf[i] != 0.0f && fabsf(buf[i]) < FLT_MIN))
            (*bad)++;
#endif
        if (v < 0.0 || v > FULL_D + 1e-12)
            (*bad)++;
    }
}

static void test_extreme_parameters(void)
{
    static const uint32_t times[] = { 0, 60000 };
    static const double   sus[] = { 0.0, 1.0 };
    static const double   curves[] = { 0.0, 1.0 };
    size_t                bad = 0, ti, si, ci;

    for (ti = 0; ti < 2; ti++)
        for (si = 0; si < 2; si++)
            for (ci = 0; ci < 2; ci++)
            {
                pcl_adsr_t    a;
                mcfg_t        c = make_cfg(times[ti], times[ti], sus[si], times[ti], curves[ci],
                    curves[ci], curves[ci], (int)ci);
                const medge_t e[] = { { 50, 1 }, { 30000, 0 }, { 31000, 1 }, { 31010, 0 } };
                make_adsr(&a, &c);
                run_events(&a, g_a, 50000, 128, e, 4);
                check_level_range(g_a, 50000, &bad);
                PCL_CHECK(max_step(g_a, 50000) <= 1.0 / (double)DIV + 1e-4);
            }
    PCL_CHECK_EQ_INT(bad, 0);
}

/* Random gates, times, curves and sustain changes for 10 minutes: in range, finite, no jumps. */
static void test_long_random_run(void)
{
    const size_t total = (size_t)SAMPLE_RATE_HZ * 600u;
    uint32_t     seed = 424242u;
    pcl_adsr_t   a;
    mcfg_t       c = make_cfg(5, 20, 0.5, 30, 0.5, 0.5, 0.5, 0);
    size_t       done = 0, bad = 0, blocks = 0;
    double       worst_step = 0.0, last = 0.0;

    make_adsr(&a, &c);
    while (done < total)
    {
        const size_t len = 1u + (size_t)(((pcl_test_rand(&seed) + 1.0) * 0.5) * 255.0);
        pcl_event_t  ev[4];
        size_t       cnt = 0, i;
        double       r = (pcl_test_rand(&seed) + 1.0) * 0.5;

        if (r < 0.04) /* a gate change somewhere in the block */
        {
            ev[cnt].offset = (uint16_t)(len > 1
                    ? (size_t)((pcl_test_rand(&seed) + 1.0) * 0.5 * (double)(len - 1))
                    : 0);
            ev[cnt].type
                = ((pcl_test_rand(&seed) + 1.0) * 0.5 < 0.5) ? PCL_EVT_NOTE_ON : PCL_EVT_NOTE_OFF;
            ev[cnt].note = 60;
            ev[cnt].velocity = 90;
            cnt++;
        }
        if (r > 0.995) /* change a parameter between blocks */
        {
            const double   u = (pcl_test_rand(&seed) + 1.0) * 0.5;
            const uint32_t ms = (uint32_t)(u * u * 2000.0);
            (void)pcl_adsr_set_attack_ms(&a, ms);
            (void)pcl_adsr_set_decay_ms(&a, ms / 2u);
            (void)pcl_adsr_set_release_ms(&a, ms);
            (void)pcl_adsr_set_sustain(
                &a, pcl_test_from_double(0.0) == 0 ? PCL_COEF(0.9 * u) : PCL_COEF(0.5));
            (void)pcl_adsr_set_decay_curve(&a, PCL_COEF(u));
            (void)pcl_adsr_set_attack_curve(&a, PCL_COEF(1.0 - u));
            pcl_adsr_set_run_through(&a, u > 0.5);
        }
        pcl_adsr_process(&a, g_a, NULL, ev, cnt, len);
        check_level_range(g_a, len, &bad);
        if (blocks > 0)
            worst_step = fmax(worst_step, fabs(pcl_test_to_double(g_a[0]) - last));
        worst_step = fmax(worst_step, max_step(g_a, len));
        last = pcl_test_to_double(g_a[len - 1]);
        for (i = 0; i < 1; i++)
            (void)i;
        done += len;
        blocks++;
    }
    printf("    10 minutes, %u random blocks: worst step %.4f (limit %.4f)\n", (unsigned)blocks,
        worst_step, 1.0 / (double)DIV);
    PCL_CHECK_EQ_INT(bad, 0);
    PCL_CHECK(worst_step <= 1.0 / (double)DIV + 1e-4);
}

int main(void)
{
    PCL_RUN_TEST(test_init_validation);
    PCL_RUN_TEST(test_matches_model);
    PCL_RUN_TEST(test_stage_sequence);
    PCL_RUN_TEST(test_stage_timing);
    PCL_RUN_TEST(test_shape_and_endpoints);
    PCL_RUN_TEST(test_no_clicks_at_transitions);
    PCL_RUN_TEST(test_events_and_gate_call);
    PCL_RUN_TEST(test_gate_buffer);
    PCL_RUN_TEST(test_run_through_short_triggers);
    PCL_RUN_TEST(test_block_size_independence);
    PCL_RUN_TEST(test_reset_determinism);
    PCL_RUN_TEST(test_sustain_glides);
    PCL_RUN_TEST(test_times_are_latched_at_stage_start);
    PCL_RUN_TEST(test_extreme_parameters);
    PCL_RUN_TEST(test_long_random_run);
    return pcl_test_summary("adsr");
}
