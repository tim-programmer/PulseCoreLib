#include "pcl_bench.h"

#include <string.h>

/* ---- Cases ---- */

typedef enum
{
    BENCH_OSC,
    BENCH_NOISE,
    BENCH_SAH,
    BENCH_VCA,
    BENCH_LFO,
    BENCH_ADSR,
    BENCH_MATH,
    BENCH_SVF,
    BENCH_BIQUAD,
    BENCH_LADDER,
    BENCH_DELAY
} bench_kind_t;

typedef enum
{
    MATH_BASELINE, /* the loop, a load and the store, with no call (the floor under every case) */
    MATH_SIN,
    MATH_TANH,
    MATH_HZ_SCALE,
    MATH_HZ_TO_OCT,
    MATH_MIDI,
    MATH_CENTS,
    MATH_DB_TO_GAIN,
    MATH_GAIN_TO_DB,
    MATH_TAN_SERIES, /* cutoff below 0.159 fs: the power series            */
    MATH_TAN_SINCOS  /* cutoff above it: sin / cos and a division          */
} bench_math_fn_t;

enum
{
    BENCH_ADSR_IDLE,    /* never gated: the idle path                                   */
    BENCH_ADSR_SUSTAIN, /* gated on, 1 ms attack and decay: almost all of it in SUSTAIN */
    BENCH_ADSR_CYCLE,   /* NOTE_ON, then NOTE_OFF half-way: every stage runs            */
    BENCH_ADSR_BUFFER   /* the same cycle driven by a gate buffer (Schmitt scan)        */
};

typedef struct
{
    const char  *name;
    bench_kind_t kind;
    /* oscillator */
    pcl_osc_waveform_t waveform;
    pcl_hz_t           hz;
    pcl_oct_t          depth_oct;
    pcl_coef_t         width;
    pcl_coef_t         pwm_depth;
    int                modulated; /* feed noise to the pitch and PWM inputs      */
    pcl_hz_t           master_hz; /* non-zero: sync to a saw master at this rate */
    /* noise */
    pcl_noise_colour_t colour;
    /* sample-and-hold */
    pcl_hz_t sah_rate_hz;
    uint32_t sah_slew_ms;
    int      sah_inputs; /* drive the gate buffer and CLOCK events as well as the internal clock */
    /* VCA */
    int vca_modulated; /* feed a noise modulation buffer                               */
    int vca_ramping;   /* change the gain before every block so the smoother is active */
    /* LFO */
    pcl_lfo_shape_t lfo_shape;
    int             lfo_modulated; /* feed a noise buffer to the rate input */
    /* ADSR */
    int    adsr_mode; /* BENCH_ADSR_* */
    double adsr_curve;
    /* math */
    bench_math_fn_t math_fn;
    /* state-variable filter */
    int svf_single; /* the single-mode call (notch) instead of the three-output call */
    int svf_mod;    /* feed a noise buffer to the cutoff modulation input             */
    int svf_sat;    /* the tanh state limiter                                         */
    /* biquad */
    int bq_type;  /* pcl_biquad_type_t                                              */
    int bq_mod;   /* feed a noise buffer to the cutoff modulation input             */
    int bq_sweep; /* move the cutoff every block, so the coefficients ramp          */
    /* ladder */
    int ld_mode; /* pcl_ladder_mode_t                                              */
    int ld_mod;  /* feed a noise buffer to the cutoff modulation input             */
    int ld_sat;  /* tanh in the feedback loop                                      */
    /* delay */
    int dl_hermite; /* Hermite interpolation instead of linear                         */
    int dl_mod;     /* feed a noise buffer to the time modulation input                */
    int dl_tone;    /* tone low-pass and tanh in the feedback loop                     */
} bench_case_t;

#define STATIC_CASE(name_, wf, hz_)                                                                \
    { .name = name_,                                                                               \
        .kind = BENCH_OSC,                                                                         \
        .waveform = wf,                                                                            \
        .hz = PCL_HZ(hz_),                                                                         \
        .depth_oct = PCL_OCT(1.0),                                                                 \
        .width = PCL_COEF(0.37),                                                                   \
        .pwm_depth = PCL_COEF(0.0) }
#define MOD_CASE(name_, wf)                                                                        \
    { .name = name_,                                                                               \
        .kind = BENCH_OSC,                                                                         \
        .waveform = wf,                                                                            \
        .hz = PCL_HZ(1500.0),                                                                      \
        .depth_oct = PCL_OCT(2.0),                                                                 \
        .width = PCL_COEF(0.4),                                                                    \
        .pwm_depth = PCL_COEF(0.3),                                                                \
        .modulated = 1 }
#define SYNC_CASE(name_, wf)                                                                       \
    { .name = name_,                                                                               \
        .kind = BENCH_OSC,                                                                         \
        .waveform = wf,                                                                            \
        .hz = PCL_HZ(2300.0),                                                                      \
        .depth_oct = PCL_OCT(1.0),                                                                 \
        .width = PCL_COEF(0.5),                                                                    \
        .pwm_depth = PCL_COEF(0.0),                                                                \
        .master_hz = PCL_HZ(1100.0) }
#define NOISE_CASE(name_, colour_) { .name = name_, .kind = BENCH_NOISE, .colour = colour_ }
#define SAH_CASE(name_, rate_hz_, slew_ms_, inputs_)                                               \
    { .name = name_,                                                                               \
        .kind = BENCH_SAH,                                                                         \
        .sah_rate_hz = PCL_HZ(rate_hz_),                                                           \
        .sah_slew_ms = slew_ms_,                                                                   \
        .sah_inputs = inputs_ }

#define VCA_CASE(name_, modulated_, ramping_)                                                      \
    { .name = name_, .kind = BENCH_VCA, .vca_modulated = modulated_, .vca_ramping = ramping_ }

#define LFO_CASE(name_, shape_, modulated_)                                                        \
    { .name = name_, .kind = BENCH_LFO, .lfo_shape = shape_, .lfo_modulated = modulated_ }

#define ADSR_CASE(name_, mode_, curve_)                                                            \
    { .name = name_, .kind = BENCH_ADSR, .adsr_mode = mode_, .adsr_curve = curve_ }

#define SVF_CASE(name_, single_, mod_, sat_)                                                       \
    { .name = name_, .kind = BENCH_SVF, .svf_single = single_, .svf_mod = mod_, .svf_sat = sat_ }

#define BIQUAD_CASE(name_, type_, mod_, sweep_)                                                    \
    { .name = name_, .kind = BENCH_BIQUAD, .bq_type = type_, .bq_mod = mod_, .bq_sweep = sweep_ }

#define LADDER_CASE(name_, mode_, mod_, sat_)                                                      \
    { .name = name_, .kind = BENCH_LADDER, .ld_mode = mode_, .ld_mod = mod_, .ld_sat = sat_ }

#define DELAY_CASE(name_, hermite_, mod_, tone_)                                                   \
    { .name = name_, .kind = BENCH_DELAY, .dl_hermite = hermite_, .dl_mod = mod_, .dl_tone = tone_ }

#define MATH_CASE(name_, fn_) { .name = name_, .kind = BENCH_MATH, .math_fn = fn_ }

/* Keep the oscillator cases first and unchanged: their ids and numbers are quoted in the docs. */
static const bench_case_t g_cases[] = {
    STATIC_CASE("sine_1k", PCL_OSC_SINE, 1000.0),
    STATIC_CASE("triangle_1k", PCL_OSC_TRIANGLE, 1000.0),
    STATIC_CASE("saw_1k", PCL_OSC_SAW, 1000.0),
    STATIC_CASE("pulse_1k", PCL_OSC_PULSE, 1000.0),
    STATIC_CASE("sine_5k", PCL_OSC_SINE, 5000.0),
    STATIC_CASE("triangle_5k", PCL_OSC_TRIANGLE, 5000.0),
    STATIC_CASE("saw_5k", PCL_OSC_SAW, 5000.0),
    STATIC_CASE("pulse_5k", PCL_OSC_PULSE, 5000.0),
    MOD_CASE("sine_mod", PCL_OSC_SINE),
    MOD_CASE("triangle_mod", PCL_OSC_TRIANGLE),
    MOD_CASE("saw_mod", PCL_OSC_SAW),
    MOD_CASE("pulse_mod", PCL_OSC_PULSE),
    SYNC_CASE("saw_sync", PCL_OSC_SAW),
    SYNC_CASE("triangle_sync", PCL_OSC_TRIANGLE),
    NOISE_CASE("noise_white", PCL_NOISE_WHITE),
    NOISE_CASE("noise_pink", PCL_NOISE_PINK),
    NOISE_CASE("noise_brown", PCL_NOISE_BROWN),
    SAH_CASE("sah_clock", 1000.0, 0, 0),
    SAH_CASE("sah_slew", 1000.0, 5, 0),
    SAH_CASE("sah_all_inputs", 1000.0, 5, 1),
    /* The per-sample baseline: clock off, nothing connected, so no pick ever happens. */
    SAH_CASE("sah_hold", 0.0, 0, 0),
    VCA_CASE("vca_steady", 0, 0),
    VCA_CASE("vca_mod", 1, 0),
    VCA_CASE("vca_ramp", 0, 1),
    VCA_CASE("vca_mod_ramp", 1, 1),
    LFO_CASE("lfo_sine", PCL_LFO_SINE, 0),
    LFO_CASE("lfo_triangle", PCL_LFO_TRIANGLE, 0),
    LFO_CASE("lfo_saw_up", PCL_LFO_SAW_UP, 0),
    LFO_CASE("lfo_square", PCL_LFO_SQUARE, 0),
    LFO_CASE("lfo_random_step", PCL_LFO_RANDOM_STEP, 0),
    LFO_CASE("lfo_random_smooth", PCL_LFO_RANDOM_SMOOTH, 0),
    LFO_CASE("lfo_sine_rate_mod", PCL_LFO_SINE, 1),
    ADSR_CASE("adsr_idle", BENCH_ADSR_IDLE, 0.5),
    ADSR_CASE("adsr_sustain", BENCH_ADSR_SUSTAIN, 0.5),
    ADSR_CASE("adsr_cycle", BENCH_ADSR_CYCLE, 0.5),
    ADSR_CASE("adsr_cycle_linear", BENCH_ADSR_CYCLE, 0.0),
    ADSR_CASE("adsr_gate_buffer", BENCH_ADSR_BUFFER, 0.5),
    MATH_CASE("math_baseline", MATH_BASELINE),
    MATH_CASE("math_sin", MATH_SIN),
    MATH_CASE("math_tanh", MATH_TANH),
    MATH_CASE("math_hz_scale", MATH_HZ_SCALE),
    MATH_CASE("math_hz_to_oct", MATH_HZ_TO_OCT),
    MATH_CASE("math_midi_to_hz", MATH_MIDI),
    MATH_CASE("math_cents_to_oct", MATH_CENTS),
    MATH_CASE("math_db_to_gain", MATH_DB_TO_GAIN),
    MATH_CASE("math_gain_to_db", MATH_GAIN_TO_DB),
    MATH_CASE("math_tan_series", MATH_TAN_SERIES),
    MATH_CASE("math_tan_sincos", MATH_TAN_SINCOS),
    SVF_CASE("svf_static", 0, 0, 0),
    SVF_CASE("svf_mod", 0, 1, 0),
    SVF_CASE("svf_notch", 1, 0, 0),
    SVF_CASE("svf_saturate", 0, 0, 1),
    SVF_CASE("svf_mod_saturate", 0, 1, 1),
    BIQUAD_CASE("biquad_lp", PCL_BIQUAD_LP, 0, 0),
    BIQUAD_CASE("biquad_peak", PCL_BIQUAD_PEAK, 0, 0),
    BIQUAD_CASE("biquad_mod", PCL_BIQUAD_LP, 1, 0),
    BIQUAD_CASE("biquad_sweep", PCL_BIQUAD_PEAK, 0, 1),
    LADDER_CASE("ladder_lp24", PCL_LADDER_LP24, 0, 0),
    LADDER_CASE("ladder_hp24", PCL_LADDER_HP24, 0, 0),
    LADDER_CASE("ladder_mod", PCL_LADDER_LP24, 1, 0),
    LADDER_CASE("ladder_saturate", PCL_LADDER_LP24, 0, 1),
    LADDER_CASE("ladder_mod_saturate", PCL_LADDER_LP24, 1, 1),
    DELAY_CASE("delay_linear", 0, 0, 0),
    DELAY_CASE("delay_linear_tone", 0, 0, 1),
    DELAY_CASE("delay_hermite_mod", 1, 1, 0),
    DELAY_CASE("delay_hermite_mod_tone", 1, 1, 1),
};

#define BENCH_SAMPLE_RATE_HZ 48000u

size_t pcl_bench_case_count(void) { return sizeof g_cases / sizeof g_cases[0]; }

const char *pcl_bench_case_name(size_t id)
{
    return id < pcl_bench_case_count() ? g_cases[id].name : "?";
}

/* ---- Deterministic noise: xorshift32, identical on every platform ---- */

static uint32_t bench_xorshift32(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

/* Uniform noise in [-0.5, 0.5) as a sample, using integer arithmetic only. */
static pcl_sample_t bench_noise(uint32_t *state)
{
    const int32_t r = (int32_t)(bench_xorshift32(state) >> 16) - 32768; /* [-32768, 32767] */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)r * (0.5f / 32768.0f);
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_sample_t)(r / 2);
#else
    return r * 32768;
#endif
}

/* ---- Running a case ---- */

static void timing_start(pcl_bench_result_t *r)
{
    r->cycles_total = 0;
    r->cycles_min_block = 0xFFFFFFFFu;
    r->cycles_max_block = 0;
}

/* Records one timed block; `cycles` wraps correctly for a 32-bit counter. */
static void timing_add(pcl_bench_result_t *r, uint32_t cycles)
{
    r->cycles_total += cycles;
    if (cycles < r->cycles_min_block)
        r->cycles_min_block = cycles;
    if (cycles > r->cycles_max_block)
        r->cycles_max_block = cycles;
}

static void timing_finish(pcl_bench_result_t *r, pcl_bench_clock_fn clock)
{
    if (clock == NULL)
        r->cycles_min_block = 0;
}

static pcl_status_t bench_run_osc(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    pcl_osc_t        osc;
    pcl_osc_t        master;
    pcl_osc_config_t cfg;
    pcl_sample_t     pitch_mod[PCL_BENCH_BLOCK];
    pcl_sample_t     pwm_mod[PCL_BENCH_BLOCK];
    pcl_sample_t     sync[PCL_BENCH_BLOCK];
    uint32_t         seed = 0x2545F491u;
    size_t           pos;
    pcl_status_t     st;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.waveform = c->waveform;
    cfg.frequency = c->hz;
    cfg.pitch_depth_oct = c->depth_oct;
    cfg.pulse_width = c->width;
    cfg.pwm_depth = c->pwm_depth;
    st = pcl_osc_init(&osc, &cfg);
    if (st != PCL_OK)
        return st;
    if (c->master_hz != 0)
    {
        cfg.waveform = PCL_OSC_SAW;
        cfg.frequency = c->master_hz;
        st = pcl_osc_init(&master, &cfg);
        if (st != PCL_OK)
            return st;
    }

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK)
    {
        size_t   i;
        uint32_t t0 = 0, t1 = 0;

        /* Inputs are prepared outside the timed region. */
        if (c->modulated)
        {
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
            {
                pitch_mod[i] = bench_noise(&seed);
                pwm_mod[i] = bench_noise(&seed);
            }
        }
        if (c->master_hz != 0)
            pcl_osc_process(&master, out + pos, NULL, NULL, NULL, sync, PCL_BENCH_BLOCK);

        if (clock != NULL)
            t0 = clock();
        pcl_osc_process(&osc, out + pos, c->modulated ? pitch_mod : NULL,
            c->modulated ? pwm_mod : NULL, c->master_hz != 0 ? sync : NULL, NULL, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

#define BENCH_NOISE_SEED 12345u

static pcl_status_t bench_run_noise(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    pcl_noise_t        noise;
    pcl_noise_config_t cfg;
    size_t             pos;
    pcl_status_t       st;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.colour = c->colour;
    cfg.seed = BENCH_NOISE_SEED;
    st = pcl_noise_init(&noise, &cfg);
    if (st != PCL_OK)
        return st;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK)
    {
        uint32_t t0 = 0, t1 = 0;
        if (clock != NULL)
            t0 = clock();
        pcl_noise_process(&noise, out + pos, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

/* Gate pattern for the all-inputs S&H case: high for 100 samples out of every 300. */
#define BENCH_GATE_PERIOD 300u
#define BENCH_GATE_HIGH_FOR 100u
/* Two CLOCK events in every block. */
#define BENCH_EVENT_A_OFFSET 17u
#define BENCH_EVENT_B_OFFSET 130u

static pcl_sample_t bench_gate_high(void)
{
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return 0.9f;
#elif PCL_SAMPLE_TYPE == PCL_Q15
    return PCL_CONST_Q15(0.9);
#else
    return PCL_CONST_Q31(0.9);
#endif
}

static pcl_status_t bench_run_sah(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    pcl_sah_t        sah;
    pcl_sah_config_t cfg;
    pcl_sample_t     gate[PCL_BENCH_BLOCK];
    pcl_event_t      events[2];
    size_t           pos;
    pcl_status_t     st;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.seed = BENCH_NOISE_SEED;
    cfg.rate_hz = c->sah_rate_hz;
    cfg.slew_ms = c->sah_slew_ms;
    st = pcl_sah_init(&sah, &cfg);
    if (st != PCL_OK)
        return st;

    events[0].offset = BENCH_EVENT_A_OFFSET;
    events[1].offset = BENCH_EVENT_B_OFFSET;
    events[0].type = events[1].type = PCL_EVT_CLOCK;
    events[0].note = events[1].note = 0;
    events[0].velocity = events[1].velocity = 0;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK)
    {
        size_t   i;
        uint32_t t0 = 0, t1 = 0;

        if (c->sah_inputs)
        {
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                gate[i] = ((pos + i) % BENCH_GATE_PERIOD) < BENCH_GATE_HIGH_FOR ? bench_gate_high()
                                                                                : (pcl_sample_t)0;
        }
        if (clock != NULL)
            t0 = clock();
        pcl_sah_process(&sah, out + pos, c->sah_inputs ? gate : NULL, c->sah_inputs ? events : NULL,
            c->sah_inputs ? 2u : 0u, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

/* Gains used by the VCA cases: steady, and the two between which the ramping cases alternate. */
#define BENCH_VCA_GAIN 0.5
#define BENCH_VCA_GAIN_LOW 0.25
#define BENCH_VCA_GAIN_HIGH 0.75

static pcl_status_t bench_run_vca(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    pcl_vca_t        vca;
    pcl_vca_config_t cfg;
    pcl_sample_t     in[PCL_BENCH_BLOCK];
    pcl_sample_t     mod[PCL_BENCH_BLOCK];
    uint32_t         seed = 0x2545F491u;
    size_t           pos;
    unsigned         block = 0;
    pcl_status_t     st;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.gain = PCL_COEF(BENCH_VCA_GAIN);
    st = pcl_vca_init(&vca, &cfg);
    if (st != PCL_OK)
        return st;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK, block++)
    {
        size_t   i;
        uint32_t t0 = 0, t1 = 0;

        /* Inputs and the (control-thread) gain change are prepared outside the timed region. */
        for (i = 0; i < PCL_BENCH_BLOCK; i++)
        {
            in[i] = bench_noise(&seed);
            if (c->vca_modulated)
                mod[i] = bench_noise(&seed);
        }
        if (c->vca_ramping)
        {
            st = pcl_vca_set_gain(
                &vca, (block & 1u) ? PCL_COEF(BENCH_VCA_GAIN_HIGH) : PCL_COEF(BENCH_VCA_GAIN_LOW));
            if (st != PCL_OK)
                return st;
        }

        if (clock != NULL)
            t0 = clock();
        pcl_vca_process(&vca, in, out + pos, c->vca_modulated ? mod : NULL, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

#define BENCH_LFO_HZ 5.3
#define BENCH_LFO_DEPTH_OCT 2.0

static pcl_status_t bench_run_lfo(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    pcl_lfo_t        lfo;
    pcl_lfo_config_t cfg;
    pcl_sample_t     mod[PCL_BENCH_BLOCK];
    uint32_t         seed = 0x2545F491u;
    size_t           pos;
    pcl_status_t     st;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.shape = c->lfo_shape;
    cfg.frequency = PCL_HZ(BENCH_LFO_HZ);
    cfg.rate_depth_oct = PCL_OCT(BENCH_LFO_DEPTH_OCT);
    cfg.pulse_width = PCL_COEF(0.4);
    cfg.unipolar = 0;
    cfg.start_phase = 0u;
    cfg.seed = BENCH_NOISE_SEED;
    st = pcl_lfo_init(&lfo, &cfg);
    if (st != PCL_OK)
        return st;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK)
    {
        size_t   i;
        uint32_t t0 = 0, t1 = 0;

        if (c->lfo_modulated)
        {
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                mod[i] = bench_noise(&seed);
        }
        if (clock != NULL)
            t0 = clock();
        pcl_lfo_process(&lfo, out + pos, c->lfo_modulated ? mod : NULL, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

/* ADSR cases: 48 kHz, A 5 ms, D 10 ms, S 0.5, R 10 ms. */
#define BENCH_ADSR_OFF_BLOCK 4u /* the NOTE_OFF / gate fall lands at the start of this block */

static pcl_status_t bench_run_adsr(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    pcl_adsr_t        adsr;
    pcl_adsr_config_t cfg;
    pcl_sample_t      gate[PCL_BENCH_BLOCK];
    pcl_event_t       ev;
    size_t            pos;
    unsigned          block = 0;
    pcl_status_t      st;
    const int         sustain_mode = c->adsr_mode == BENCH_ADSR_SUSTAIN;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.attack_ms = sustain_mode ? 1u : 5u;
    cfg.decay_ms = sustain_mode ? 1u : 10u;
    cfg.release_ms = 10u;
    cfg.sustain = PCL_COEF(0.5);
    cfg.attack_curve = cfg.decay_curve = cfg.release_curve = PCL_COEF(c->adsr_curve);
    cfg.run_through = 0;
    st = pcl_adsr_init(&adsr, &cfg);
    if (st != PCL_OK)
        return st;
    if (sustain_mode)
        pcl_adsr_gate(&adsr, 1);

    ev.offset = 0;
    ev.note = 60;
    ev.velocity = 100;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK, block++)
    {
        const pcl_event_t  *events = NULL;
        const pcl_sample_t *gate_in = NULL;
        size_t              num_events = 0;
        uint32_t            t0 = 0, t1 = 0;

        if (c->adsr_mode == BENCH_ADSR_CYCLE && (block == 0u || block == BENCH_ADSR_OFF_BLOCK))
        {
            ev.type = block == 0u ? PCL_EVT_NOTE_ON : PCL_EVT_NOTE_OFF;
            events = &ev;
            num_events = 1;
        }
        else if (c->adsr_mode == BENCH_ADSR_BUFFER)
        {
            size_t i;
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                gate[i] = (block < BENCH_ADSR_OFF_BLOCK) ? bench_gate_high() : (pcl_sample_t)0;
            gate_in = gate;
        }

        if (clock != NULL)
            t0 = clock();
        pcl_adsr_process(&adsr, out + pos, gate_in, events, num_events, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

/* SVF cases: 48 kHz, cutoff 1.2 kHz, resonance 0.6, noise in. */
#define BENCH_SVF_CUTOFF_HZ 1200.0
#define BENCH_SVF_RESONANCE 0.6
#define BENCH_SVF_DEPTH_OCT 2.0

static pcl_status_t bench_run_svf(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    pcl_svf_t        svf;
    pcl_svf_config_t cfg;
    pcl_sample_t     in[PCL_BENCH_BLOCK];
    pcl_sample_t     mod[PCL_BENCH_BLOCK];
    pcl_sample_t     lp[PCL_BENCH_BLOCK];
    pcl_sample_t     bp[PCL_BENCH_BLOCK];
    pcl_sample_t     hp[PCL_BENCH_BLOCK];
    uint32_t         seed = 0x2545F491u;
    size_t           pos;
    pcl_status_t     st;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.cutoff_hz = PCL_HZ(BENCH_SVF_CUTOFF_HZ);
    cfg.resonance = PCL_COEF(BENCH_SVF_RESONANCE);
    cfg.mod_depth_oct = PCL_OCT(BENCH_SVF_DEPTH_OCT);
    cfg.saturate = c->svf_sat;
    st = pcl_svf_init(&svf, &cfg);
    if (st != PCL_OK)
        return st;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK)
    {
        size_t   i;
        uint32_t t0 = 0, t1 = 0;

        for (i = 0; i < PCL_BENCH_BLOCK; i++)
        {
            in[i] = bench_noise(&seed);
            mod[i] = bench_noise(&seed);
        }
        if (clock != NULL)
            t0 = clock();
        if (c->svf_single)
            pcl_svf_process_mode(
                &svf, in, lp, PCL_SVF_NOTCH, c->svf_mod ? mod : NULL, PCL_BENCH_BLOCK);
        else
            pcl_svf_process(&svf, in, lp, bp, hp, c->svf_mod ? mod : NULL, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
        /* One stream that depends on all three outputs, so the comparison covers them. */
        for (i = 0; i < PCL_BENCH_BLOCK; i++)
        {
            if (c->svf_single)
                out[pos + i] = lp[i];
            else
#if PCL_SAMPLE_TYPE == PCL_FLOAT
                out[pos + i] = lp[i] + bp[i] + hp[i];
#else
                out[pos + i] = (pcl_sample_t)(lp[i] ^ bp[i] ^ hp[i]);
#endif
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

/* Ladder cases: as the SVF's: 48 kHz, cutoff 1.2 kHz, resonance 0.6, noise in. Drive 2. */
#define BENCH_LADDER_DRIVE 2.0

static pcl_status_t bench_run_ladder(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    pcl_ladder_t        ld;
    pcl_ladder_config_t cfg;
    pcl_sample_t        in[PCL_BENCH_BLOCK];
    pcl_sample_t        mod[PCL_BENCH_BLOCK];
    uint32_t            seed = 0x2545F491u;
    size_t              pos;
    pcl_status_t        st;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.cutoff_hz = PCL_HZ(BENCH_SVF_CUTOFF_HZ);
    cfg.resonance = PCL_COEF(BENCH_SVF_RESONANCE);
    cfg.mod_depth_oct = PCL_OCT(BENCH_SVF_DEPTH_OCT);
    cfg.mode = (pcl_ladder_mode_t)c->ld_mode;
    cfg.saturate = c->ld_sat;
    cfg.drive = PCL_WIDE(BENCH_LADDER_DRIVE);
    cfg.bass_comp = 0;
    st = pcl_ladder_init(&ld, &cfg);
    if (st != PCL_OK)
        return st;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK)
    {
        size_t   i;
        uint32_t t0 = 0, t1 = 0;

        for (i = 0; i < PCL_BENCH_BLOCK; i++)
        {
            in[i] = bench_noise(&seed);
            mod[i] = bench_noise(&seed);
        }
        if (clock != NULL)
            t0 = clock();
        pcl_ladder_process(&ld, in, out + pos, c->ld_mod ? mod : NULL, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

/* Delay cases: 48 kHz, 20 ms of a 50 ms line (native storage), feedback 0.6, mix 0.5, noise in;
   modulated: +-5 ms of noise on the time; tone: 3 kHz low-pass and tanh in the loop. */
#define BENCH_DELAY_MAX_MS 50u
#define BENCH_DELAY_BYTES (50u * 48u * sizeof(pcl_sample_t) + 64u)

static pcl_status_t bench_run_delay(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    static unsigned char mem[BENCH_DELAY_BYTES];
    pcl_delay_t          dl;
    pcl_delay_config_t   cfg;
    pcl_sample_t         in[PCL_BENCH_BLOCK];
    pcl_sample_t         mod[PCL_BENCH_BLOCK];
    uint32_t             seed = 0x2545F491u;
    size_t               pos;
    pcl_status_t         st;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.max_delay_ms = BENCH_DELAY_MAX_MS;
    cfg.time_ms = PCL_MS(20.0);
    cfg.mod_depth_ms = PCL_MS(5.0);
    cfg.glide_ms = 0u;
    cfg.feedback = PCL_COEF(0.6);
    cfg.tone_hz = c->dl_tone ? PCL_HZ(3000.0) : PCL_HZ(0.0);
    cfg.saturate = c->dl_tone;
    cfg.mix = PCL_COEF(0.5);
    cfg.interp = c->dl_hermite ? PCL_DELAY_HERMITE : PCL_DELAY_LINEAR;
    cfg.storage = PCL_DELAY_STORE_NATIVE;
    st = pcl_delay_init(&dl, &cfg, mem, sizeof mem);
    if (st != PCL_OK)
        return st;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK)
    {
        size_t   i;
        uint32_t t0 = 0, t1 = 0;

        for (i = 0; i < PCL_BENCH_BLOCK; i++)
        {
            in[i] = bench_noise(&seed);
            mod[i] = bench_noise(&seed);
        }
        if (clock != NULL)
            t0 = clock();
        pcl_delay_process(&dl, in, out + pos, c->dl_mod ? mod : NULL, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

/* Biquad cases: 48 kHz, cutoff 1.2 kHz, Q 2, +6 dB, noise in. */
#define BENCH_BQ_CUTOFF_HZ 1200.0
#define BENCH_BQ_Q 2.0
#define BENCH_BQ_GAIN_DB 6.0
#define BENCH_BQ_DEPTH_OCT 2.0

static pcl_status_t bench_run_biquad(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    pcl_biquad_t        bq;
    pcl_biquad_config_t cfg;
    pcl_sample_t        in[PCL_BENCH_BLOCK];
    pcl_sample_t        mod[PCL_BENCH_BLOCK];
    uint32_t            seed = 0x2545F491u;
    size_t              pos;
    int                 high = 0;
    pcl_status_t        st;

    cfg.sample_rate_hz = BENCH_SAMPLE_RATE_HZ;
    cfg.type = (pcl_biquad_type_t)c->bq_type;
    cfg.cutoff_hz = PCL_HZ(BENCH_BQ_CUTOFF_HZ);
    cfg.q = PCL_QUALITY(BENCH_BQ_Q);
    cfg.gain_db = PCL_DB(BENCH_BQ_GAIN_DB);
    cfg.mod_depth_oct = PCL_OCT(BENCH_BQ_DEPTH_OCT);
    st = pcl_biquad_init(&bq, &cfg);
    if (st != PCL_OK)
        return st;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK)
    {
        size_t   i;
        uint32_t t0 = 0, t1 = 0;

        for (i = 0; i < PCL_BENCH_BLOCK; i++)
        {
            in[i] = bench_noise(&seed);
            mod[i] = bench_noise(&seed);
        }
        if (c->bq_sweep)
        {
            /* an octave up, then down, every block: the cutoff is always gliding */
            high = !high;
            st = pcl_biquad_set_cutoff(
                &bq, PCL_HZ(high ? 2.0 * BENCH_BQ_CUTOFF_HZ : 0.5 * BENCH_BQ_CUTOFF_HZ));
            if (st != PCL_OK)
                return st;
        }
        if (clock != NULL)
            t0 = clock();
        pcl_biquad_process(&bq, in, out + pos, c->bq_mod ? mod : NULL, PCL_BENCH_BLOCK);
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

/*
 * Math cases: one call per output sample, each argument drawn from a seeded xorshift so the
 * host and the target see identical inputs. The cost of a call is cycles_total / frames minus the
 * math_baseline case (loop, argument conversion and store).
 *
 * Inputs and results are built from integer random bits with exact conversions (float builds
 * convert 32-bit integers and scale by powers of two), so the output is bit-comparable.
 */
#define BENCH_MATH_RATE_HZ 48000u

/*
 * Folds a 32-bit fixed-point result into a sample (a checksum-like mapping; float keeps the value).
 * A function, not a macro, so the call being measured is evaluated exactly once.
 */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
#define BENCH_FOLD(x) ((pcl_sample_t)(x))
#else
static pcl_sample_t bench_fold(uint32_t v)
{
#if PCL_SAMPLE_TYPE == PCL_Q15
    return (pcl_sample_t)(v ^ (v >> 16));
#else
    return (pcl_sample_t)v;
#endif
}
#define BENCH_FOLD(x) bench_fold((uint32_t)(x))
#endif

/* Random pcl_wide_t, uniform over [-8, 8). */
static pcl_wide_t bench_rand_wide(uint32_t *s)
{
    const int32_t r = (int32_t)bench_xorshift32(s);
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)r * (1.0f / 268435456.0f);
#else
    return r;
#endif
}

/* Random pcl_oct_t, uniform over [-2, 2). */
static pcl_oct_t bench_rand_oct(uint32_t *s)
{
    const int32_t r = (int32_t)bench_xorshift32(s) >> 2;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)r * (1.0f / 268435456.0f);
#else
    return r;
#endif
}

/* Random pcl_hz_t of raw value 1 .. 2^24 (up to 256 Hz) plus `base_raw` (Q16.16 units). */
static pcl_hz_t bench_rand_hz(uint32_t *s, uint32_t span_bits, uint32_t base_raw)
{
    const uint32_t raw = (bench_xorshift32(s) >> (32u - span_bits)) + base_raw + 1u;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
    return (float)raw * (1.0f / 65536.0f);
#else
    return raw;
#endif
}

static pcl_status_t bench_run_math(
    const bench_case_t *c, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    uint32_t seed = 0x2545F491u;
    size_t   pos;

    timing_start(result);
    for (pos = 0; pos < PCL_BENCH_FRAMES; pos += PCL_BENCH_BLOCK)
    {
        /* Inputs for the block, prepared outside the timed region. */
        union
        {
            uint32_t   u[PCL_BENCH_BLOCK];
            pcl_wide_t w[PCL_BENCH_BLOCK];
            pcl_oct_t  o[PCL_BENCH_BLOCK];
            pcl_hz_t   h[PCL_BENCH_BLOCK];
            pcl_coef_t k[PCL_BENCH_BLOCK];
            pcl_db_t   d[PCL_BENCH_BLOCK];
            uint8_t    n[PCL_BENCH_BLOCK];
            int32_t    i[PCL_BENCH_BLOCK];
        } in;
        pcl_hz_t ref_hz = PCL_HZ(100.0);
        size_t   i;
        uint32_t t0 = 0, t1 = 0;

        switch (c->math_fn)
        {
        case MATH_SIN:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                in.u[i] = bench_xorshift32(&seed);
            break;
        case MATH_BASELINE:
        case MATH_TANH:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                in.w[i] = bench_rand_wide(&seed);
            break;
        case MATH_HZ_SCALE:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                in.o[i] = bench_rand_oct(&seed);
            break;
        case MATH_HZ_TO_OCT:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                in.h[i] = bench_rand_hz(&seed, 24u, 0u);
            break;
        case MATH_MIDI:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                in.n[i] = (uint8_t)(bench_xorshift32(&seed) >> 25);
            break;
        case MATH_CENTS:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                in.i[i] = (int32_t)(bench_xorshift32(&seed) >> 18) - 8192; /* +-8192 cents */
            break;
        case MATH_DB_TO_GAIN:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
            {
                const uint32_t x = bench_xorshift32(&seed) >> 8; /* 24 bits: -120 .. 0 dB */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
                in.d[i] = -(float)x * (120.0f / 16777216.0f);
#else
                in.d[i] = -(int32_t)(x * 120u);
#endif
            }
            break;
        case MATH_GAIN_TO_DB:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
            {
                const uint32_t x = (bench_xorshift32(&seed) >> 2) | 1u; /* Q2.30 up to 1.0 */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
                in.k[i] = (float)x * (1.0f / 1073741824.0f);
#else
                in.k[i] = (int32_t)x;
#endif
            }
            break;
        case MATH_TAN_SERIES: /* 0 .. 4 kHz at 48 kHz */
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                in.h[i] = bench_rand_hz(&seed, 28u, 0u);
            break;
        case MATH_TAN_SINCOS: /* 8 .. 12 kHz at 48 kHz */
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                in.h[i] = bench_rand_hz(&seed, 28u, 8000u * 65536u);
            break;
        }

        if (clock != NULL)
            t0 = clock();
        switch (c->math_fn)
        {
        case MATH_BASELINE:
        {
            /* volatile: otherwise the compiler turns this copy into a (slow) memcpy call */
            volatile pcl_sample_t *vout = out + pos;
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                vout[i] = BENCH_FOLD(in.w[i]);
        }
        break;
        case MATH_SIN:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                out[pos + i] = pcl_sin(in.u[i]);
            break;
        case MATH_TANH:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                out[pos + i] = pcl_tanh(in.w[i]);
            break;
        case MATH_HZ_SCALE:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                out[pos + i] = BENCH_FOLD(pcl_hz_scale(PCL_HZ(440.0), in.o[i]));
            break;
        case MATH_HZ_TO_OCT:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                out[pos + i] = BENCH_FOLD(pcl_hz_to_oct(in.h[i], ref_hz));
            break;
        case MATH_MIDI:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                out[pos + i] = BENCH_FOLD(pcl_midi_to_hz(in.n[i]));
            break;
        case MATH_CENTS:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                out[pos + i] = BENCH_FOLD(pcl_cents_to_oct(in.i[i]));
            break;
        case MATH_DB_TO_GAIN:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                out[pos + i] = BENCH_FOLD(pcl_db_to_gain(in.d[i]));
            break;
        case MATH_GAIN_TO_DB:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                out[pos + i] = BENCH_FOLD(pcl_gain_to_db(in.k[i]));
            break;
        case MATH_TAN_SERIES:
        case MATH_TAN_SINCOS:
            for (i = 0; i < PCL_BENCH_BLOCK; i++)
                out[pos + i] = BENCH_FOLD(pcl_tan_hz(in.h[i], BENCH_MATH_RATE_HZ));
            break;
        }
        if (clock != NULL)
        {
            t1 = clock();
            timing_add(result, t1 - t0);
        }
    }
    timing_finish(result, clock);
    return PCL_OK;
}

pcl_status_t pcl_bench_run(
    size_t id, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result)
{
    if (id >= pcl_bench_case_count())
        return PCL_ERR_OUT_OF_RANGE;
    switch (g_cases[id].kind)
    {
    case BENCH_NOISE:
        return bench_run_noise(&g_cases[id], out, clock, result);
    case BENCH_SAH:
        return bench_run_sah(&g_cases[id], out, clock, result);
    case BENCH_VCA:
        return bench_run_vca(&g_cases[id], out, clock, result);
    case BENCH_LFO:
        return bench_run_lfo(&g_cases[id], out, clock, result);
    case BENCH_ADSR:
        return bench_run_adsr(&g_cases[id], out, clock, result);
    case BENCH_MATH:
        return bench_run_math(&g_cases[id], out, clock, result);
    case BENCH_SVF:
        return bench_run_svf(&g_cases[id], out, clock, result);
    case BENCH_BIQUAD:
        return bench_run_biquad(&g_cases[id], out, clock, result);
    case BENCH_LADDER:
        return bench_run_ladder(&g_cases[id], out, clock, result);
    case BENCH_DELAY:
        return bench_run_delay(&g_cases[id], out, clock, result);
    default:
        return bench_run_osc(&g_cases[id], out, clock, result);
    }
}

/* ---- Output ---- */

static void put_u32(pcl_bench_sink_fn sink, uint32_t v)
{
    unsigned char b[4];
    b[0] = (unsigned char)(v & 0xFFu);
    b[1] = (unsigned char)((v >> 8) & 0xFFu);
    b[2] = (unsigned char)((v >> 16) & 0xFFu);
    b[3] = (unsigned char)((v >> 24) & 0xFFu);
    sink(b, sizeof b);
}

static uint32_t bench_crc32(const unsigned char *p, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    size_t   i;
    int      bit;
    for (i = 0; i < n; i++)
    {
        crc ^= p[i];
        for (bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

pcl_status_t pcl_bench_emit(size_t id, pcl_bench_clock_fn clock, pcl_bench_sink_fn sink)
{
    static pcl_sample_t  samples[PCL_BENCH_FRAMES];
    static unsigned char payload[PCL_BENCH_FRAMES * sizeof(pcl_sample_t)];
    pcl_bench_result_t   r;
    const pcl_status_t   st = pcl_bench_run(id, samples, clock, &r);
    size_t               i;

    if (st != PCL_OK)
        return st;

    /* Serialize little-endian regardless of host byte order. */
    for (i = 0; i < PCL_BENCH_FRAMES; i++)
    {
        uint32_t bits;
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        memcpy(&bits, &samples[i], sizeof bits);
#else
        bits = (uint32_t)samples[i];
#endif
        payload[i * sizeof(pcl_sample_t)] = (unsigned char)(bits & 0xFFu);
        payload[i * sizeof(pcl_sample_t) + 1] = (unsigned char)((bits >> 8) & 0xFFu);
#if PCL_SAMPLE_TYPE != PCL_Q15
        payload[i * sizeof(pcl_sample_t) + 2] = (unsigned char)((bits >> 16) & 0xFFu);
        payload[i * sizeof(pcl_sample_t) + 3] = (unsigned char)((bits >> 24) & 0xFFu);
#endif
    }

    {
        const unsigned char head[2] = { 'C', (unsigned char)id };
        sink(head, sizeof head);
    }
    put_u32(sink, PCL_BENCH_FRAMES);
    put_u32(sink, r.cycles_total);
    put_u32(sink, r.cycles_min_block);
    put_u32(sink, r.cycles_max_block);
    sink(payload, sizeof payload);
    put_u32(sink, bench_crc32(payload, sizeof payload));
    return PCL_OK;
}

void pcl_bench_emit_banner(pcl_bench_sink_fn sink)
{
    static const char *const type =
#if PCL_SAMPLE_TYPE == PCL_FLOAT
        "FLOAT";
#elif PCL_SAMPLE_TYPE == PCL_Q15
        "Q15";
#else
        "Q31";
#endif
    char        line[64];
    size_t      n = 0;
    const char *parts[] = { "PCLBENCH1 type=", type,
        " sample_bytes=", sizeof(pcl_sample_t) == 2 ? "2" : "4", " cases=" };
    size_t      i;
    unsigned    cases = (unsigned)pcl_bench_case_count();

    for (i = 0; i < sizeof parts / sizeof parts[0]; i++)
    {
        size_t len = strlen(parts[i]);
        memcpy(line + n, parts[i], len);
        n += len;
    }
    if (cases >= 10u)
        line[n++] = (char)('0' + cases / 10u);
    line[n++] = (char)('0' + cases % 10u);
    line[n++] = '\n';
    sink(line, n);
}
