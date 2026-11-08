/*
 * pcl_adsr.h - ADSR envelope generator: a gate in, a unipolar level in [0, 1] out.
 *
 * Design note: docs/modules/adsr.md.
 *
 *     gate on:   IDLE/RELEASE --> ATTACK (level -> 1) --> DECAY (-> sustain) --> SUSTAIN (held)
 *     gate off:  any stage    --> RELEASE (-> 0) --> IDLE
 *
 * Rate: CONTROL RATE. The level is evaluated once every PCL_CONTROL_DIV samples (counted from the
 * start of the stream, so the output does not depend on how it is split into blocks) and linearly
 * interpolated to audio rate in between. Consequences:
 *   - gate edges act at the NEXT control point: 0 to PCL_CONTROL_DIV-1 samples late (0.16 ms on
 *     average at 48 kHz with the default of 16);
 *   - a stage lasts a whole number of control periods: round(time_ms * sample_rate / 1000 /
 *     PCL_CONTROL_DIV), at least one, so it is within half a period of the requested time (0 ms
 * means one period, ramped, so there is no click);
 *   - a gate that falls and rises again inside one control period counts as having stayed high.
 *
 * Stage times are the time the stage takes wherever it starts (a retrigger from level 0.9 still
 * takes the full attack time). The curve of each stage is a normalised exponential: 0 = linear, 1 =
 * the stage falls through 96 dB of exponential before being normalised to its end point (0.5 is a
 * typical analog character); a curve below 1/1024 is exactly linear. Attack is the mirror image of
 * decay.
 *
 * Setters: times and curves are LATCHED when their stage begins (smoothing a duration has no
 * meaning); the sustain level is smoothed (~PCL_SMOOTH_MS).
 *
 * Output: exactly 0 in IDLE, exactly the largest representable level at the end of ATTACK (32767 in
 * Q15, 0x7FFFFFFE in Q31, 1.0f in float; the fixed-point formats have no +1.0), never negative.
 */
#ifndef PCL_ADSR_H
#define PCL_ADSR_H

#include "pcl/pcl_event.h"
#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

/* Longest stage time accepted. */
#define PCL_ADSR_TIME_MAX_MS 60000u

typedef enum
{
    PCL_ADSR_IDLE = 0,
    PCL_ADSR_ATTACK = 1,
    PCL_ADSR_DECAY = 2,
    PCL_ADSR_SUSTAIN = 3,
    PCL_ADSR_RELEASE = 4
} pcl_adsr_stage_t;

typedef struct
{
    uint32_t   sample_rate_hz; /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ         */
    uint32_t   attack_ms;      /* 0 .. PCL_ADSR_TIME_MAX_MS                                */
    uint32_t   decay_ms;       /* 0 .. PCL_ADSR_TIME_MAX_MS                                */
    uint32_t   release_ms;     /* 0 .. PCL_ADSR_TIME_MAX_MS                                */
    pcl_coef_t sustain;        /* level, 0 .. 1; PCL_COEF(x)                               */
    pcl_coef_t attack_curve;   /* 0 (linear) .. 1; PCL_COEF(x)                             */
    pcl_coef_t decay_curve;    /* 0 (linear) .. 1; PCL_COEF(x)                             */
    pcl_coef_t release_curve;  /* 0 (linear) .. 1; PCL_COEF(x)                             */
    int        run_through;    /* non-zero: a gate-off during attack/decay waits for sustain */
} pcl_adsr_config_t;

/* Private: the constants of one stage. */
#if PCL_SAMPLE_TYPE == PCL_FLOAT
typedef float pcl_adsr_k_t;
#else
typedef uint32_t pcl_adsr_k_t;
#endif

typedef struct
{
    uint32_t     ticks;  /* control periods in the stage, >= 1                      */
    uint32_t     inc;    /* progress per period, 2^32 / ticks                       */
    pcl_adsr_k_t h;      /* exponent scale (half the curve's octaves); 0 = linear   */
    pcl_adsr_k_t e1;     /* 2^(-2 h), the exponential's value at the stage end      */
    pcl_adsr_k_t invden; /* 1 / (1 - e1)                                            */
} pcl_adsr_stage_cfg_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_adsr_stage_cfg_t cfg[3]; /* attack, decay, release: what the setters last set        */
    pcl_adsr_stage_cfg_t act;    /* the running stage's constants, copied at stage start     */
    pcl_smooth_t         sustain;
    uint32_t             sample_rate_hz;
    uint32_t             x;          /* progress through the running stage, 2^32 = done       */
    uint32_t             ticks_left; /* control periods left in the running stage             */
    pcl_coef_t           v0;         /* level at the end of the previous period               */
    pcl_coef_t           start;      /* level when the running stage began                    */
    pcl_coef_t           cur;        /* output so far in the current period                   */
    pcl_coef_t           step;       /* per-sample increment in the current period            */
    uint8_t              stage;
    uint8_t              pos;             /* samples already output in the current period     */
    uint8_t              gate;            /* logical gate                                     */
    uint8_t              schmitt;         /* Schmitt-trigger state of the gate buffer         */
    uint8_t              rose, fell;      /* edges seen since the last control point          */
    uint8_t              run_through;     /* config                                           */
    uint8_t              release_pending; /* run_through: release when sustain is reached     */
} pcl_adsr_t;

/*
 * Initializes `adsr` from `cfg`, idle at level 0 with the gate low. Returns PCL_ERR_NULL_POINTER
 * for NULL arguments and PCL_ERR_OUT_OF_RANGE for any field outside the ranges above (NaN
 * included); `adsr` is left untouched on error. Control-thread call.
 */
pcl_status_t pcl_adsr_init(pcl_adsr_t *adsr, const pcl_adsr_config_t *cfg);

/* Back to IDLE at level 0 with the gate low; the configuration is kept. Control-thread call. */
void pcl_adsr_reset(pcl_adsr_t *adsr);

/*
 * Setters. They return PCL_ERR_NULL_POINTER / PCL_ERR_OUT_OF_RANGE as in the config and leave the
 * module unchanged on error. Times and curves apply from the next time that stage begins; the
 * sustain level glides.
 */
pcl_status_t pcl_adsr_set_attack_ms(pcl_adsr_t *adsr, uint32_t ms);
pcl_status_t pcl_adsr_set_decay_ms(pcl_adsr_t *adsr, uint32_t ms);
pcl_status_t pcl_adsr_set_release_ms(pcl_adsr_t *adsr, uint32_t ms);
pcl_status_t pcl_adsr_set_sustain(pcl_adsr_t *adsr, pcl_coef_t sustain);
pcl_status_t pcl_adsr_set_attack_curve(pcl_adsr_t *adsr, pcl_coef_t curve);
pcl_status_t pcl_adsr_set_decay_curve(pcl_adsr_t *adsr, pcl_coef_t curve);
pcl_status_t pcl_adsr_set_release_curve(pcl_adsr_t *adsr, pcl_coef_t curve);

/* Sets run_through (non-zero: a gate-off before SUSTAIN is applied when SUSTAIN is reached). */
void pcl_adsr_set_run_through(pcl_adsr_t *adsr, int run_through);

/* Gate on or off, from the control thread; applied at the next control point. */
void pcl_adsr_gate(pcl_adsr_t *adsr, int on);

/*
 * The stage the most recent control period belonged to. A voice allocator can free the voice once
 * this is PCL_ADSR_IDLE: the level has then been exactly 0 for a whole control period.
 */
pcl_adsr_stage_t pcl_adsr_stage(const pcl_adsr_t *adsr);

/*
 * Writes `frames` samples (any count >= 1) of level to `out` (required). The gate comes from:
 *   - `gate` (may be NULL): a Schmitt trigger, on when the signal rises through 0.5, off when it
 * falls below 0.25; its state persists across calls, and a buffer that starts high after init/reset
 *     turns the gate on at its first sample;
 *   - `events` / `num_events` (may be NULL / 0, sorted by offset): PCL_EVT_NOTE_ON turns the gate
 * on, PCL_EVT_NOTE_OFF turns it off (note numbers are ignored: voice assignment is the caller's
 * job); other types and offsets >= frames are ignored. At one sample, events are applied before the
 * buffer. `gate` may alias `out`. Does not allocate or block, and does not check its arguments.
 */
void pcl_adsr_process(pcl_adsr_t *adsr, pcl_sample_t *out, const pcl_sample_t *gate,
    const pcl_event_t *events, size_t num_events, size_t frames);

PCL_END_DECLS

#endif /* PCL_ADSR_H */
