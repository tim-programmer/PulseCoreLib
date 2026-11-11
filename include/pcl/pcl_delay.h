/*
 * pcl_delay.h - modulatable delay line with feedback, tone, saturation and dry/wet mix, in
 * caller-provided memory. Design note: docs/modules/delay.md.
 *
 * One code path for echo, slapback, chorus, flanger and vibrato: a delay whose time can be
 * modulated. The delay time moves smoothly (tape-style: a moving time bends the pitch) and is read
 * with linear or 4-point cubic Hermite interpolation.
 *
 *     size_t bytes = pcl_delay_required_bytes(&cfg);     -> caller provides `bytes` of memory
 *     pcl_delay_init(&dl, &cfg, buffer, bytes);
 *
 * Rate: audio rate. The modulation input and the time glide are evaluated at control rate (every
 * PCL_CONTROL_DIV samples, counted from the start of the stream) and the read position moves
 * linearly in between. Feedback and mix are smoothed per sample over ~PCL_SMOOTH_MS.
 *
 * The minimum delay is 1 sample with linear interpolation and 2 samples with Hermite (the line is
 * read before the current sample is written, because that sample depends on the feedback).
 */
#ifndef PCL_DELAY_H
#define PCL_DELAY_H

#include "pcl/pcl_smooth.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

/* Times in milliseconds use the pcl_hz_t representation (float, or Q16.16 in fixed point). */
typedef pcl_hz_t pcl_ms_t;
#define PCL_MS(x) PCL_HZ(x)

#define PCL_DELAY_MAX_MS 60000u /* largest max_delay_ms accepted (subject to the line limit) */
#define PCL_DELAY_MAX_GLIDE_MS 10000u
#define PCL_DELAY_TONE_MIN_HZ 20    /* tone_hz: 0 (off) or 20 .. 0.45 * sample rate             */
#define PCL_DELAY_FEEDBACK_MAX 0.99 /* |feedback| <= this                                      */

typedef enum
{
    PCL_DELAY_LINEAR = 0, /* 2 taps: cheap; a mild low-pass at fractional delays    */
    PCL_DELAY_HERMITE,    /* 4-point cubic Hermite (Catmull-Rom): for modulation      */
    PCL_DELAY_INTERP_COUNT
} pcl_delay_interp_t;

typedef enum
{
    PCL_DELAY_STORE_NATIVE = 0, /* pcl_sample_t                                         */
    PCL_DELAY_STORE_Q15,        /* int16_t in every build: half the RAM of float / Q31  */
    PCL_DELAY_STORE_COUNT
} pcl_delay_storage_t;

typedef struct
{
    uint32_t   sample_rate_hz; /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ          */
    uint32_t   max_delay_ms;   /* 1 .. PCL_DELAY_MAX_MS, and at most 2^20 - 8 samples        */
    pcl_ms_t   time_ms;        /* 0 .. max_delay_ms; PCL_MS(x)                              */
    pcl_ms_t   mod_depth_ms;   /* time swing per unit of time_mod, 0 .. max_delay_ms        */
    uint32_t   glide_ms;       /* time-change glide, 0 (jump) .. PCL_DELAY_MAX_GLIDE_MS     */
    pcl_coef_t feedback;       /* -0.99 .. 0.99; negative inverts the repeats; PCL_COEF(x) */
    pcl_hz_t   tone_hz;        /* low-pass in the loop: 0 = off, else 20 .. 0.45 fs; PCL_HZ */
    int        saturate;       /* non-zero: tanh on the feedback signal                     */
    pcl_coef_t mix;            /* 0 (dry) .. 1 (wet only), linear; PCL_COEF(x)              */
    pcl_delay_interp_t  interp;
    pcl_delay_storage_t storage;
} pcl_delay_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    void        *line;     /* the caller's buffer, aligned for the storage type          */
    uint32_t     length;   /* samples in the line                                        */
    uint32_t     w;        /* next write index                                           */
    uint32_t     d;        /* read delay in use, Q20.12 samples                          */
    uint32_t     d_end;    /* delay at the end of the control period                     */
    int32_t      d_step;   /* per-sample change of d during the period                   */
    uint32_t     d_min;    /* 1 sample (linear) or 2 (Hermite), Q20.12                    */
    uint32_t     d_max;    /* max_delay_ms in samples, Q20.12                             */
    uint32_t     time_cur; /* glided base delay, Q20.12                                   */
    uint32_t     time_target;
    int64_t      time_step;   /* per control period while gliding                           */
    uint32_t     glide_left;  /* control periods left in the glide                          */
    uint32_t     glide_ticks; /* control periods per glide                                  */
    uint32_t     mod_depth;   /* Q20.12 samples                                             */
    uint32_t     max_ms;      /* max_delay_ms, for validating the setters                  */
    pcl_smooth_t feedback;
    pcl_smooth_t mix;
    pcl_coef_t   tone_g; /* one-pole G = g / (1 + g), Q2.30                              */
    pcl_wide_t   tone_s; /* one-pole state, Q4.28                                        */
    uint32_t     sample_rate_hz;
    uint8_t      pos; /* samples already output in the current control period          */
    uint8_t      interp;
    uint8_t      storage;
    uint8_t      saturate;
    uint8_t      tone_on;
} pcl_delay_t;

/*
 * Bytes of caller memory `cfg` needs (including slack for alignment, so any byte buffer works), or
 * 0 if `cfg` is NULL or invalid. Not for the audio thread.
 */
size_t pcl_delay_required_bytes(const pcl_delay_config_t *cfg);

/*
 * Initializes `dl` on `buffer` (`bytes` long) and clears the line. Returns PCL_ERR_NULL_POINTER for
 * NULL arguments, PCL_ERR_OUT_OF_RANGE for an invalid config (NaN included), and
 * PCL_ERR_BUFFER_TOO_SMALL if `bytes` < pcl_delay_required_bytes(cfg); `dl` is untouched on error.
 * The buffer must stay valid and unused by anything else while `dl` is in use. Not for the audio
 * thread.
 */
pcl_status_t pcl_delay_init(
    pcl_delay_t *dl, const pcl_delay_config_t *cfg, void *buffer, size_t bytes);

/* Clears the line and the tone state, cancels every glide and ramp. O(line length). */
void pcl_delay_reset(pcl_delay_t *dl);

/*
 * Setters: validate (errors leave the delay unchanged), take effect at the next control point, and
 * are control-thread calls. The time glides over glide_ms (a new glide restarts from where the
 * current one is); feedback and mix ramp over ~PCL_SMOOTH_MS; the rest switch at once.
 */
pcl_status_t pcl_delay_set_time(pcl_delay_t *dl, pcl_ms_t time_ms);
pcl_status_t pcl_delay_set_glide(pcl_delay_t *dl, uint32_t glide_ms);
pcl_status_t pcl_delay_set_mod_depth(pcl_delay_t *dl, pcl_ms_t depth_ms);
pcl_status_t pcl_delay_set_feedback(pcl_delay_t *dl, pcl_coef_t feedback);
pcl_status_t pcl_delay_set_tone(pcl_delay_t *dl, pcl_hz_t tone_hz);
pcl_status_t pcl_delay_set_mix(pcl_delay_t *dl, pcl_coef_t mix);
pcl_status_t pcl_delay_set_interp(pcl_delay_t *dl, pcl_delay_interp_t interp);
void         pcl_delay_set_saturate(pcl_delay_t *dl, int saturate);

/*
 * Processes `frames` samples (any count >= 1). `time_mod` is optional (NULL = unmodulated):
 * normalized [-1, 1], times mod_depth_ms, sampled at control rate; the delay is clamped to
 * [minimum, max_delay_ms]. `in` and `out` may alias. Fixed point saturates the line and the output;
 * float does not clip them (|feedback| < 1 keeps the line bounded). Does not allocate, block or
 * check arguments.
 */
void pcl_delay_process(pcl_delay_t *dl, const pcl_sample_t *in, pcl_sample_t *out,
    const pcl_sample_t *time_mod, size_t frames);

PCL_END_DECLS

#endif /* PCL_DELAY_H */
