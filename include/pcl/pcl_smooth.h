/*
 * pcl_smooth.h - linear parameter smoothing (implementation in src/core/).
 *
 * Lives in include/ rather than src/ because modules embed pcl_smooth_t in their
 * caller-declared state structs. Treat the fields as private.
 *
 * A change of target starts a straight-line ramp from the current value that
 * lands exactly on the target after `ramp_samples` calls to pcl_smooth_next().
 * Unlike a one-pole smoother there is no asymptotic tail, so no denormals and no
 * residual error. Runs at audio rate. Fixed-point builds use only integer maths.
 */
#ifndef PCL_SMOOTH_H
#define PCL_SMOOTH_H

#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

/* Default ramp time for parameter changes, in milliseconds (integer, 1 .. 5000). */
#ifndef PCL_SMOOTH_MS
#define PCL_SMOOTH_MS 5
#endif

typedef struct
{
    /* private - callers must not touch fields directly */
    pcl_coef_t current;
    pcl_coef_t target;
    pcl_coef_t step;         /* per-sample increment; fixed point: truncated toward zero */
    uint32_t   ramp_samples; /* length of a full ramp, >= 1                              */
    uint32_t   remaining;    /* samples left in the current ramp, 0 when settled         */
} pcl_smooth_t;

/*
 * Number of samples that PCL_SMOOTH_MS spans at the given sample rate (>= 1).
 * sample_rate_hz must be within PCL_SAMPLE_RATE_MIN_HZ..PCL_SAMPLE_RATE_MAX_HZ.
 */
uint32_t pcl_smooth_ramp_samples(uint32_t sample_rate_hz);

/*
 * Starts settled at `value`. ramp_samples is the length of every later ramp;
 * 0 or 1 makes set_target jump immediately.
 */
void pcl_smooth_init(pcl_smooth_t *s, pcl_coef_t value, uint32_t ramp_samples);

/* Cancels any ramp in progress and jumps to the current target. */
void pcl_smooth_reset(pcl_smooth_t *s);

/*
 * Ramps from the current value to `target` over the full ramp length, even if a
 * ramp is already in progress. Control-thread call; not for use inside a block.
 */
void pcl_smooth_set_target(pcl_smooth_t *s, pcl_coef_t target);

/* Returns 1 when no ramp is in progress. */
PCL_INLINE int pcl_smooth_is_settled(const pcl_smooth_t *s) { return s->remaining == 0u; }

/* Advances one sample and returns the smoothed value. Audio rate; allocation-free. */
PCL_INLINE pcl_coef_t pcl_smooth_next(pcl_smooth_t *s)
{
    if (s->remaining != 0u)
    {
        s->remaining--;
        /* Snap on the last step so the ramp lands exactly, whatever the rounding. */
        s->current = (s->remaining == 0u) ? s->target : (pcl_coef_t)(s->current + s->step);
    }
    return s->current;
}

PCL_END_DECLS

#endif /* PCL_SMOOTH_H */
