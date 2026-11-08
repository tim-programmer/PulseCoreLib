/*
 * pcl_clock.h - tempo clock: PCL_EVT_CLOCK pulses at sample-accurate offsets, with START / STOP.
 *
 * Design note: docs/modules/clock.md.
 *
 * The pulse period is kept as a fixed-point number of samples (Q32.32), so pulses are placed at
 * floor(k * period) with no accumulated rounding: a tempo that does not divide the sample rate (120
 * bpm at 44.1 kHz and 24 ppqn is 918.75 samples per pulse) still keeps exact long-term time. The
 * period is computed from integers only, so the float build uses no libm and the three builds agree
 * exactly.
 *
 * Rate: event rate. process() does work proportional to the number of pulses, not samples.
 */
#ifndef PCL_CLOCK_H
#define PCL_CLOCK_H

#include "pcl/pcl_event.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

#define PCL_CLOCK_BPM_MIN 10
#define PCL_CLOCK_BPM_MAX 999
#define PCL_CLOCK_PPQN_MAX 96 /* 24 is the MIDI clock; 96 resolves a 1/16 note into 24 pulses */

typedef struct
{
    uint32_t sample_rate_hz; /* PCL_SAMPLE_RATE_MIN_HZ .. PCL_SAMPLE_RATE_MAX_HZ */
    pcl_hz_t bpm;     /* beats (quarter notes) per minute, 10 .. 999; PCL_HZ(120.0). The pcl_hz_t
                         is a plain number here, not a frequency (Q16.16 in fixed point).      */
    uint32_t ppqn;    /* pulses per quarter note, 1 .. 96                                     */
    int      running; /* non-zero: the first block carries START                              */
} pcl_clock_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    uint64_t period; /* samples per pulse, Q32.32                                              */
    uint64_t next;   /* samples from the start of the next block to the next pulse, Q32.32     */
    uint32_t sample_rate_hz;
    uint32_t bpm_raw; /* bpm * 65536                                                            */
    uint32_t ppqn;
    uint32_t dropped; /* events that did not fit the caller's buffer                            */
    uint8_t  running;
    uint8_t  start_pending;
    uint8_t  stop_pending;
} pcl_clock_t;

/*
 * Initializes `clk` from `cfg`. Returns PCL_ERR_NULL_POINTER for NULL arguments and
 * PCL_ERR_OUT_OF_RANGE for a field outside its range (NaN included); `clk` is untouched on error.
 * Control-thread call.
 */
pcl_status_t pcl_clock_init(pcl_clock_t *clk, const pcl_clock_config_t *cfg);

/* Stops the clock and clears pending transport requests; tempo and resolution are kept. */
void pcl_clock_reset(pcl_clock_t *clk);

/*
 * Tempo and resolution. They apply from the pulse after the next one; pulses already scheduled keep
 * their time. The same errors as the config; unchanged on error. Control-thread calls.
 */
pcl_status_t pcl_clock_set_bpm(pcl_clock_t *clk, pcl_hz_t bpm);
pcl_status_t pcl_clock_set_ppqn(pcl_clock_t *clk, uint32_t ppqn);

/*
 * Transport. start() makes the next process() call begin with START and a pulse at offset 0 (and
 * restart the phase: the first pulse is at once). stop() makes it begin with STOP and then emit
 * nothing. Calling start() on a running clock restarts it; the last call before process() wins.
 */
void pcl_clock_start(pcl_clock_t *clk);
void pcl_clock_stop(pcl_clock_t *clk);

/* Non-zero while the clock emits pulses (after a start() has been processed). */
int pcl_clock_is_running(const pcl_clock_t *clk);

/*
 * Advances the clock by `frames` samples (1 .. 65536) and writes the events of that block, sorted
 * by offset, to `out` (room for `capacity`). Returns the number written. Events that do not fit are
 * dropped (and counted, see pcl_clock_dropped): size `capacity` with pcl_clock_max_events. Does not
 * allocate or block, and does not check its arguments.
 */
size_t pcl_clock_process(pcl_clock_t *clk, size_t frames, pcl_event_t *out, size_t capacity);

/* An upper bound on the events process() can write for `frames` samples at the current tempo. */
size_t pcl_clock_max_events(const pcl_clock_t *clk, size_t frames);

/* Events dropped so far for lack of room in the caller's buffer. */
uint32_t pcl_clock_dropped(const pcl_clock_t *clk);

PCL_END_DECLS

#endif /* PCL_CLOCK_H */
