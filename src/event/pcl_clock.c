/*
 * pcl_clock.c - see include/pcl/pcl_clock.h and docs/modules/clock.md.
 *
 * Integer arithmetic only. The period is samples per pulse in Q32.32; pulse k of a run lands in
 * the sample floor(k * period), so the error never accumulates.
 */
#include "pcl/pcl_clock.h"

#include "core/pcl_evbuf.h"

#define CLOCK_FRAC_BITS 32
#define CLOCK_BPM_RAW_SHIFT 16 /* Q16.16 beats per minute */
#define CLOCK_SECONDS_PER_MINUTE 60u
#define CLOCK_MAX_FRAMES 65536u /* offset is 16 bit */

#if PCL_SAMPLE_TYPE == PCL_FLOAT
static int clock_bpm_to_raw(pcl_hz_t bpm, uint32_t *raw)
{
    /* written positively so that NaN fails */
    if (!(bpm >= (float)PCL_CLOCK_BPM_MIN && bpm <= (float)PCL_CLOCK_BPM_MAX))
        return 0;
    *raw = (uint32_t)(bpm * 65536.0f + 0.5f);
    return 1;
}
#else
static int clock_bpm_to_raw(pcl_hz_t bpm, uint32_t *raw)
{
    if (bpm < PCL_HZ(PCL_CLOCK_BPM_MIN) || bpm > PCL_HZ(PCL_CLOCK_BPM_MAX))
        return 0;
    *raw = bpm;
    return 1;
}
#endif

/*
 * Samples per pulse, Q32.32: fs * 60 / (bpm * ppqn) = fs * 60 * 65536 / (bpm_raw * ppqn). The
 * numerator fits 64 bits but shifted by 32 it would not, so the fraction comes from a long division
 * in two 16-bit steps (the remainder is below the denominator, below 2^33).
 */
static uint64_t clock_period(uint32_t fs, uint32_t bpm_raw, uint32_t ppqn)
{
    const uint64_t num = (uint64_t)fs * CLOCK_SECONDS_PER_MINUTE * 65536u;
    const uint64_t den = (uint64_t)bpm_raw * ppqn;
    uint64_t       rem = num % den;
    uint64_t       frac = 0;
    unsigned       i;

    for (i = 0; i < 2u; i++)
    {
        rem <<= 16;
        frac = (frac << 16) | (rem / den);
        rem %= den;
    }
    return ((num / den) << CLOCK_FRAC_BITS) | frac;
}

/* The offsets are samples, so pulses must be at least one sample apart: the fastest pulse rate is
   999 bpm * 96 ppqn / 60 s, and the lowest sample rate is far above it. */
PCL_STATIC_ASSERT(PCL_SAMPLE_RATE_MIN_HZ *CLOCK_SECONDS_PER_MINUTE
        >= (uint32_t)PCL_CLOCK_BPM_MAX * PCL_CLOCK_PPQN_MAX,
    clock_pulses_at_least_one_sample_apart);

pcl_status_t pcl_clock_init(pcl_clock_t *clk, const pcl_clock_config_t *cfg)
{
    pcl_clock_t tmp;
    uint32_t    raw;

    if (clk == NULL || cfg == NULL)
        return PCL_ERR_NULL_POINTER;
    if (cfg->sample_rate_hz < PCL_SAMPLE_RATE_MIN_HZ || cfg->sample_rate_hz > PCL_SAMPLE_RATE_MAX_HZ
        || cfg->ppqn < 1u || cfg->ppqn > PCL_CLOCK_PPQN_MAX || !clock_bpm_to_raw(cfg->bpm, &raw))
        return PCL_ERR_OUT_OF_RANGE;
    tmp.period = clock_period(cfg->sample_rate_hz, raw, cfg->ppqn);
    tmp.next = 0;
    tmp.sample_rate_hz = cfg->sample_rate_hz;
    tmp.bpm_raw = raw;
    tmp.ppqn = cfg->ppqn;
    tmp.dropped = 0;
    tmp.running = 0;
    tmp.start_pending = cfg->running != 0;
    tmp.stop_pending = 0;
    *clk = tmp;
    return PCL_OK;
}

void pcl_clock_reset(pcl_clock_t *clk)
{
    clk->next = 0;
    clk->running = 0;
    clk->start_pending = 0;
    clk->stop_pending = 0;
}

pcl_status_t pcl_clock_set_bpm(pcl_clock_t *clk, pcl_hz_t bpm)
{
    uint32_t raw;

    if (clk == NULL)
        return PCL_ERR_NULL_POINTER;
    if (!clock_bpm_to_raw(bpm, &raw))
        return PCL_ERR_OUT_OF_RANGE;
    clk->bpm_raw = raw;
    clk->period = clock_period(clk->sample_rate_hz, raw, clk->ppqn);
    return PCL_OK;
}

pcl_status_t pcl_clock_set_ppqn(pcl_clock_t *clk, uint32_t ppqn)
{

    if (clk == NULL)
        return PCL_ERR_NULL_POINTER;
    if (ppqn < 1u || ppqn > PCL_CLOCK_PPQN_MAX)
        return PCL_ERR_OUT_OF_RANGE;
    clk->ppqn = ppqn;
    clk->period = clock_period(clk->sample_rate_hz, clk->bpm_raw, ppqn);
    return PCL_OK;
}

void pcl_clock_start(pcl_clock_t *clk)
{
    clk->start_pending = 1;
    clk->stop_pending = 0;
}

void pcl_clock_stop(pcl_clock_t *clk)
{
    clk->stop_pending = 1;
    clk->start_pending = 0;
}

int pcl_clock_is_running(const pcl_clock_t *clk) { return clk->running != 0; }

uint32_t pcl_clock_dropped(const pcl_clock_t *clk) { return clk->dropped; }

size_t pcl_clock_process(pcl_clock_t *clk, size_t frames, pcl_event_t *out, size_t capacity)
{
    const uint64_t block = (uint64_t)frames << CLOCK_FRAC_BITS;
    pcl_evbuf_t    b;

    b.out = out;
    b.capacity = capacity;
    b.count = 0;
    b.dropped = &clk->dropped;
    if (clk->stop_pending)
    {
        if (clk->running)
            pcl_evbuf_emit(&b, 0, PCL_EVT_STOP, 0, 0);
        clk->running = 0;
        clk->stop_pending = 0;
    }
    if (clk->start_pending)
    {
        pcl_evbuf_emit(&b, 0, PCL_EVT_START, 0, 0);
        clk->running = 1;
        clk->start_pending = 0;
        clk->next = 0; /* the first pulse is at once */
    }
    if (clk->running)
    {
        while (clk->next < block)
        {
            pcl_evbuf_emit(&b, (uint16_t)(clk->next >> CLOCK_FRAC_BITS), PCL_EVT_CLOCK, 0, 0);
            clk->next += clk->period;
        }
        clk->next -= block;
    }
    return b.count;
}

size_t pcl_clock_max_events(const pcl_clock_t *clk, size_t frames)
{
    /* Pulses at 0, period, 2 * period ... below `frames`: at most frames / period + 1 of them; plus
       a START or STOP (a start() cancels a pending stop() and the other way round). */
    return (size_t)(((uint64_t)frames << CLOCK_FRAC_BITS) / clk->period) + 2u;
}
