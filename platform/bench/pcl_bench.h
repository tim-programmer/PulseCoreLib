/*
 * pcl_bench.h - deterministic test vectors and cycle benchmark (oscillator, noise,
 * sample-and-hold, VCA, LFO, ADSR, math, state-variable filter, biquad).
 *
 * The same code runs on a target (where it is timed with a hardware cycle counter) and on the
 * host (where it produces the golden reference), so the two outputs can be compared sample
 * for sample. It does no I/O: results go out through a caller-supplied sink function.
 * Lives in platform/ rather than src/ because it is test harness, not library.
 *
 * Wire format of one case, little-endian:
 *   'C'  id:u8  frames:u32  cycles_total:u32  cycles_min_block:u32  cycles_max_block:u32
 *   payload (frames * sizeof(pcl_sample_t) bytes)  crc32(payload):u32
 */
#ifndef PCL_BENCH_H
#define PCL_BENCH_H

#include "pcl/pcl.h"

PCL_BEGIN_DECLS

#define PCL_BENCH_FRAMES 2048u /* samples rendered per case                         */
#define PCL_BENCH_BLOCK 256u   /* frames per process call, and per timing           */

typedef uint32_t (*pcl_bench_clock_fn)(void); /* free-running cycle counter */
typedef void (*pcl_bench_sink_fn)(const void *data, size_t bytes);

typedef struct
{
    uint32_t cycles_total;
    uint32_t cycles_min_block;
    uint32_t cycles_max_block;
} pcl_bench_result_t;

size_t      pcl_bench_case_count(void);
const char *pcl_bench_case_name(size_t id);

/*
 * Renders case `id` into out[PCL_BENCH_FRAMES]. If `clock` is non-NULL every
 * process call is timed with it (input preparation is not).
 */
pcl_status_t pcl_bench_run(
    size_t id, pcl_sample_t *out, pcl_bench_clock_fn clock, pcl_bench_result_t *result);

/* Runs a case and writes it to `sink` in the wire format above. */
pcl_status_t pcl_bench_emit(size_t id, pcl_bench_clock_fn clock, pcl_bench_sink_fn sink);

/* "PCLBENCH1 type=Q15 sample_bytes=2 cases=57\n" */
void pcl_bench_emit_banner(pcl_bench_sink_fn sink);

PCL_END_DECLS

#endif /* PCL_BENCH_H */
