/*
 * pcl_tables.h - constant lookup tables shared by modules (internal; not installed).
 *
 * Generated data lives in pcl_tables.c (see tools/gen_tables.c). All tables are const
 * so they stay in flash.
 */
#ifndef PCL_TABLES_H
#define PCL_TABLES_H

#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

#define PCL_QUARTER_SINE_SEGMENTS 256u
#define PCL_EXP2_SEGMENTS 256u
#define PCL_LOG2_SEGMENTS 256u
#define PCL_TANH_SEGMENTS 384u
#define PCL_TANH_STEPS_PER_UNIT 64u /* tanh table step 1/64; range [0, 6] */

/*
 * sin(pi/2 * i/256) for i = 0..256, in the sample format (full scale = 1.0, stored as
 * the largest representable value). 257 entries: 514 B in Q15 builds, 1028 B otherwise.
 */
extern const pcl_sample_t pcl_sine_quarter_tab[PCL_QUARTER_SINE_SEGMENTS + 1u];

/*
 * (2^(i/256) - 1) * 65536 for i = 0..256, as uint16_t (the last entry saturates 65536
 * to 65535). 257 entries, 514 B.
 */
extern const uint16_t pcl_exp2_tab[PCL_EXP2_SEGMENTS + 1u];

/*
 * log2(1 + i/256) * 65536 for i = 0..256, as uint16_t (the last entry saturates 65536 to 65535).
 * 257 entries, 514 B.
 */
extern const uint16_t pcl_log2_tab[PCL_LOG2_SEGMENTS + 1u];

/*
 * tanh(i/64) for i = 0..384 (x from 0 to 6) in the sample format (full scale = 1.0, stored as the
 * largest representable value). 385 entries: 770 B in Q15 builds, 1540 B otherwise.
 */
extern const pcl_sample_t pcl_tanh_tab[PCL_TANH_SEGMENTS + 1u];

PCL_END_DECLS

#endif /* PCL_TABLES_H */
