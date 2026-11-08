/*
 * pcl_evbuf.h - append to a caller-provided event buffer (internal; not part of the public API).
 *
 * The event modules write into a buffer of fixed capacity. An event that does not fit is dropped
 * and counted, never written past the end: the module's state moves on regardless, so a buffer that
 * is too small loses events but never corrupts memory or the module.
 */
#ifndef PCL_EVBUF_H
#define PCL_EVBUF_H

#include "pcl/pcl_event.h"

typedef struct
{
    pcl_event_t *out;
    size_t       capacity;
    size_t       count;
    uint32_t    *dropped;
} pcl_evbuf_t;

PCL_INLINE void pcl_evbuf_emit(
    pcl_evbuf_t *b, uint16_t offset, uint8_t type, uint8_t note, uint8_t velocity)
{
    if (b->count >= b->capacity)
    {
        (*b->dropped)++;
        return;
    }
    b->out[b->count].offset = offset;
    b->out[b->count].type = type;
    b->out[b->count].note = note;
    b->out[b->count].velocity = velocity;
    b->count++;
}

#endif /* PCL_EVBUF_H */
