/*
 * pcl_event.h - sample-accurate events passed to event-driven modules.
 *
 * A block of events is an array sorted by `offset` (the sample index within the block
 * the event takes effect at, 0 .. frames-1). Modules consume such arrays; producers
 * (arpeggiators, sequencers, clocks) fill caller-provided arrays.
 *
 * Producers write events in this order at one offset: STOP, NOTE_OFF, START, CLOCK, NOTE_ON. So
 * a note is released before the next one starts, and START reaches a consumer before the CLOCK
 * pulse that is the first of the run. A block is at most 65536 samples (offset is 16 bit).
 * Consumers apply events in array order and ignore the types they do not know.
 *
 * NOTE_ON with velocity 0 is a NOTE_OFF, as in MIDI (the event modules accept both).
 */
#ifndef PCL_EVENT_H
#define PCL_EVENT_H

#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

typedef enum
{
    PCL_EVT_NOTE_ON = 1,
    PCL_EVT_NOTE_OFF = 2,
    PCL_EVT_CLOCK = 3, /* one pulse of the clock (24 per quarter note by default)             */
    PCL_EVT_START = 4, /* the transport starts: sequences go back to their first step        */
    PCL_EVT_STOP = 5   /* the transport stops: sounding notes are released                   */
} pcl_event_type_t;

typedef struct
{
    uint16_t offset;   /* sample offset within the current block */
    uint8_t  type;     /* a pcl_event_type_t value               */
    uint8_t  note;     /* MIDI note number, where it applies     */
    uint8_t  velocity; /* 0 .. 127, where it applies             */
} pcl_event_t;

PCL_END_DECLS

#endif /* PCL_EVENT_H */
