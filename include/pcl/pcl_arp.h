/*
 * pcl_arp.h - arpeggiator: turns held notes and clock pulses into a stream of notes.
 *
 * Design note: docs/modules/arp.md.
 *
 * Input events (sorted by offset, as in pcl_event.h):
 *   NOTE_ON  - a key goes down (velocity 0 counts as NOTE_OFF); it joins the held set, up to
 *              PCL_ARP_MAX_NOTES (further keys are ignored);
 *   NOTE_OFF - a key goes up; it leaves the set (unless latched), and if that empties the set
 *              the sounding note is released at once;
 *   CLOCK    - one pulse; a step starts on every `division`-th pulse and the note is released
 *              `gate` pulses later, so every note time is the offset of a clock event;
 *   START    - the pulse count and the pattern position go back to 0, sounding notes are released;
 *   STOP     - sounding notes are released; nothing else changes.
 * Other types are ignored and nothing is passed through.
 *
 * Output: NOTE_ON / NOTE_OFF at the offsets of the events that caused them, sorted, a NOTE_OFF
 * before a NOTE_ON at the same offset. A step that fires while the previous note still sounds
 * (gate >= division) releases it first. The velocity is that of the key the note came from.
 * Notes above 127 (a high key plus octaves) are clamped to 127.
 *
 * Rate: event rate. Memory: the caller's pcl_arp_t, no heap; loops are bounded by
 * PCL_ARP_MAX_NOTES.
 */
#ifndef PCL_ARP_H
#define PCL_ARP_H

#include "pcl/pcl_event.h"
#include "pcl/pcl_types.h"

PCL_BEGIN_DECLS

#define PCL_ARP_MAX_NOTES 16
#define PCL_ARP_MAX_OCTAVES 4
#define PCL_ARP_MAX_DIVISION 96

typedef enum
{
    PCL_ARP_UP,     /* lowest to highest, then again                                      */
    PCL_ARP_DOWN,   /* highest to lowest                                                  */
    PCL_ARP_UPDOWN, /* up then down, the end notes played once per turn                   */
    PCL_ARP_UPDOWN_INCLUSIVE, /* up then down, the end notes played twice */
    PCL_ARP_RANDOM,    /* any note of the pattern, uniformly                                 */
    PCL_ARP_AS_PLAYED, /* in the order the keys went down                                    */
    PCL_ARP_CHORD      /* all held notes together on every step (octaves not applied)       */
} pcl_arp_mode_t;

typedef struct
{
    pcl_arp_mode_t mode;
    uint32_t octaves;  /* 1 .. PCL_ARP_MAX_OCTAVES: the pattern repeats this many octaves up  */
    uint32_t division; /* clock pulses per step, 1 .. PCL_ARP_MAX_DIVISION (6 at 24 ppqn is a
                          sixteenth note)                                                     */
    uint32_t gate;     /* clock pulses a note lasts, 1 .. 96; at or above `division` it lasts
                          until the next step (legato)                                        */
    int      latch;    /* non-zero: released keys stay in the set until a new chord is played */
    uint32_t seed;     /* RANDOM mode; 0 means PCL_NOISE_DEFAULT_SEED                         */
} pcl_arp_config_t;

typedef struct
{
    /* private - callers must not touch fields directly */
    uint8_t note[PCL_ARP_MAX_NOTES]; /* the held set, in the order the keys went down           */
    uint8_t vel[PCL_ARP_MAX_NOTES];
    uint8_t down[PCL_ARP_MAX_NOTES]; /* the key is physically down (latch keeps notes up)       */
    uint8_t on_note[PCL_ARP_MAX_NOTES]; /* sounding                                               */
    uint8_t n;
    uint8_t on_count;
    uint8_t mode;
    uint8_t octaves;
    uint8_t division;
    uint8_t gate;
    uint8_t latch;
    uint32_t pulse; /* clock pulses since START (or init/reset) */
    uint32_t pos; /* steps played                                                                */
    uint32_t rng;
    uint32_t dropped;
} pcl_arp_t;

/*
 * Initializes `arp` from `cfg` with an empty set. Returns PCL_ERR_NULL_POINTER for NULL arguments
 * and PCL_ERR_OUT_OF_RANGE for a field outside its range; `arp` is untouched on error.
 * Control-thread call.
 */
pcl_status_t pcl_arp_init(pcl_arp_t *arp, const pcl_arp_config_t *cfg);

/*
 * Empties the set and goes back to the first step. Sounding notes are NOT released (no event is
 * produced): send STOP through pcl_arp_process first if a voice may be holding one.
 */
void pcl_arp_reset(pcl_arp_t *arp);

/* Setters: the same ranges and errors as the config; unchanged on error. Control-thread calls. */
pcl_status_t pcl_arp_set_mode(pcl_arp_t *arp, pcl_arp_mode_t mode);
pcl_status_t pcl_arp_set_octaves(pcl_arp_t *arp, uint32_t octaves);
pcl_status_t pcl_arp_set_division(pcl_arp_t *arp, uint32_t division);
pcl_status_t pcl_arp_set_gate(pcl_arp_t *arp, uint32_t gate);

/* Latch on or off; turning it off drops the keys that are not physically down. */
void pcl_arp_set_latch(pcl_arp_t *arp, int latch);

/*
 * Consumes `n_in` input events (sorted; `in` may be NULL if 0) and writes the notes they cause to
 * `out` (room for `capacity`), returning how many. `in` and `out` must not overlap. Events that do
 * not fit are dropped and counted (pcl_arp_dropped); a step produces at most 2 * PCL_ARP_MAX_NOTES
 * events (CHORD), a stop or key release at most PCL_ARP_MAX_NOTES. Does not allocate or block, and
 * does not check its arguments.
 */
size_t pcl_arp_process(
    pcl_arp_t *arp, const pcl_event_t *in, size_t n_in, pcl_event_t *out, size_t capacity);

/* Number of keys in the held set. */
size_t pcl_arp_held(const pcl_arp_t *arp);

/* Events dropped so far for lack of room in the caller's buffer. */
uint32_t pcl_arp_dropped(const pcl_arp_t *arp);

PCL_END_DECLS

#endif /* PCL_ARP_H */
