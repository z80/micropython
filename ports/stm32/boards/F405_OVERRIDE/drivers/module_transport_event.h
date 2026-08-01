#ifndef MICROPY_INCLUDED_USERMOD_TRANSPORT_EVENT_H
#define MICROPY_INCLUDED_USERMOD_TRANSPORT_EVENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "py/obj.h"

typedef struct _mp_transport_event_obj_t {
    mp_obj_base_t base;
    mp_obj_t buffer_owner;
    uint8_t *buffer;
    size_t capacity;
    size_t length;
    bool valid;
} mp_transport_event_obj_t;

extern const mp_obj_type_t mp_transport_event_type;

/* Helpers used by module_transport_core.c.  These validate that event_in is
 * an Event instance.  set_length also validates the encoded event record. */
mp_transport_event_obj_t *mp_transport_event_get(mp_obj_t event_in);
void mp_transport_event_get_buffer(mp_obj_t event_in, uint8_t **buffer,
    size_t *capacity);
void mp_transport_event_invalidate(mp_obj_t event_in);
void mp_transport_event_set_length(mp_obj_t event_in, size_t length);

#endif /* MICROPY_INCLUDED_USERMOD_TRANSPORT_EVENT_H */
