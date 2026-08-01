#include "transport_ring.h"

#include <stdint.h>
#include <string.h>

bool transport_ring_attach(transport_ring_t *ring, uint8_t *data, size_t capacity) {
    if (ring == NULL || data == NULL || capacity == 0 || capacity > UINT32_MAX) return false;
    ring->data = data;
    ring->capacity = capacity;
    ring->read_count = 0;
    ring->write_count = 0;
    return true;
}

void transport_ring_reset(transport_ring_t *ring) {
    if (ring == NULL) return;
    ring->read_count = 0;
    ring->write_count = 0;
}

bool transport_ring_is_attached(const transport_ring_t *ring) {
    return ring != NULL && ring->data != NULL && ring->capacity != 0;
}

size_t transport_ring_readable(const transport_ring_t *ring) {
    uint32_t read_count, write_count, used;
    if (!transport_ring_is_attached(ring)) return 0;
    read_count = ring->read_count;
    TRANSPORT_RING_MEMORY_BARRIER();
    write_count = ring->write_count;
    used = write_count - read_count;
    return used <= ring->capacity ? (size_t)used : ring->capacity;
}

size_t transport_ring_writable(const transport_ring_t *ring) {
    return transport_ring_is_attached(ring) ? ring->capacity - transport_ring_readable(ring) : 0;
}

size_t transport_ring_write(transport_ring_t *ring, const uint8_t *source, size_t length) {
    uint32_t write_count;
    size_t offset, first, available;
    if (!transport_ring_is_attached(ring) || source == NULL || length == 0) return 0;
    available = transport_ring_writable(ring);
    if (length > available) length = available;
    write_count = ring->write_count;
    offset = (size_t)(write_count % ring->capacity);
    first = ring->capacity - offset;
    if (first > length) first = length;
    memcpy(ring->data + offset, source, first);
    if (first < length) memcpy(ring->data, source + first, length - first);
    TRANSPORT_RING_MEMORY_BARRIER();
    ring->write_count = write_count + (uint32_t)length;
    return length;
}

size_t transport_ring_peek(const transport_ring_t *ring, uint8_t *destination, size_t length) {
    uint32_t read_count;
    size_t offset, first, available;
    if (!transport_ring_is_attached(ring) || destination == NULL || length == 0) return 0;
    available = transport_ring_readable(ring);
    if (length > available) length = available;
    read_count = ring->read_count;
    offset = (size_t)(read_count % ring->capacity);
    first = ring->capacity - offset;
    if (first > length) first = length;
    memcpy(destination, ring->data + offset, first);
    if (first < length) memcpy(destination + first, ring->data, length - first);
    return length;
}

size_t transport_ring_discard(transport_ring_t *ring, size_t length) {
    uint32_t read_count;
    size_t available;
    if (!transport_ring_is_attached(ring) || length == 0) return 0;
    available = transport_ring_readable(ring);
    if (length > available) length = available;
    read_count = ring->read_count;
    TRANSPORT_RING_MEMORY_BARRIER();
    ring->read_count = read_count + (uint32_t)length;
    return length;
}

size_t transport_ring_read(transport_ring_t *ring, uint8_t *destination, size_t length) {
    size_t copied = transport_ring_peek(ring, destination, length);
    transport_ring_discard(ring, copied);
    return copied;
}
