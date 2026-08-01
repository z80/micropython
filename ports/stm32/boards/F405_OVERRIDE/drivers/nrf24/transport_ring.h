#ifndef TRANSPORT_RING_H
#define TRANSPORT_RING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef TRANSPORT_RING_MEMORY_BARRIER
#if defined(__GNUC__)
#define TRANSPORT_RING_MEMORY_BARRIER() __asm__ volatile ("" ::: "memory")
#else
#define TRANSPORT_RING_MEMORY_BARRIER() ((void)0)
#endif
#endif

/* Single-producer/single-consumer byte ring using caller-owned storage. */
typedef struct {
    uint8_t *data;
    size_t capacity;
    volatile uint32_t read_count;
    volatile uint32_t write_count;
} transport_ring_t;

bool transport_ring_attach(transport_ring_t *ring, uint8_t *data, size_t capacity);
void transport_ring_reset(transport_ring_t *ring);
bool transport_ring_is_attached(const transport_ring_t *ring);
size_t transport_ring_readable(const transport_ring_t *ring);
size_t transport_ring_writable(const transport_ring_t *ring);
size_t transport_ring_write(transport_ring_t *ring, const uint8_t *source, size_t length);
size_t transport_ring_peek(const transport_ring_t *ring, uint8_t *destination, size_t length);
size_t transport_ring_read(transport_ring_t *ring, uint8_t *destination, size_t length);
size_t transport_ring_discard(transport_ring_t *ring, size_t length);

#endif
