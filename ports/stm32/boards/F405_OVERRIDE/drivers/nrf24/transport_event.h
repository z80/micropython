#ifndef TRANSPORT_EVENT_H
#define TRANSPORT_EVENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TRANSPORT_EVENT_LAYOUT_VERSION 1u
#define TRANSPORT_EVENT_HEADER_SIZE 16u

#define TRANSPORT_EVENT_OFFSET_TYPE 0u
#define TRANSPORT_EVENT_OFFSET_FLAGS 1u
#define TRANSPORT_EVENT_OFFSET_OBJECT_ID 2u
#define TRANSPORT_EVENT_OFFSET_SOURCE_ID 3u
#define TRANSPORT_EVENT_OFFSET_TRANSACTION_ID 4u
#define TRANSPORT_EVENT_OFFSET_DATA_LENGTH 6u
#define TRANSPORT_EVENT_OFFSET_VALUE0 8u
#define TRANSPORT_EVENT_OFFSET_VALUE1 12u

typedef enum {
    TRANSPORT_EVENT_NONE = 0,
    TRANSPORT_EVENT_COMMAND_READY = 1,
    TRANSPORT_EVENT_COMMAND_SENT = 2,
    TRANSPORT_EVENT_COMMAND_FAILED = 3,
    TRANSPORT_EVENT_PIPE_OPENED = 4,
    TRANSPORT_EVENT_PIPE_RX_DATA = 5,
    TRANSPORT_EVENT_PIPE_TX_SPACE = 6,
    TRANSPORT_EVENT_PIPE_CLOSED = 7,
    TRANSPORT_EVENT_PIPE_FAILED = 8,
    TRANSPORT_EVENT_NODE_FAILED = 9,
    TRANSPORT_EVENT_CORE_ERROR = 10,
    TRANSPORT_EVENT_REGISTRATION_RECEIVED = 11,
    TRANSPORT_EVENT_REGISTRATION_SENT = 12,
    TRANSPORT_EVENT_REGISTRATION_FAILED = 13
} transport_event_type_t;

typedef enum {
    TRANSPORT_EVENT_FLAG_NONE = 0,
    TRANSPORT_EVENT_FLAG_MORE_DATA = 1u << 0,
    TRANSPORT_EVENT_FLAG_STICKY = 1u << 1,
    TRANSPORT_EVENT_FLAG_TX = 1u << 2
} transport_event_flag_t;

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint8_t object_id;
    uint8_t source_id;
    uint16_t transaction_id;
    uint16_t data_length;
    uint32_t value0;
    uint32_t value1;
} transport_event_fields_t;

size_t transport_event_record_size(uint16_t data_length);
bool transport_event_write_header(uint8_t *buffer, size_t capacity,
    const transport_event_fields_t *fields);
bool transport_event_read_header(const uint8_t *buffer, size_t length,
    transport_event_fields_t *fields);

uint8_t transport_event_get_type(const uint8_t *buffer);
uint8_t transport_event_get_flags(const uint8_t *buffer);
uint8_t transport_event_get_object_id(const uint8_t *buffer);
uint8_t transport_event_get_source_id(const uint8_t *buffer);
uint16_t transport_event_get_transaction_id(const uint8_t *buffer);
uint16_t transport_event_get_data_length(const uint8_t *buffer);
uint32_t transport_event_get_value0(const uint8_t *buffer);
uint32_t transport_event_get_value1(const uint8_t *buffer);
uint8_t *transport_event_get_data(uint8_t *buffer);
const uint8_t *transport_event_get_const_data(const uint8_t *buffer);

/* Validated accessors used by language bindings. */
bool transport_event_type(const uint8_t *record, size_t length, uint8_t *value);
bool transport_event_flags(const uint8_t *record, size_t length, uint8_t *value);
bool transport_event_object_id(const uint8_t *record, size_t length, uint8_t *value);
bool transport_event_source(const uint8_t *record, size_t length, uint8_t *value);
bool transport_event_transaction(const uint8_t *record, size_t length, uint16_t *value);
bool transport_event_data_length(const uint8_t *record, size_t length, uint16_t *value);
bool transport_event_value0(const uint8_t *record, size_t length, uint32_t *value);
bool transport_event_value1(const uint8_t *record, size_t length, uint32_t *value);
bool transport_event_data(const uint8_t *record, size_t length,
    const uint8_t **data, size_t *data_length);

#endif
