#include "transport_event.h"

static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);
}

static void write_u16_le(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

size_t transport_event_record_size(uint16_t data_length) {
    return TRANSPORT_EVENT_HEADER_SIZE + (size_t)data_length;
}

bool transport_event_write_header(uint8_t *buffer, size_t capacity,
        const transport_event_fields_t *fields) {
    if (buffer == NULL || fields == NULL ||
            capacity < transport_event_record_size(fields->data_length)) {
        return false;
    }

    buffer[TRANSPORT_EVENT_OFFSET_TYPE] = fields->type;
    buffer[TRANSPORT_EVENT_OFFSET_FLAGS] = fields->flags;
    buffer[TRANSPORT_EVENT_OFFSET_OBJECT_ID] = fields->object_id;
    buffer[TRANSPORT_EVENT_OFFSET_SOURCE_ID] = fields->source_id;
    write_u16_le(buffer + TRANSPORT_EVENT_OFFSET_TRANSACTION_ID,
        fields->transaction_id);
    write_u16_le(buffer + TRANSPORT_EVENT_OFFSET_DATA_LENGTH,
        fields->data_length);
    write_u32_le(buffer + TRANSPORT_EVENT_OFFSET_VALUE0, fields->value0);
    write_u32_le(buffer + TRANSPORT_EVENT_OFFSET_VALUE1, fields->value1);
    return true;
}

bool transport_event_read_header(const uint8_t *buffer, size_t length,
        transport_event_fields_t *fields) {
    uint16_t data_length;
    if (buffer == NULL || fields == NULL || length < TRANSPORT_EVENT_HEADER_SIZE) {
        return false;
    }
    data_length = transport_event_get_data_length(buffer);
    if (length < transport_event_record_size(data_length)) {
        return false;
    }

    fields->type = transport_event_get_type(buffer);
    fields->flags = transport_event_get_flags(buffer);
    fields->object_id = transport_event_get_object_id(buffer);
    fields->source_id = transport_event_get_source_id(buffer);
    fields->transaction_id = transport_event_get_transaction_id(buffer);
    fields->data_length = data_length;
    fields->value0 = transport_event_get_value0(buffer);
    fields->value1 = transport_event_get_value1(buffer);
    return true;
}

uint8_t transport_event_get_type(const uint8_t *buffer) {
    return buffer[TRANSPORT_EVENT_OFFSET_TYPE];
}

uint8_t transport_event_get_flags(const uint8_t *buffer) {
    return buffer[TRANSPORT_EVENT_OFFSET_FLAGS];
}

uint8_t transport_event_get_object_id(const uint8_t *buffer) {
    return buffer[TRANSPORT_EVENT_OFFSET_OBJECT_ID];
}

uint8_t transport_event_get_source_id(const uint8_t *buffer) {
    return buffer[TRANSPORT_EVENT_OFFSET_SOURCE_ID];
}

uint16_t transport_event_get_transaction_id(const uint8_t *buffer) {
    return read_u16_le(buffer + TRANSPORT_EVENT_OFFSET_TRANSACTION_ID);
}

uint16_t transport_event_get_data_length(const uint8_t *buffer) {
    return read_u16_le(buffer + TRANSPORT_EVENT_OFFSET_DATA_LENGTH);
}

uint32_t transport_event_get_value0(const uint8_t *buffer) {
    return read_u32_le(buffer + TRANSPORT_EVENT_OFFSET_VALUE0);
}

uint32_t transport_event_get_value1(const uint8_t *buffer) {
    return read_u32_le(buffer + TRANSPORT_EVENT_OFFSET_VALUE1);
}

uint8_t *transport_event_get_data(uint8_t *buffer) {
    return buffer + TRANSPORT_EVENT_HEADER_SIZE;
}

const uint8_t *transport_event_get_const_data(const uint8_t *buffer) {
    return buffer + TRANSPORT_EVENT_HEADER_SIZE;
}

static bool header_available(const uint8_t *record, size_t length, const void *out) {
    return record != NULL && out != NULL && length >= TRANSPORT_EVENT_HEADER_SIZE;
}

bool transport_event_type(const uint8_t *record, size_t length, uint8_t *value) {
    if (!header_available(record, length, value)) return false;
    *value = transport_event_get_type(record);
    return true;
}

bool transport_event_flags(const uint8_t *record, size_t length, uint8_t *value) {
    if (!header_available(record, length, value)) return false;
    *value = transport_event_get_flags(record);
    return true;
}

bool transport_event_object_id(const uint8_t *record, size_t length, uint8_t *value) {
    if (!header_available(record, length, value)) return false;
    *value = transport_event_get_object_id(record);
    return true;
}

bool transport_event_source(const uint8_t *record, size_t length, uint8_t *value) {
    if (!header_available(record, length, value)) return false;
    *value = transport_event_get_source_id(record);
    return true;
}

bool transport_event_transaction(const uint8_t *record, size_t length, uint16_t *value) {
    if (!header_available(record, length, value)) return false;
    *value = transport_event_get_transaction_id(record);
    return true;
}

bool transport_event_data_length(const uint8_t *record, size_t length, uint16_t *value) {
    if (!header_available(record, length, value)) return false;
    *value = transport_event_get_data_length(record);
    return length >= transport_event_record_size(*value);
}

bool transport_event_value0(const uint8_t *record, size_t length, uint32_t *value) {
    if (!header_available(record, length, value)) return false;
    *value = transport_event_get_value0(record);
    return true;
}

bool transport_event_value1(const uint8_t *record, size_t length, uint32_t *value) {
    if (!header_available(record, length, value)) return false;
    *value = transport_event_get_value1(record);
    return true;
}

bool transport_event_data(const uint8_t *record, size_t length,
        const uint8_t **data, size_t *data_length) {
    uint16_t encoded_length;
    if (data == NULL || data_length == NULL ||
            !transport_event_data_length(record, length, &encoded_length)) {
        return false;
    }
    *data = transport_event_get_const_data(record);
    *data_length = encoded_length;
    return true;
}
