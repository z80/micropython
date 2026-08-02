#ifndef TRANSPORT_CORE_H
#define TRANSPORT_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "nrf24.h"
#include "nrf24_regs.h"
#include "transport_event.h"
#include "transport_ring.h"

#ifndef TRANSPORT_CORE_COMMAND_SLOTS
#define TRANSPORT_CORE_COMMAND_SLOTS 2u
#endif
#ifndef TRANSPORT_CORE_PIPE_SLOTS
#define TRANSPORT_CORE_PIPE_SLOTS 2u
#endif
#ifndef TRANSPORT_CORE_EVENT_QUEUE_SIZE
#define TRANSPORT_CORE_EVENT_QUEUE_SIZE 8u
#endif
#ifndef TRANSPORT_CORE_IRQ_SERVICE_LIMIT
#define TRANSPORT_CORE_IRQ_SERVICE_LIMIT 8u
#endif
#ifndef TRANSPORT_CORE_CONTROL_TX_SLOTS
#define TRANSPORT_CORE_CONTROL_TX_SLOTS 4u
#endif
#ifndef TRANSPORT_CORE_CTS_TURNAROUND_MS
#define TRANSPORT_CORE_CTS_TURNAROUND_MS 2u
#endif
#if TRANSPORT_CORE_COMMAND_SLOTS == 0 || TRANSPORT_CORE_COMMAND_SLOTS > 255
#error "TRANSPORT_CORE_COMMAND_SLOTS must be in 1..255"
#endif
#if TRANSPORT_CORE_PIPE_SLOTS == 0 || TRANSPORT_CORE_PIPE_SLOTS > 255
#error "TRANSPORT_CORE_PIPE_SLOTS must be in 1..255"
#endif
#if TRANSPORT_CORE_EVENT_QUEUE_SIZE == 0 || TRANSPORT_CORE_EVENT_QUEUE_SIZE > 127
#error "TRANSPORT_CORE_EVENT_QUEUE_SIZE must be in 1..127"
#endif
#if TRANSPORT_CORE_CONTROL_TX_SLOTS == 0 || TRANSPORT_CORE_CONTROL_TX_SLOTS > 127
#error "TRANSPORT_CORE_CONTROL_TX_SLOTS must be in 1..127"
#endif
#define TRANSPORT_CORE_INVALID_ID 0xffu
#define TRANSPORT_CORE_RESTARTS_UNBOUNDED 0xffu

/* The wire header remains five bytes.  NRF payloads are fixed at 32 bytes;
   unused bytes after the logical CRC are zero-filled. */
#define TRANSPORT_WIRE_PROTOCOL_ID 0xa0u
#define TRANSPORT_WIRE_PROTOCOL_MASK 0xf0u
#define TRANSPORT_WIRE_TYPE_MASK 0x0fu
#define TRANSPORT_WIRE_HEADER_SIZE 5u
#define TRANSPORT_WIRE_CRC_SIZE 1u
#define TRANSPORT_WIRE_LAST_PACKET 0x80u
#define TRANSPORT_WIRE_RESERVED_FLAG 0x40u
#define TRANSPORT_WIRE_INTENT 0x20u
#define TRANSPORT_WIRE_LENGTH_MASK 0x1fu
#define TRANSPORT_WIRE_MAX_DATA \
    (NRF24_MAX_PAYLOAD - TRANSPORT_WIRE_HEADER_SIZE - TRANSPORT_WIRE_CRC_SIZE)
#define TRANSPORT_REGISTRATION_MAX_DATA 9u
#define TRANSPORT_CORE_CONTROL_RECORD_SIZE \
    (NRF24_MAX_PAYLOAD)

#define TRANSPORT_WIRE_COMMAND 1u
#define TRANSPORT_WIRE_COMMAND_REPLY 2u
#define TRANSPORT_WIRE_STREAM 3u
#define TRANSPORT_WIRE_MANAGEMENT_REQUEST 4u
#define TRANSPORT_WIRE_MANAGEMENT_REPLY 5u
#define TRANSPORT_WIRE_ENUM_HELLO 6u
#define TRANSPORT_WIRE_ENUM_ASSIGN 7u
#define TRANSPORT_WIRE_ENUM_CONFIRM 8u
#define TRANSPORT_WIRE_CTS 9u

typedef enum {
    TRANSPORT_REGISTRATION_FAILURE_RADIO = 1
} transport_registration_failure_t;

typedef enum {
    TRANSPORT_CTS_ACCEPTED = 0,
    TRANSPORT_CTS_NO_BUFFER = 1,
    TRANSPORT_CTS_TOO_LARGE = 2,
    TRANSPORT_CTS_BUSY = 3,
    TRANSPORT_CTS_INVALID = 4
} transport_cts_result_t;

typedef enum {
    TRANSPORT_DIRECTION_NONE = 0,
    TRANSPORT_DIRECTION_RX,
    TRANSPORT_DIRECTION_TX
} transport_direction_t;

typedef enum {
    TRANSPORT_COMMAND_FREE = 0, TRANSPORT_COMMAND_INTENT_RECEIVED,
    TRANSPORT_COMMAND_CTS_SENT, TRANSPORT_COMMAND_RECEIVING,
    TRANSPORT_COMMAND_COMPLETE, TRANSPORT_COMMAND_DELIVERED,
    TRANSPORT_COMMAND_FAILED, TRANSPORT_COMMAND_TX_LOADING,
    TRANSPORT_COMMAND_TX_INTENT_QUEUED, TRANSPORT_COMMAND_TX_WAIT_CTS,
    TRANSPORT_COMMAND_TX_SENDING
} transport_command_state_t;

typedef enum {
    TRANSPORT_COMMAND_FAILURE_TIMEOUT = 1,
    TRANSPORT_COMMAND_FAILURE_RADIO = 2,
    TRANSPORT_COMMAND_FAILURE_CTS_BASE = 0x100
} transport_command_failure_t;

typedef enum {
    TRANSPORT_PIPE_FREE = 0, TRANSPORT_PIPE_OPEN_INTENT_RECEIVED,
    TRANSPORT_PIPE_CTS_SENT, TRANSPORT_PIPE_OPEN, TRANSPORT_PIPE_CLOSING,
    TRANSPORT_PIPE_CLOSED, TRANSPORT_PIPE_FAILED,
    TRANSPORT_PIPE_TX_INTENT_QUEUED, TRANSPORT_PIPE_TX_WAIT_CTS,
    TRANSPORT_PIPE_TX_OPEN, TRANSPORT_PIPE_TX_CLOSING
} transport_pipe_state_t;

typedef enum {
    TRANSPORT_PIPE_FAILURE_TIMEOUT = 1,
    TRANSPORT_PIPE_FAILURE_RADIO = 2,
    TRANSPORT_PIPE_FAILURE_CTS_BASE = 0x100
} transport_pipe_failure_t;

typedef enum {
    TRANSPORT_RETRY_IDLE = 0, TRANSPORT_RETRY_HW_ACTIVE,
    TRANSPORT_RETRY_MAX_RT, TRANSPORT_RETRY_BACKOFF,
    TRANSPORT_RETRY_EXHAUSTED
} transport_retry_state_t;

typedef enum {
    TRANSPORT_ERROR_NONE = 0,
    TRANSPORT_ERROR_EVENT_QUEUE_OVERFLOW = 1,
    TRANSPORT_ERROR_RX_OVERRUN = 2,
    TRANSPORT_ERROR_TX_UNDERRUN = 3,
    TRANSPORT_ERROR_COMMAND_TOO_LARGE = 4,
    TRANSPORT_ERROR_COMMAND_TIMEOUT = 5,
    TRANSPORT_ERROR_PIPE_TIMEOUT = 6,
    TRANSPORT_ERROR_PROTOCOL = 7,
    TRANSPORT_ERROR_RADIO = 8,
    TRANSPORT_ERROR_NOT_STARTED = 9,
    TRANSPORT_ERROR_CONTROL_QUEUE_FULL = 10
} transport_error_t;

typedef enum {
    TRANSPORT_POLL_EMPTY = 0, TRANSPORT_POLL_EVENT = 1,
    TRANSPORT_POLL_BUFFER_TOO_SMALL = -1,
    TRANSPORT_POLL_INVALID_ARGUMENT = -2
} transport_poll_result_t;

typedef uint32_t (*transport_ticks_ms_fn)(void *context);
typedef bool (*transport_radio_state_fn)(void *context);
typedef uintptr_t transport_critical_state_t;
typedef transport_critical_state_t (*transport_critical_enter_fn)(void *context);
typedef void (*transport_critical_exit_fn)(void *context,
    transport_critical_state_t state);
typedef struct {
    nrf24_t *radio;
    transport_ticks_ms_fn ticks_ms;
    transport_radio_state_fn irq_is_low;
    transport_radio_state_fn io_ok;
    transport_critical_enter_fn critical_enter;
    transport_critical_exit_fn critical_exit;
    void *context;
} transport_radio_binding_t;

typedef struct {
    uint8_t node_id;
    uint8_t network_id[4];
    uint8_t service_address[NRF24_ADDR_LEN];
    bool has_service_address;
    uint16_t preferred_pipe_event_bytes;
    uint16_t command_lease_ms;
    uint16_t pipe_lease_ms;
    uint16_t max_rt_window_ms;
    uint8_t max_rt_restarts;
    transport_radio_binding_t radio;
} transport_core_config_t;

typedef struct {
    transport_ring_t buffer;
    uint8_t peer_id, message_type;
    uint16_t transaction_id, target_length, transferred_length;
    uint32_t lease_deadline_ms;
    transport_command_state_t state;
    transport_direction_t direction;
} transport_command_slot_t;

typedef struct {
    transport_ring_t buffer;
    uint8_t peer_id;
    uint16_t session_id;
    uint32_t granted_bytes, transferred_bytes;
    uint32_t lease_deadline_ms;
    transport_pipe_state_t state;
    transport_direction_t direction;
    bool rx_event_pending, tx_space_event_pending, credit_update_pending;
} transport_pipe_slot_t;

typedef struct {
    transport_retry_state_t state;
    uint8_t max_restarts, restarts;
    uint32_t deadline_ms;
} transport_retry_record_t;

typedef enum {
    TRANSPORT_TX_NONE = 0,
    TRANSPORT_TX_CONTROL,
    TRANSPORT_TX_REGISTRATION,
    TRANSPORT_TX_COMMAND,
    TRANSPORT_TX_PIPE
} transport_tx_kind_t;

typedef struct {
    bool active;
    bool last_packet;
    uint8_t destination_id, slot, payload_length;
    transport_tx_kind_t kind;
} transport_tx_record_t;

typedef struct {
    bool pending;
    uint8_t address[NRF24_ADDR_LEN];
    uint8_t packet[NRF24_MAX_PAYLOAD];
} transport_registration_tx_t;

typedef struct {
    uint32_t radio_irqs, events_emitted, events_polled;
    uint32_t rx_packets, tx_packets;
    uint32_t event_queue_overflows, rx_overruns, tx_underruns;
    uint32_t max_rt_events, max_rt_restarts, protocol_errors;
    uint32_t pipe_rx_bytes, pipe_tx_bytes;
    uint32_t control_packets_queued, control_packets_acked;
    uint32_t commands_sent, commands_failed;
    uint32_t pipes_opened, pipes_closed, pipes_failed;
    uint32_t registration_rx, registration_sent, registration_failed;
} transport_core_stats_t;

typedef enum {
    TRANSPORT_EVENT_PAYLOAD_NONE = 0,
    TRANSPORT_EVENT_PAYLOAD_COMMAND_SLOT,
    TRANSPORT_EVENT_PAYLOAD_PIPE_RX,
    TRANSPORT_EVENT_PAYLOAD_INLINE
} transport_event_payload_kind_t;

typedef struct {
    transport_event_fields_t fields;
    uint8_t payload_kind, payload_index;
    uint8_t inline_data[TRANSPORT_REGISTRATION_MAX_DATA];
} transport_event_descriptor_t;

typedef struct {
    transport_core_config_t config;
    transport_command_slot_t commands[TRANSPORT_CORE_COMMAND_SLOTS];
    transport_pipe_slot_t pipes[TRANSPORT_CORE_PIPE_SLOTS];
    transport_ring_t control_tx;
    transport_event_descriptor_t events[TRANSPORT_CORE_EVENT_QUEUE_SIZE];
    /* Monotonic SPSC counters: IRQ/native producer writes event_write;
       foreground poller writes event_read. */
    volatile uint8_t event_read, event_write;
    bool started;
    uint32_t sticky_errors;
    uint32_t application_tx_not_before_ms;
    transport_retry_record_t retry;
    transport_tx_record_t tx;
    transport_registration_tx_t registration_tx;
    transport_core_stats_t stats;
} transport_core_t;

void transport_core_init(transport_core_t *, const transport_core_config_t *);
bool transport_core_attach_command_buffer(transport_core_t *, uint8_t, uint8_t *, size_t);
bool transport_core_attach_pipe_buffer(transport_core_t *, uint8_t, uint8_t *, size_t);
bool transport_core_attach_control_tx_buffer(transport_core_t *, uint8_t *, size_t);
bool transport_core_start(transport_core_t *);
void transport_core_stop(transport_core_t *);

/* Services every currently latched NRF interrupt and drains the RX FIFO. */
void transport_core_on_radio_irq(transport_core_t *);
void transport_core_service(transport_core_t *);

bool transport_core_commit_command(transport_core_t *, uint8_t, uint8_t, uint16_t, size_t);
bool transport_core_send_command(transport_core_t *, uint8_t, uint8_t, uint8_t,
    const uint8_t *, size_t);
bool transport_core_send_registration(transport_core_t *, uint8_t, uint8_t,
    uint8_t, const uint8_t[NRF24_ADDR_LEN], const uint8_t *, size_t);
bool transport_core_set_node_id(transport_core_t *, uint8_t);
int transport_core_open_pipe(transport_core_t *, uint8_t, uint8_t);
bool transport_core_close_pipe(transport_core_t *, uint8_t);
size_t transport_core_pipe_rx_write(transport_core_t *, uint8_t, const uint8_t *, size_t);
size_t transport_core_pipe_tx_write(transport_core_t *, uint8_t, const uint8_t *, size_t);
size_t transport_core_pipe_tx_read(transport_core_t *, uint8_t, uint8_t *, size_t);
bool transport_core_queue_control(transport_core_t *, uint8_t, uint8_t, uint8_t,
    const uint8_t *, size_t);
bool transport_core_emit(transport_core_t *, const transport_event_fields_t *);
void transport_core_report_error(transport_core_t *, transport_error_t, uint8_t, uint32_t);
transport_poll_result_t transport_core_poll_into(transport_core_t *, uint8_t *, size_t,
    size_t *, size_t *);
const transport_core_stats_t *transport_core_get_stats(const transport_core_t *);
uint32_t transport_core_get_sticky_errors(const transport_core_t *);
void transport_core_clear_sticky_errors(transport_core_t *, uint32_t);

#endif
