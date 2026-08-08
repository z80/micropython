#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "transport_core.h"
#include "nrf24_regs.h"

static uint8_t fake_status;
static uint8_t fake_rx_payload[NRF24_MAX_PAYLOAD];
static bool fake_rx_ready;
static uint8_t captured_tx[NRF24_MAX_PAYLOAD];
static uint8_t fake_tx_fifo[3][NRF24_MAX_PAYLOAD];
static uint8_t fake_tx_read, fake_tx_count;
static unsigned fake_tx_flush_count;
static uint8_t captured_address[NRF24_ADDR_LEN];
static uint8_t captured_rx_address[6][NRF24_ADDR_LEN];
static uint8_t captured_rx_open;
static unsigned captured_tx_count;
static uint32_t fake_now;
static bool fake_rx_mode;
static bool fake_ce;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length) {
    size_t i;
    crc ^= 0xffffffffu;
    for (i = 0; i < length; ++i) {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xedb88320u & mask);
        }
    }
    return crc ^ 0xffffffffu;
}

static void make_packet(uint8_t *packet, const uint8_t network[4],
        uint8_t type, uint8_t source, uint8_t destination, uint8_t id,
        uint8_t flags, const uint8_t *payload, size_t length) {
    uint32_t crc;
    memset(packet, 0, NRF24_MAX_PAYLOAD);
    packet[0] = TRANSPORT_WIRE_PROTOCOL_ID | type;
    packet[1] = source;
    packet[2] = destination;
    packet[3] = id;
    packet[4] = flags | (uint8_t)length;
    if (length != 0) memcpy(packet + TRANSPORT_WIRE_HEADER_SIZE, payload, length);
    crc = crc32_update(0, network, 4);
    crc = crc32_update(crc, packet, TRANSPORT_WIRE_HEADER_SIZE);
    crc = crc32_update(crc, packet + TRANSPORT_WIRE_HEADER_SIZE, length);
    packet[TRANSPORT_WIRE_HEADER_SIZE + length] = (uint8_t)crc;
}

static void make_cts(uint8_t *packet, const uint8_t network[4],
        uint8_t source, uint8_t destination, uint8_t id,
        uint8_t original_type, uint8_t result, uint32_t value) {
    uint8_t payload[6];
    payload[0] = original_type;
    payload[1] = result;
    payload[2] = (uint8_t)value;
    payload[3] = (uint8_t)(value >> 8);
    payload[4] = (uint8_t)(value >> 16);
    payload[5] = (uint8_t)(value >> 24);
    make_packet(packet, network, TRANSPORT_WIRE_CTS, source, destination, id,
        TRANSPORT_WIRE_LAST_PACKET, payload, sizeof(payload));
}

static uint32_t ticks_ms(void *context) {
    (void)context;
    return fake_now;
}

static bool irq_is_low(void *context) {
    (void)context;
    return fake_status != 0;
}

static bool io_ok(void *context) {
    (void)context;
    return true;
}

uint8_t nrf24_read_status(nrf24_t *radio) {
    (void)radio;
    return fake_status;
}

void nrf24_clear_irq(nrf24_t *radio, uint8_t sources) {
    (void)radio;
    fake_status &= (uint8_t)~sources;
}

bool nrf24_rx_fifo_empty(nrf24_t *radio) {
    (void)radio;
    return !fake_rx_ready;
}

bool nrf24_tx_fifo_empty(nrf24_t *radio) {
    (void)radio;
    return fake_tx_count == 0;
}

bool nrf24_tx_fifo_full(nrf24_t *radio) {
    (void)radio;
    return fake_tx_count == 3;
}

bool nrf24_read_payload(nrf24_t *radio, uint8_t *buffer, size_t capacity,
        size_t *length) {
    (void)radio;
    if (!fake_rx_ready || capacity < NRF24_MAX_PAYLOAD) return false;
    memcpy(buffer, fake_rx_payload, NRF24_MAX_PAYLOAD);
    fake_rx_ready = false;
    if (length != NULL) *length = NRF24_MAX_PAYLOAD;
    return true;
}

void nrf24_stop_listening(nrf24_t *radio) {
    (void)radio;
    fake_rx_mode = false;
}

void nrf24_enter_tx_mode(nrf24_t *radio) {
    (void)radio;
    fake_rx_mode = false;
    fake_ce = false;
    fake_status &= (uint8_t)~(NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
}

void nrf24_enter_rx_mode(nrf24_t *radio) {
    (void)radio;
    fake_rx_mode = true;
    fake_ce = true;
    fake_status &= (uint8_t)~(NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
}

void nrf24_start_listening(nrf24_t *radio) {
    nrf24_enter_rx_mode(radio);
}

void nrf24_set_ce(nrf24_t *radio, bool high) {
    (void)radio;
    fake_ce = high;
}

void nrf24_open_tx_pipe(nrf24_t *radio, const uint8_t *address) {
    (void)radio;
    memcpy(captured_address, address, sizeof(captured_address));
}

void nrf24_open_rx_pipe(nrf24_t *radio, uint8_t pipe, const uint8_t *address) {
    (void)radio;
    assert(pipe < 6);
    memcpy(captured_rx_address[pipe], address, NRF24_ADDR_LEN);
    captured_rx_open |= (uint8_t)(1u << pipe);
}

void nrf24_close_rx_pipe(nrf24_t *radio, uint8_t pipe) {
    (void)radio;
    assert(pipe < 6);
    captured_rx_open &= (uint8_t)~(1u << pipe);
}

bool nrf24_write_payload(nrf24_t *radio, const uint8_t *buffer, size_t length) {
    (void)radio;
    if (length != NRF24_MAX_PAYLOAD || fake_tx_count == 3) return false;
    if (fake_tx_count == 0) memcpy(captured_tx, buffer, length);
    memcpy(fake_tx_fifo[(fake_tx_read + fake_tx_count) % 3], buffer, length);
    fake_tx_count++;
    captured_tx_count++;
    return true;
}

void nrf24_abort_send(nrf24_t *radio) {
    (void)radio;
    fake_ce = false;
    fake_status = 0;
    fake_tx_read = 0;
    fake_tx_count = 0;
    fake_tx_flush_count++;
}

static void deliver(transport_core_t *core, const uint8_t *packet) {
    memcpy(fake_rx_payload, packet, NRF24_MAX_PAYLOAD);
    fake_rx_ready = true;
    fake_status |= NRF24_STATUS_RX_DR;
    transport_core_on_radio_irq(core);
}

static void deliver_close_ack(transport_core_t *core,
        const uint8_t network[4], uint8_t source, uint8_t destination,
        uint8_t stream_id) {
    uint8_t packet[NRF24_MAX_PAYLOAD];
    make_packet(packet, network, TRANSPORT_WIRE_PIPE_CLOSE_ACK, source,
        destination, stream_id, TRANSPORT_WIRE_LAST_PACKET, NULL, 0);
    deliver(core, packet);
}

static void ack_tx(transport_core_t *core) {
    assert(core->tx.active);
    assert(fake_tx_count != 0);
    fake_tx_read = (uint8_t)((fake_tx_read + 1) % 3);
    fake_tx_count--;
    if (fake_tx_count != 0) {
        memcpy(captured_tx, fake_tx_fifo[fake_tx_read], NRF24_MAX_PAYLOAD);
    }
    fake_status |= NRF24_STATUS_TX_DS;
    transport_core_on_radio_irq(core);
}

static void ack_then_max_rt(transport_core_t *core) {
    assert(core->tx.active);
    assert(fake_tx_count >= 2);
    fake_tx_read = (uint8_t)((fake_tx_read + 1) % 3);
    fake_tx_count--;
    memcpy(captured_tx, fake_tx_fifo[fake_tx_read], NRF24_MAX_PAYLOAD);
    fake_status |= NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT;
    transport_core_on_radio_irq(core);
}

static void send_command_to_receiver(transport_core_t *core,
        const uint8_t network[4], uint8_t id, const uint8_t *data,
        uint16_t length) {
    uint8_t packet[NRF24_MAX_PAYLOAD];
    uint8_t intent[2];
    intent[0] = (uint8_t)length;
    intent[1] = (uint8_t)(length >> 8);
    make_packet(packet, network, TRANSPORT_WIRE_COMMAND, 7, 1, id,
        TRANSPORT_WIRE_INTENT | TRANSPORT_WIRE_LAST_PACKET,
        intent, sizeof(intent));
    deliver(core, packet);
    assert((captured_tx[0] & TRANSPORT_WIRE_TYPE_MASK) == TRANSPORT_WIRE_CTS);
    assert(captured_tx[5] == TRANSPORT_WIRE_COMMAND);
    assert(captured_tx[6] == TRANSPORT_CTS_ACCEPTED);
    ack_tx(core);
    make_packet(packet, network, TRANSPORT_WIRE_COMMAND, 7, 1, id,
        TRANSPORT_WIRE_LAST_PACKET, data, length);
    deliver(core, packet);
}

int main(void) {
    transport_core_t core;
    transport_core_config_t config;
    nrf24_t radio;
    uint8_t command_buffer_small[8];
    uint8_t command_buffer_large[64];
    uint8_t pipe_buffers[TRANSPORT_CORE_PIPE_SLOTS][160];
    uint8_t control_buffer[TRANSPORT_CORE_CONTROL_TX_SLOTS * NRF24_MAX_PAYLOAD];
    uint8_t event_buffer[64];
    uint8_t packet[NRF24_MAX_PAYLOAD];
    uint8_t intent[2];
    size_t event_length, required;
    const uint8_t first[] = { 'h', 'e', 'l', 'l', 'o' };
    const uint8_t wrapped[] = { 'w', 'r', 'a', 'p', '!', '!' };
    uint8_t outbound[60];
    uint8_t long_stream[120];
    uint8_t i;

    memset(&config, 0, sizeof(config));
    memset(&radio, 0, sizeof(radio));
    config.node_id = 1;
    config.network_id[0] = 0x01;
    config.network_id[1] = 0xb2;
    config.network_id[2] = 0x37;
    config.network_id[3] = 0xaa;
    config.command_lease_ms = 100;
    config.pipe_lease_ms = 100;
    config.max_rt_window_ms = 100;
    config.max_rt_restarts = 3;
    config.radio.radio = &radio;
    config.radio.ticks_ms = ticks_ms;
    config.radio.irq_is_low = irq_is_low;
    config.radio.io_ok = io_ok;
    transport_core_init(&core, &config);
    assert(transport_core_attach_command_buffer(&core, 0,
        command_buffer_small, sizeof(command_buffer_small)));
    assert(transport_core_attach_command_buffer(&core, 1,
        command_buffer_large, sizeof(command_buffer_large)));
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        assert(transport_core_attach_pipe_buffer(&core, i,
            pipe_buffers[i], i == 0 ? 32 : sizeof(pipe_buffers[i])));
    }
    assert(transport_core_attach_control_tx_buffer(&core, control_buffer,
        sizeof(control_buffer)));
    assert(transport_core_start(&core));
    {
        uint16_t max_tx_ms = 0, rx_ms = 0;
        transport_core_get_radio_schedule(&core, &max_tx_ms, &rx_ms);
        assert(max_tx_ms == TRANSPORT_CORE_DEFAULT_MAX_CONTINUOUS_TX_MS);
        assert(rx_ms == TRANSPORT_CORE_DEFAULT_FORCED_RX_MS);
        assert(transport_core_set_radio_schedule(&core, 40, 5));
        transport_core_get_radio_schedule(&core, &max_tx_ms, &rx_ms);
        assert(max_tx_ms == 40 && rx_ms == 5);
        assert(transport_core_set_radio_schedule(&core,
            TRANSPORT_CORE_DEFAULT_MAX_CONTINUOUS_TX_MS,
            TRANSPORT_CORE_DEFAULT_FORCED_RX_MS));
    }

    assert(transport_core_queue_control(&core, TRANSPORT_WIRE_CTS, 9, 3,
        first, sizeof(first)));
    transport_core_service(&core);
    assert(core.tx.active && fake_ce && !fake_rx_mode);
    assert(captured_address[0] == 9);
    assert(memcmp(captured_address + 1, config.network_id, 4) == 0);
    assert(captured_tx[1] == 1 && captured_tx[2] == 9 && captured_tx[3] == 3);
    ack_tx(&core);
    assert(!core.tx.active && fake_rx_mode);

    send_command_to_receiver(&core, config.network_id, 10, first,
        (uint16_t)sizeof(first));
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_COMMAND_READY);
    assert(transport_event_get_data_length(event_buffer) == sizeof(first));
    assert(memcmp(transport_event_get_data(event_buffer), first, sizeof(first)) == 0);

    /* The first command leaves the ring index at five.  Six bytes therefore
       cross the eight-byte physical end and exercise the two-part ring copy. */
    send_command_to_receiver(&core, config.network_id, 11, wrapped,
        (uint16_t)sizeof(wrapped));
    assert(transport_core_poll_into(&core, event_buffer,
        TRANSPORT_EVENT_HEADER_SIZE + 2, &event_length, &required) ==
            TRANSPORT_POLL_BUFFER_TOO_SMALL);
    assert(required == TRANSPORT_EVENT_HEADER_SIZE + sizeof(wrapped));
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(memcmp(transport_event_get_data(event_buffer), wrapped,
        sizeof(wrapped)) == 0);

    make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1, 20,
        TRANSPORT_WIRE_INTENT | TRANSPORT_WIRE_LAST_PACKET, NULL, 0);
    deliver(&core, packet);
    assert(core.pipes[0].state == TRANSPORT_PIPE_CTS_SENT);
    ack_tx(&core);
    assert(core.pipes[0].state == TRANSPORT_PIPE_OPEN);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_PIPE_OPENED);
    make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1, 20,
        TRANSPORT_WIRE_LAST_PACKET, first, sizeof(first));
    deliver(&core, packet);
    assert((captured_tx[0] & TRANSPORT_WIRE_TYPE_MASK) ==
        TRANSPORT_WIRE_PIPE_CLOSE_ACK);
    ack_tx(&core);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_PIPE_RX_DATA);
    assert(memcmp(transport_event_get_data(event_buffer), first, sizeof(first)) == 0);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_PIPE_CLOSED);

    intent[0] = 65;
    intent[1] = 0;
    make_packet(packet, config.network_id, TRANSPORT_WIRE_COMMAND, 7, 1, 12,
        TRANSPORT_WIRE_INTENT | TRANSPORT_WIRE_LAST_PACKET,
        intent, sizeof(intent));
    deliver(&core, packet);
    assert(captured_tx[6] == TRANSPORT_CTS_TOO_LARGE);
    ack_tx(&core);

    for (i = 0; i < sizeof(outbound); ++i) outbound[i] = i;
    for (i = 0; i < sizeof(long_stream); ++i) long_stream[i] = i;
    assert(transport_core_send_command(&core, 9, TRANSPORT_WIRE_COMMAND,
        30, outbound, sizeof(outbound)));
    assert(core.tx.kind == TRANSPORT_TX_CONTROL);
    assert((captured_tx[4] & TRANSPORT_WIRE_INTENT) != 0);
    assert(captured_tx[5] == sizeof(outbound) && captured_tx[6] == 0);
    assert(transport_ring_readable(&core.commands[1].buffer) ==
        sizeof(outbound));
    assert(!transport_core_send_command(&core, 8, TRANSPORT_WIRE_COMMAND,
        31, first, sizeof(first)));
    ack_tx(&core);
    assert(core.commands[1].state == TRANSPORT_COMMAND_TX_WAIT_CTS);
    assert(fake_rx_mode);

    make_cts(packet, config.network_id, 9, 1, 99, TRANSPORT_WIRE_COMMAND,
        TRANSPORT_CTS_ACCEPTED, sizeof(outbound));
    deliver(&core, packet);
    assert(core.commands[1].state == TRANSPORT_COMMAND_TX_WAIT_CTS);
    make_cts(packet, config.network_id, 9, 1, 30, TRANSPORT_WIRE_COMMAND,
        TRANSPORT_CTS_ACCEPTED, sizeof(outbound));
    deliver(&core, packet);
    assert(!core.tx.active);
    fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
    transport_core_service(&core);
    assert(core.tx.kind == TRANSPORT_TX_COMMAND);
    assert((captured_tx[4] & TRANSPORT_WIRE_LENGTH_MASK) ==
        TRANSPORT_WIRE_MAX_DATA);
    assert((captured_tx[4] & TRANSPORT_WIRE_LAST_PACKET) == 0);
    assert(memcmp(captured_tx + TRANSPORT_WIRE_HEADER_SIZE, outbound,
        TRANSPORT_WIRE_MAX_DATA) == 0);
    {
        unsigned writes_before_retry = captured_tx_count;
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(core.tx.active);
        assert(core.retry.restarts == 1);
        assert(captured_tx_count == writes_before_retry);
        assert(fake_tx_count == 3);
        assert(core.commands[1].transferred_length == 0);
        assert(transport_ring_readable(&core.commands[1].buffer) == 0);

        /* The first head succeeds and the second reaches MAX_RT before the
           IRQ is serviced.  The second head gets a fresh retry budget. */
        ack_then_max_rt(&core);
        assert(core.tx.active);
        assert(core.retry.restarts == 1);
        assert(captured_tx_count == writes_before_retry);
        assert(fake_tx_count == 2);
        assert(core.commands[1].transferred_length == 0);
    }
    ack_tx(&core);
    assert(transport_ring_readable(&core.commands[1].buffer) == 0);
    assert((captured_tx[4] & TRANSPORT_WIRE_LENGTH_MASK) == 8);
    assert((captured_tx[4] & TRANSPORT_WIRE_LAST_PACKET) != 0);
    ack_tx(&core);
    assert(core.commands[1].state == TRANSPORT_COMMAND_FREE);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_COMMAND_SENT);
    assert(transport_event_get_source_id(event_buffer) == 9);
    assert(transport_event_get_transaction_id(event_buffer) == 30);
    assert(transport_event_get_value1(event_buffer) == sizeof(outbound));

    assert(transport_core_send_command(&core, 9, TRANSPORT_WIRE_COMMAND,
        31, NULL, 0));
    ack_tx(&core);
    make_cts(packet, config.network_id, 9, 1, 31, TRANSPORT_WIRE_COMMAND,
        TRANSPORT_CTS_ACCEPTED, 0);
    deliver(&core, packet);
    fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
    transport_core_service(&core);
    assert(core.tx.kind == TRANSPORT_TX_COMMAND);
    assert((captured_tx[4] & TRANSPORT_WIRE_LENGTH_MASK) == 0);
    assert((captured_tx[4] & TRANSPORT_WIRE_LAST_PACKET) != 0);
    ack_tx(&core);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_COMMAND_SENT);

    assert(transport_core_send_command(&core, 9, TRANSPORT_WIRE_COMMAND,
        32, first, sizeof(first)));
    ack_tx(&core);
    make_cts(packet, config.network_id, 9, 1, 32, TRANSPORT_WIRE_COMMAND,
        TRANSPORT_CTS_BUSY, 0);
    deliver(&core, packet);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_COMMAND_FAILED);
    assert(transport_event_get_value1(event_buffer) ==
        TRANSPORT_COMMAND_FAILURE_CTS_BASE + TRANSPORT_CTS_BUSY);

    assert(transport_core_send_command(&core, 9, TRANSPORT_WIRE_COMMAND,
        33, first, sizeof(first)));
    ack_tx(&core);
    fake_now += config.command_lease_ms;
    transport_core_service(&core);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_COMMAND_FAILED);
    assert(transport_event_get_value1(event_buffer) ==
        TRANSPORT_COMMAND_FAILURE_TIMEOUT);

    assert(transport_core_send_command(&core, 9, TRANSPORT_WIRE_COMMAND,
        35, first, sizeof(first)));
    for (i = 0; i < config.max_rt_restarts; ++i) {
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(core.tx.active);
    }
    fake_status |= NRF24_STATUS_MAX_RT;
    transport_core_on_radio_irq(&core);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_COMMAND_FAILED);
    assert(transport_event_get_value1(event_buffer) ==
        TRANSPORT_COMMAND_FAILURE_RADIO);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_CORE_ERROR);

    assert(transport_core_send_command(&core, 9, TRANSPORT_WIRE_COMMAND,
        34, first, sizeof(first)));
    ack_tx(&core);
    make_cts(packet, config.network_id, 9, 1, 34, TRANSPORT_WIRE_COMMAND,
        TRANSPORT_CTS_ACCEPTED, sizeof(first));
    deliver(&core, packet);
    fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
    transport_core_service(&core);
    for (i = 0; i < config.max_rt_restarts; ++i) {
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(core.tx.active);
    }
    fake_status |= NRF24_STATUS_MAX_RT;
    transport_core_on_radio_irq(&core);
    assert(core.commands[0].state == TRANSPORT_COMMAND_FREE);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_COMMAND_FAILED);
    assert(transport_event_get_value1(event_buffer) ==
        TRANSPORT_COMMAND_FAILURE_RADIO);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) == TRANSPORT_EVENT_CORE_ERROR);

    assert(core.stats.commands_sent == 2);
    assert(core.stats.commands_failed == 4);

    /* An unbounded restart count is still bounded by the retry window. */
    core.config.max_rt_restarts = TRANSPORT_CORE_RESTARTS_UNBOUNDED;
    assert(transport_core_send_command(&core, 9, TRANSPORT_WIRE_COMMAND,
        35, first, sizeof(first)));
    ack_tx(&core);
    make_cts(packet, config.network_id, 9, 1, 35, TRANSPORT_WIRE_COMMAND,
        TRANSPORT_CTS_ACCEPTED, sizeof(first));
    deliver(&core, packet);
    fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
    transport_core_service(&core);
    for (i = 0; i < 5; ++i) {
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(core.tx.active);
    }
    fake_now += config.max_rt_window_ms;
    fake_status |= NRF24_STATUS_MAX_RT;
    transport_core_on_radio_irq(&core);
    assert(!core.tx.active);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) ==
        TRANSPORT_EVENT_COMMAND_FAILED);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) ==
        TRANSPORT_EVENT_CORE_ERROR);
    core.config.max_rt_restarts = config.max_rt_restarts;

    /* Outbound streams reserve the receiver first.  Ring space is released
       as data enters the hardware FIFO, while cumulative credit is committed
       only after that FIFO drains. */
    {
        int pipe_id = transport_core_open_pipe(&core, 9, 40);
        unsigned writes_before_retry;
        assert(pipe_id == 0);
        assert(core.tx.kind == TRANSPORT_TX_CONTROL);
        assert((captured_tx[0] & TRANSPORT_WIRE_TYPE_MASK) ==
            TRANSPORT_WIRE_STREAM);
        assert((captured_tx[4] & TRANSPORT_WIRE_INTENT) != 0);
        assert(transport_core_pipe_tx_write(&core, (uint8_t)pipe_id,
            outbound, 32) == 32);
        assert(!transport_core_send_command(&core, 8,
            TRANSPORT_WIRE_COMMAND, 41, first, sizeof(first)));
        ack_tx(&core);
        assert(core.pipes[pipe_id].state == TRANSPORT_PIPE_TX_WAIT_CTS);

        make_cts(packet, config.network_id, 9, 1, 40,
            TRANSPORT_WIRE_STREAM, TRANSPORT_CTS_ACCEPTED, 32);
        deliver(&core, packet);
        assert(!core.tx.active);
        fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
        transport_core_service(&core);
        assert(core.tx.kind == TRANSPORT_TX_PIPE);
        assert((captured_tx[4] & TRANSPORT_WIRE_LENGTH_MASK) == 26);
        assert(memcmp(captured_tx + TRANSPORT_WIRE_HEADER_SIZE, outbound,
            26) == 0);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_OPENED);
        assert((transport_event_get_flags(event_buffer) &
            TRANSPORT_EVENT_FLAG_TX) != 0);
        assert(transport_event_get_value1(event_buffer) == 32);

        writes_before_retry = captured_tx_count;
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(captured_tx_count == writes_before_retry);
        assert(transport_ring_readable(&core.pipes[pipe_id].buffer) == 0);
        ack_tx(&core);
        assert(transport_ring_readable(&core.pipes[pipe_id].buffer) == 0);
        assert((captured_tx[4] & TRANSPORT_WIRE_LENGTH_MASK) == 6);
        ack_tx(&core);
        assert(!core.tx.active);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_TX_SPACE);

        assert(transport_core_pipe_tx_write(&core, (uint8_t)pipe_id,
            outbound + 32, 28) == 28);
        assert(!core.tx.active);
        make_cts(packet, config.network_id, 9, 1, 40,
            TRANSPORT_WIRE_STREAM, TRANSPORT_CTS_ACCEPTED, 64);
        deliver(&core, packet);
        assert(!core.tx.active);
        fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
        transport_core_service(&core);
        assert(core.tx.kind == TRANSPORT_TX_PIPE);
        assert((captured_tx[4] & TRANSPORT_WIRE_LENGTH_MASK) == 26);
        ack_tx(&core);
        assert((captured_tx[4] & TRANSPORT_WIRE_LENGTH_MASK) == 2);
        ack_tx(&core);
        assert(transport_core_close_pipe(&core, (uint8_t)pipe_id));
        assert(core.tx.kind == TRANSPORT_TX_PIPE);
        assert((captured_tx[4] & TRANSPORT_WIRE_LENGTH_MASK) == 0);
        assert((captured_tx[4] & TRANSPORT_WIRE_LAST_PACKET) != 0);
        ack_tx(&core);
        assert(core.pipes[pipe_id].state ==
            TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK);
        deliver_close_ack(&core, config.network_id, 9, 1, 40);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_TX_SPACE);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_CLOSED);
        assert((transport_event_get_flags(event_buffer) &
            TRANSPORT_EVENT_FLAG_TX) != 0);
        assert(transport_event_get_value0(event_buffer) == sizeof(outbound));
        assert(core.pipes[pipe_id].state == TRANSPORT_PIPE_FREE);
    }

    /* Closing while the final data fragments are still staged must first
       drain the data FIFO.  The terminal packet then starts as a fresh,
       isolated FIFO batch. */
    {
        transport_pipe_slot_t *pipe = &core.pipes[1];
        unsigned writes_after_data;
        transport_ring_reset(&pipe->buffer);
        pipe->peer_id = 9;
        pipe->session_id = 47;
        pipe->granted_bytes = sizeof(outbound);
        pipe->transferred_bytes = 0;
        pipe->direction = TRANSPORT_DIRECTION_TX;
        pipe->state = TRANSPORT_PIPE_TX_OPEN;
        pipe->lease_deadline_ms = fake_now + config.pipe_lease_ms;

        assert(transport_core_pipe_tx_write(&core, 1, outbound,
            sizeof(outbound)) == sizeof(outbound));
        assert(fake_tx_count == 3);
        writes_after_data = captured_tx_count;
        assert(transport_core_close_pipe(&core, 1));
        assert(captured_tx_count == writes_after_data);
        assert(core.tx.active);
        ack_tx(&core);
        assert(core.tx.drain_requested);
        assert(fake_tx_count == 2);
        assert(captured_tx_count == writes_after_data);
        assert(core.tx.active);
        ack_tx(&core);
        assert(fake_tx_count == 1);
        assert(captured_tx_count == writes_after_data);
        assert(core.tx.active);
        ack_tx(&core);
        assert(fake_tx_count == 0);
        assert(captured_tx_count == writes_after_data);
        assert(!core.tx.active);

        /* Exact credit exhaustion is a turnaround boundary: wait for the
           receiver's replenishment before transmitting close. */
        make_cts(packet, config.network_id, 9, 1, 47,
            TRANSPORT_WIRE_STREAM, TRANSPORT_CTS_ACCEPTED,
            2 * sizeof(outbound));
        deliver(&core, packet);
        assert(!core.tx.active);
        fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
        transport_core_service(&core);
        assert(fake_tx_count == 1);
        assert(captured_tx_count == writes_after_data + 1);
        assert((captured_tx[4] & TRANSPORT_WIRE_LAST_PACKET) != 0);
        assert((captured_tx[4] & TRANSPORT_WIRE_LENGTH_MASK) == 0);
        assert(core.tx.active);
        ack_tx(&core);
        assert(core.pipes[1].state == TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK);
        deliver_close_ack(&core, config.network_id, 9, 1, 47);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_TX_SPACE);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_CLOSED);
        assert(transport_event_get_value0(event_buffer) == sizeof(outbound));
    }

    /* TX_DS for close only starts the protocol-ACK wait.  If that ACK is
       absent, resend close after the peer's hardware retry window. */
    {
        transport_pipe_slot_t *pipe = &core.pipes[1];
        uint16_t saved_lease = core.config.pipe_lease_ms;
        core.config.pipe_lease_ms = 500;
        transport_ring_reset(&pipe->buffer);
        pipe->peer_id = 9;
        pipe->session_id = 48;
        pipe->granted_bytes = 1;
        pipe->transferred_bytes = 0;
        pipe->direction = TRANSPORT_DIRECTION_TX;
        pipe->state = TRANSPORT_PIPE_TX_OPEN;
        pipe->lease_deadline_ms = fake_now + core.config.pipe_lease_ms;

        assert(transport_core_close_pipe(&core, 1));
        assert(core.tx.kind == TRANSPORT_TX_PIPE);
        ack_tx(&core);
        assert(pipe->state == TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK);
        deliver_close_ack(&core, config.network_id, 9, 1, 99);
        assert(pipe->state == TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK);
        {
            uint8_t malformed = 1;
            uint32_t protocol_before = core.stats.protocol_errors;
            make_packet(packet, config.network_id,
                TRANSPORT_WIRE_PIPE_CLOSE_ACK, 9, 1, 48,
                TRANSPORT_WIRE_LAST_PACKET, &malformed, 1);
            deliver(&core, packet);
            assert(pipe->state == TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK);
            assert(core.stats.protocol_errors == protocol_before + 1);
        }
        fake_now += core.config.max_rt_window_ms +
            TRANSPORT_CORE_CTS_TURNAROUND_MS - 1;
        transport_core_service(&core);
        assert(!core.tx.active);
        fake_now++;
        transport_core_service(&core);
        assert(core.tx.kind == TRANSPORT_TX_PIPE);
        assert((captured_tx[4] & TRANSPORT_WIRE_LAST_PACKET) != 0);
        ack_tx(&core);
        deliver_close_ack(&core, config.network_id, 9, 1, 48);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_CLOSED);
        core.config.pipe_lease_ms = saved_lease;
    }

    /* Exhausting hardware retries for a terminal close is an attempt
       failure, not an immediate pipe failure.  The protocol deadline still
       permits a fresh close attempt. */
    {
        transport_pipe_slot_t *pipe = &core.pipes[1];
        uint16_t saved_lease = core.config.pipe_lease_ms;
        core.config.pipe_lease_ms = 500;
        transport_ring_reset(&pipe->buffer);
        pipe->peer_id = 9;
        pipe->session_id = 49;
        pipe->granted_bytes = 1;
        pipe->transferred_bytes = 0;
        pipe->direction = TRANSPORT_DIRECTION_TX;
        pipe->state = TRANSPORT_PIPE_TX_OPEN;
        pipe->lease_deadline_ms = fake_now + core.config.pipe_lease_ms;

        assert(transport_core_close_pipe(&core, 1));
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(core.tx.active);
        fake_now += core.config.max_rt_window_ms;
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(!core.tx.active);
        assert(pipe->state == TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK);
        fake_now += core.config.max_rt_window_ms +
            TRANSPORT_CORE_CTS_TURNAROUND_MS;
        transport_core_service(&core);
        assert(core.tx.kind == TRANSPORT_TX_PIPE);
        ack_tx(&core);
        deliver_close_ack(&core, config.network_id, 9, 1, 49);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_CLOSED);
        core.config.pipe_lease_ms = saved_lease;
    }

    /* Waiting for close confirmation remains bounded by the original close
       lease, and timeout releases the outbound slot through a durable event. */
    {
        transport_pipe_slot_t *pipe = &core.pipes[1];
        transport_ring_reset(&pipe->buffer);
        pipe->peer_id = 9;
        pipe->session_id = 51;
        pipe->granted_bytes = 1;
        pipe->transferred_bytes = 0;
        pipe->direction = TRANSPORT_DIRECTION_TX;
        pipe->state = TRANSPORT_PIPE_TX_OPEN;
        pipe->lease_deadline_ms = fake_now + core.config.pipe_lease_ms;

        assert(transport_core_close_pipe(&core, 1));
        ack_tx(&core);
        assert(pipe->state == TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK);
        fake_now += core.config.pipe_lease_ms;
        transport_core_service(&core);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_FAILED);
        assert(transport_event_get_value0(event_buffer) ==
            TRANSPORT_PIPE_FAILURE_TIMEOUT);
        assert(pipe->state == TRANSPORT_PIPE_FREE);
    }

    /* A live pipe keeps the three-entry FIFO full.  A command to another
       peer requests draining, owns the FIFO next, and the pipe then resumes. */
    {
        transport_pipe_slot_t *pipe = &core.pipes[1];
        unsigned writes_before_refill;
        bool saw_command_sent = false, saw_pipe_closed = false;
        transport_ring_reset(&pipe->buffer);
        pipe->peer_id = 9;
        pipe->session_id = 44;
        pipe->granted_bytes = sizeof(long_stream);
        pipe->transferred_bytes = 0;
        pipe->direction = TRANSPORT_DIRECTION_TX;
        pipe->state = TRANSPORT_PIPE_TX_OPEN;
        pipe->lease_deadline_ms = fake_now + config.pipe_lease_ms;

        assert(transport_core_pipe_tx_write(&core, 1, long_stream,
            sizeof(long_stream)) == sizeof(long_stream));
        assert(core.tx.kind == TRANSPORT_TX_PIPE && fake_tx_count == 3);
        assert(transport_ring_readable(&pipe->buffer) ==
            sizeof(long_stream) - 3 * TRANSPORT_WIRE_MAX_DATA);
        writes_before_refill = captured_tx_count;
        ack_tx(&core);
        assert(fake_tx_count == 3);
        assert(captured_tx_count == writes_before_refill + 1);
        assert(transport_ring_readable(&pipe->buffer) ==
            sizeof(long_stream) - 4 * TRANSPORT_WIRE_MAX_DATA);

        assert(transport_core_send_command(&core, 8,
            TRANSPORT_WIRE_COMMAND, 44, first, sizeof(first)));
        assert(core.tx.kind == TRANSPORT_TX_PIPE && core.tx.drain_requested);
        ack_tx(&core);
        ack_tx(&core);
        ack_tx(&core);
        assert(core.tx.kind == TRANSPORT_TX_CONTROL);
        assert(captured_address[0] == 8);
        ack_tx(&core);
        assert(core.commands[0].state == TRANSPORT_COMMAND_TX_WAIT_CTS);
        assert(fake_rx_mode);

        make_cts(packet, config.network_id, 8, 1, 44,
            TRANSPORT_WIRE_COMMAND, TRANSPORT_CTS_ACCEPTED, sizeof(first));
        deliver(&core, packet);
        fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
        transport_core_service(&core);
        assert(core.tx.kind == TRANSPORT_TX_COMMAND);
        assert(captured_address[0] == 8);
        ack_tx(&core);
        assert(core.tx.kind == TRANSPORT_TX_PIPE);
        assert(captured_address[0] == 9);
        ack_tx(&core);
        assert(pipe->transferred_bytes == sizeof(long_stream));
        assert(transport_core_close_pipe(&core, 1));
        assert(!core.tx.active);
        make_cts(packet, config.network_id, 9, 1, 44,
            TRANSPORT_WIRE_STREAM, TRANSPORT_CTS_ACCEPTED,
            2 * sizeof(long_stream));
        deliver(&core, packet);
        fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
        transport_core_service(&core);
        ack_tx(&core);
        deliver_close_ack(&core, config.network_id, 9, 1, 44);

        while (transport_core_poll_into(&core, event_buffer,
                sizeof(event_buffer), &event_length, &required) ==
                TRANSPORT_POLL_EVENT) {
            if (transport_event_get_type(event_buffer) ==
                    TRANSPORT_EVENT_COMMAND_SENT) saw_command_sent = true;
            if (transport_event_get_type(event_buffer) ==
                    TRANSPORT_EVENT_PIPE_CLOSED) saw_pipe_closed = true;
        }
        assert(saw_command_sent && saw_pipe_closed);
        assert(core.pipes[1].state == TRANSPORT_PIPE_FREE);
    }

    {
        int pipe_id = transport_core_open_pipe(&core, 9, 42);
        assert(pipe_id == 0);
        ack_tx(&core);
        make_cts(packet, config.network_id, 9, 1, 42,
            TRANSPORT_WIRE_STREAM, TRANSPORT_CTS_BUSY, 0);
        deliver(&core, packet);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_FAILED);
        assert(transport_event_get_value0(event_buffer) ==
            TRANSPORT_PIPE_FAILURE_CTS_BASE + TRANSPORT_CTS_BUSY);
    }

    /* Reading a live RX stream replenishes cumulative credit only after the
       current grant is exhausted, avoiding a half-duplex turnaround while
       the sender can still be transmitting. */
    make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1, 43,
        TRANSPORT_WIRE_INTENT | TRANSPORT_WIRE_LAST_PACKET, NULL, 0);
    deliver(&core, packet);
    ack_tx(&core);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) ==
        TRANSPORT_EVENT_PIPE_OPENED);
    make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1, 43,
        0, outbound, 26);
    deliver(&core, packet);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) ==
        TRANSPORT_EVENT_PIPE_RX_DATA);
    assert(!core.tx.active);
    make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1, 43,
        0, outbound + 26, 6);
    deliver(&core, packet);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) ==
        TRANSPORT_EVENT_PIPE_RX_DATA);
    assert(core.tx.kind == TRANSPORT_TX_CONTROL);
    assert(captured_tx[5] == TRANSPORT_WIRE_STREAM);
    assert(captured_tx[6] == TRANSPORT_CTS_ACCEPTED);
    assert(captured_tx[7] == 64 && captured_tx[8] == 0);
    ack_tx(&core);
    make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1, 43,
        TRANSPORT_WIRE_LAST_PACKET, NULL, 0);
    deliver(&core, packet);
    assert((captured_tx[0] & TRANSPORT_WIRE_TYPE_MASK) ==
        TRANSPORT_WIRE_PIPE_CLOSE_ACK);
    ack_tx(&core);
    assert(transport_core_poll_into(&core, event_buffer, sizeof(event_buffer),
        &event_length, &required) == TRANSPORT_POLL_EVENT);
    assert(transport_event_get_type(event_buffer) ==
        TRANSPORT_EVENT_PIPE_CLOSED);

    /* A short-lived tombstone confirms a duplicate close after the original
       event and ACK have already released the receive buffer. */
    assert(core.pipes[0].state == TRANSPORT_PIPE_FREE);
    make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1, 43,
        TRANSPORT_WIRE_LAST_PACKET, NULL, 0);
    deliver(&core, packet);
    assert(core.tx.kind == TRANSPORT_TX_CONTROL);
    assert((captured_tx[0] & TRANSPORT_WIRE_TYPE_MASK) ==
        TRANSPORT_WIRE_PIPE_CLOSE_ACK);
    ack_tx(&core);
    assert(transport_core_poll_into(&core, event_buffer,
        sizeof(event_buffer), &event_length, &required) ==
        TRANSPORT_POLL_EMPTY);

    /* A receiver retains terminal identity while its close confirmation is
       retrying, so MAX_RT cannot strand the sender waiting for confirmation. */
    {
        transport_pipe_slot_t *pipe = &core.pipes[0];
        uint16_t saved_lease = core.config.pipe_lease_ms;
        core.config.pipe_lease_ms = 500;
        transport_ring_reset(&pipe->buffer);
        pipe->peer_id = 7;
        pipe->session_id = 50;
        pipe->direction = TRANSPORT_DIRECTION_RX;
        pipe->state = TRANSPORT_PIPE_CLOSED;
        pipe->close_ack_pending = true;
        pipe->close_ack_queued = false;
        pipe->terminal_event_polled = true;
        pipe->lease_deadline_ms = fake_now + core.config.pipe_lease_ms;

        make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1,
            50, TRANSPORT_WIRE_LAST_PACKET, NULL, 0);
        deliver(&core, packet);
        assert(core.tx.kind == TRANSPORT_TX_CONTROL);
        assert((captured_tx[0] & TRANSPORT_WIRE_TYPE_MASK) ==
            TRANSPORT_WIRE_PIPE_CLOSE_ACK);
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(core.tx.active);
        fake_now += core.config.max_rt_window_ms;
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(!core.tx.active);
        make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1,
            50, TRANSPORT_WIRE_LAST_PACKET, NULL, 0);
        deliver(&core, packet);
        assert(core.tx.active);
        assert((captured_tx[0] & TRANSPORT_WIRE_TYPE_MASK) ==
            TRANSPORT_WIRE_PIPE_CLOSE_ACK);
        ack_tx(&core);
        assert(pipe->state == TRANSPORT_PIPE_FREE);
        core.config.pipe_lease_ms = saved_lease;
    }

    /* If the sender disappears after an ACK failure, passive duplicate wait
       still expires and releases the receive slot. */
    {
        transport_pipe_slot_t *pipe = &core.pipes[0];
        transport_ring_reset(&pipe->buffer);
        pipe->peer_id = 7;
        pipe->session_id = 52;
        pipe->direction = TRANSPORT_DIRECTION_RX;
        pipe->state = TRANSPORT_PIPE_CLOSED;
        pipe->close_ack_pending = true;
        pipe->close_ack_queued = false;
        pipe->close_ack_wait_duplicate = true;
        pipe->terminal_event_polled = true;
        pipe->lease_deadline_ms = fake_now + core.config.pipe_lease_ms;
        fake_now += core.config.pipe_lease_ms;
        transport_core_service(&core);
        assert(pipe->state == TRANSPORT_PIPE_FREE);
    }

    /* A terminal RX event must survive temporary event-queue saturation.
       Once foreground polling frees one descriptor, service retries it. */
    {
        transport_event_fields_t filler;
        uint32_t overflows_before;
        memset(&filler, 0, sizeof(filler));
        filler.type = TRANSPORT_EVENT_CORE_ERROR;
        make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1,
            46, TRANSPORT_WIRE_INTENT | TRANSPORT_WIRE_LAST_PACKET, NULL, 0);
        deliver(&core, packet);
        ack_tx(&core);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_OPENED);
        for (i = 0; i < TRANSPORT_CORE_EVENT_QUEUE_SIZE; ++i) {
            assert(transport_core_emit(&core, &filler));
        }
        overflows_before = core.stats.event_queue_overflows;
        make_packet(packet, config.network_id, TRANSPORT_WIRE_STREAM, 7, 1,
            46, TRANSPORT_WIRE_LAST_PACKET, NULL, 0);
        deliver(&core, packet);
        assert((captured_tx[0] & TRANSPORT_WIRE_TYPE_MASK) ==
            TRANSPORT_WIRE_PIPE_CLOSE_ACK);
        ack_tx(&core);
        assert(core.pipes[0].state == TRANSPORT_PIPE_CLOSED);
        assert(core.pipes[0].close_event_pending);
        assert(core.stats.event_queue_overflows > overflows_before);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_CORE_ERROR);
        transport_core_service(&core);
        assert(!core.pipes[0].close_event_pending);
        for (i = 1; i < TRANSPORT_CORE_EVENT_QUEUE_SIZE; ++i) {
            assert(transport_core_poll_into(&core, event_buffer,
                sizeof(event_buffer), &event_length, &required) ==
                TRANSPORT_POLL_EVENT);
            assert(transport_event_get_type(event_buffer) ==
                TRANSPORT_EVENT_CORE_ERROR);
        }
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_CLOSED);
        assert(core.pipes[0].state == TRANSPORT_PIPE_FREE);
    }

    {
        int pipe_id = transport_core_open_pipe(&core, 9, 44);
        assert(pipe_id == 0);
        ack_tx(&core);
        fake_now += config.pipe_lease_ms;
        transport_core_service(&core);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_FAILED);
        assert(transport_event_get_value0(event_buffer) ==
            TRANSPORT_PIPE_FAILURE_TIMEOUT);
    }

    /* An accepted outbound stream must retain an idle lease.  Otherwise a
       vanished application can leave the slot allocated forever.  Its
       failure event must also survive a temporarily full event queue. */
    {
        int pipe_id = transport_core_open_pipe(&core, 9, 45);
        transport_event_fields_t filler;
        assert(pipe_id == 0);
        ack_tx(&core);
        make_cts(packet, config.network_id, 9, 1, 45,
            TRANSPORT_WIRE_STREAM, TRANSPORT_CTS_ACCEPTED, 32);
        deliver(&core, packet);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_OPENED);
        assert(transport_core_pipe_tx_write(&core, (uint8_t)pipe_id,
            outbound, 10) == 10);
        fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
        transport_core_service(&core);
        ack_tx(&core);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_TX_SPACE);
        memset(&filler, 0, sizeof(filler));
        filler.type = TRANSPORT_EVENT_CORE_ERROR;
        for (i = 0; i < TRANSPORT_CORE_EVENT_QUEUE_SIZE; ++i) {
            assert(transport_core_emit(&core, &filler));
        }
        fake_now += config.pipe_lease_ms;
        transport_core_service(&core);
        assert(core.pipes[pipe_id].state == TRANSPORT_PIPE_FAILED);
        assert(core.pipes[pipe_id].close_event_pending);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_CORE_ERROR);
        transport_core_service(&core);
        assert(!core.pipes[pipe_id].close_event_pending);
        for (i = 1; i < TRANSPORT_CORE_EVENT_QUEUE_SIZE; ++i) {
            assert(transport_core_poll_into(&core, event_buffer,
                sizeof(event_buffer), &event_length, &required) ==
                TRANSPORT_POLL_EVENT);
            assert(transport_event_get_type(event_buffer) ==
                TRANSPORT_EVENT_CORE_ERROR);
        }
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_PIPE_FAILED);
        assert(transport_event_get_value0(event_buffer) ==
            TRANSPORT_PIPE_FAILURE_TIMEOUT);
        assert(transport_event_get_value1(event_buffer) == 10);
        assert(core.pipes[pipe_id].state == TRANSPORT_PIPE_FREE);
    }

    assert(core.stats.pipes_opened == 5);
    assert(core.stats.pipes_closed == 8);
    assert(core.stats.pipes_failed == 4);

    /* A continuously supplied pipe drains at the fairness deadline, remains
       in RX for the configured dwell, and then resumes the same ring. */
    {
        transport_pipe_slot_t *pipe = &core.pipes[1];
        transport_ring_reset(&pipe->buffer);
        pipe->peer_id = 9;
        pipe->session_id = 45;
        pipe->granted_bytes = sizeof(long_stream);
        pipe->transferred_bytes = 0;
        pipe->direction = TRANSPORT_DIRECTION_TX;
        pipe->state = TRANSPORT_PIPE_TX_OPEN;
        pipe->lease_deadline_ms = fake_now + config.pipe_lease_ms;
        assert(transport_core_pipe_tx_write(&core, 1, long_stream,
            sizeof(long_stream)) == sizeof(long_stream));
        assert(fake_tx_count == 3);

        fake_now += TRANSPORT_CORE_DEFAULT_MAX_CONTINUOUS_TX_MS;
        ack_tx(&core);
        assert(core.radio_phase == TRANSPORT_RADIO_TX_DRAINING);
        assert(transport_ring_readable(&pipe->buffer) ==
            sizeof(long_stream) - 3 * TRANSPORT_WIRE_MAX_DATA);
        ack_tx(&core);
        ack_tx(&core);
        assert(core.radio_phase == TRANSPORT_RADIO_RX_YIELD);
        assert(fake_rx_mode && !core.tx.active);

        fake_now += TRANSPORT_CORE_DEFAULT_FORCED_RX_MS - 1;
        transport_core_service(&core);
        assert(core.radio_phase == TRANSPORT_RADIO_RX_YIELD);
        fake_now++;
        transport_core_service(&core);
        assert(core.tx.kind == TRANSPORT_TX_PIPE);
        assert(fake_tx_count == 2);
        ack_tx(&core);
        ack_tx(&core);
        assert(pipe->transferred_bytes == sizeof(long_stream));
        assert(fake_rx_mode && !core.tx.active);
        assert(transport_core_close_pipe(&core, 1));
        assert(!core.tx.active);
        make_cts(packet, config.network_id, 9, 1, 45,
            TRANSPORT_WIRE_STREAM, TRANSPORT_CTS_ACCEPTED,
            2 * sizeof(long_stream));
        deliver(&core, packet);
        fake_now += TRANSPORT_CORE_CTS_TURNAROUND_MS;
        transport_core_service(&core);
        ack_tx(&core);
        deliver_close_ack(&core, config.network_id, 9, 1, 45);
        while (transport_core_poll_into(&core, event_buffer,
                sizeof(event_buffer), &event_length, &required) ==
                TRANSPORT_POLL_EVENT) {
        }
        assert(core.pipes[1].state == TRANSPORT_PIPE_FREE);
    }

    /* Registration datagrams use explicit RF addresses while preserving the
       logical source/destination fields used by Python registration policy. */
    {
        const uint8_t uuid[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
        uint8_t assignment[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 5 };
        const uint8_t temporary[NRF24_ADDR_LEN] =
            { 0x91, 0x82, 0x73, 0x64, 0x55 };
        uint8_t registration[NRF24_ADDR_LEN];
        uint8_t master_address[NRF24_ADDR_LEN];
        uint32_t protocol_before;

        registration[0] = TRANSPORT_CORE_INVALID_ID;
        master_address[0] = 0;
        memcpy(registration + 1, config.network_id, 4);
        memcpy(master_address + 1, config.network_id, 4);

        transport_core_stop(&core);
        config.node_id = 0;
        config.has_service_address = false;
        transport_core_init(&core, &config);
        assert(transport_core_attach_command_buffer(&core, 0,
            command_buffer_small, sizeof(command_buffer_small)));
        assert(transport_core_attach_command_buffer(&core, 1,
            command_buffer_large, sizeof(command_buffer_large)));
        for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
            assert(transport_core_attach_pipe_buffer(&core, i,
                pipe_buffers[i], i == 0 ? 32 : sizeof(pipe_buffers[i])));
        }
        assert(transport_core_attach_control_tx_buffer(&core, control_buffer,
            sizeof(control_buffer)));
        captured_rx_open = 0;
        assert(transport_core_start(&core));
        assert((captured_rx_open & (1u << 1)) != 0);
        assert((captured_rx_open & (1u << 2)) != 0);
        assert(memcmp(captured_rx_address[1], master_address,
            NRF24_ADDR_LEN) == 0);
        assert(memcmp(captured_rx_address[2], registration,
            NRF24_ADDR_LEN) == 0);

        make_packet(packet, config.network_id, TRANSPORT_WIRE_ENUM_HELLO,
            TRANSPORT_CORE_INVALID_ID, 0, 50,
            TRANSPORT_WIRE_LAST_PACKET, uuid, sizeof(uuid));
        deliver(&core, packet);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_REGISTRATION_RECEIVED);
        assert(transport_event_get_object_id(event_buffer) ==
            TRANSPORT_WIRE_ENUM_HELLO);
        assert(transport_event_get_source_id(event_buffer) ==
            TRANSPORT_CORE_INVALID_ID);
        assert(transport_event_get_transaction_id(event_buffer) == 50);
        assert(transport_event_get_value0(event_buffer) == 0);
        assert(transport_event_get_data_length(event_buffer) == sizeof(uuid));
        assert(memcmp(transport_event_get_data(event_buffer), uuid,
            sizeof(uuid)) == 0);

        protocol_before = core.stats.protocol_errors;
        make_packet(packet, config.network_id, TRANSPORT_WIRE_ENUM_HELLO,
            TRANSPORT_CORE_INVALID_ID, 0, 51,
            TRANSPORT_WIRE_INTENT | TRANSPORT_WIRE_LAST_PACKET,
            uuid, sizeof(uuid));
        deliver(&core, packet);
        assert(core.stats.protocol_errors == protocol_before + 1);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EMPTY);

        assert(transport_core_send_registration(&core,
            TRANSPORT_WIRE_ENUM_ASSIGN, TRANSPORT_CORE_INVALID_ID, 52,
            temporary, assignment, sizeof(assignment)));
        assert(!transport_core_send_registration(&core,
            TRANSPORT_WIRE_ENUM_ASSIGN, TRANSPORT_CORE_INVALID_ID, 53,
            temporary, assignment, sizeof(assignment)));
        assert(core.tx.kind == TRANSPORT_TX_REGISTRATION);
        assert(memcmp(captured_address, temporary, NRF24_ADDR_LEN) == 0);
        assert((captured_tx[0] & TRANSPORT_WIRE_TYPE_MASK) ==
            TRANSPORT_WIRE_ENUM_ASSIGN);
        assert(captured_tx[1] == 0 &&
            captured_tx[2] == TRANSPORT_CORE_INVALID_ID);
        ack_tx(&core);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_REGISTRATION_SENT);
        assert((transport_event_get_flags(event_buffer) &
            TRANSPORT_EVENT_FLAG_TX) != 0);
        assert(transport_event_get_object_id(event_buffer) ==
            TRANSPORT_WIRE_ENUM_ASSIGN);

        make_packet(packet, config.network_id, TRANSPORT_WIRE_ENUM_CONFIRM,
            5, 0, 54, TRANSPORT_WIRE_LAST_PACKET,
            assignment, sizeof(assignment));
        deliver(&core, packet);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_REGISTRATION_RECEIVED);
        assert(transport_event_get_object_id(event_buffer) ==
            TRANSPORT_WIRE_ENUM_CONFIRM);

        assert(transport_core_send_registration(&core,
            TRANSPORT_WIRE_ENUM_ASSIGN, TRANSPORT_CORE_INVALID_ID, 55,
            temporary, assignment, sizeof(assignment)));
        for (i = 0; i < config.max_rt_restarts; ++i) {
            fake_status |= NRF24_STATUS_MAX_RT;
            transport_core_on_radio_irq(&core);
            assert(core.registration_tx.pending);
        }
        fake_status |= NRF24_STATUS_MAX_RT;
        transport_core_on_radio_irq(&core);
        assert(!core.registration_tx.pending);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_REGISTRATION_FAILED);
        assert(transport_event_get_value1(event_buffer) ==
            TRANSPORT_REGISTRATION_FAILURE_RADIO);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_CORE_ERROR);

        transport_core_stop(&core);
        config.node_id = TRANSPORT_CORE_INVALID_ID;
        memcpy(config.service_address, temporary, NRF24_ADDR_LEN);
        config.has_service_address = true;
        transport_core_init(&core, &config);
        assert(transport_core_attach_command_buffer(&core, 0,
            command_buffer_small, sizeof(command_buffer_small)));
        assert(transport_core_attach_command_buffer(&core, 1,
            command_buffer_large, sizeof(command_buffer_large)));
        for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
            assert(transport_core_attach_pipe_buffer(&core, i,
                pipe_buffers[i], i == 0 ? 32 : sizeof(pipe_buffers[i])));
        }
        assert(transport_core_attach_control_tx_buffer(&core, control_buffer,
            sizeof(control_buffer)));
        captured_rx_open = 0;
        assert(transport_core_start(&core));
        assert(captured_rx_open == (1u << 1));
        assert(memcmp(captured_rx_address[1], temporary,
            NRF24_ADDR_LEN) == 0);

        make_packet(packet, config.network_id, TRANSPORT_WIRE_ENUM_ASSIGN,
            0, TRANSPORT_CORE_INVALID_ID, 56,
            TRANSPORT_WIRE_LAST_PACKET, assignment, sizeof(assignment));
        deliver(&core, packet);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_REGISTRATION_RECEIVED);
        assert(transport_event_get_object_id(event_buffer) ==
            TRANSPORT_WIRE_ENUM_ASSIGN);
        assert(transport_core_set_node_id(&core, 5));
        master_address[0] = 5;
        assert(memcmp(captured_rx_address[1], master_address,
            NRF24_ADDR_LEN) == 0);

        master_address[0] = 0;
        assert(transport_core_send_registration(&core,
            TRANSPORT_WIRE_ENUM_CONFIRM, 0, 57, master_address,
            assignment, sizeof(assignment)));
        assert(!transport_core_set_node_id(&core,
            TRANSPORT_CORE_INVALID_ID));
        ack_tx(&core);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_REGISTRATION_SENT);
        assert(transport_core_set_node_id(&core,
            TRANSPORT_CORE_INVALID_ID));
        assert(memcmp(captured_rx_address[1], temporary,
            NRF24_ADDR_LEN) == 0);
        assert(transport_core_send_registration(&core,
            TRANSPORT_WIRE_ENUM_HELLO, 0, 58, registration,
            uuid, sizeof(uuid)));
        assert(memcmp(captured_address, registration, NRF24_ADDR_LEN) == 0);
        assert(captured_tx[1] == TRANSPORT_CORE_INVALID_ID &&
            captured_tx[2] == 0);
        ack_tx(&core);
        assert(transport_core_poll_into(&core, event_buffer,
            sizeof(event_buffer), &event_length, &required) ==
            TRANSPORT_POLL_EVENT);
        assert(transport_event_get_type(event_buffer) ==
            TRANSPORT_EVENT_REGISTRATION_SENT);
    }

    puts("host transport core tests passed");
    return 0;
}
