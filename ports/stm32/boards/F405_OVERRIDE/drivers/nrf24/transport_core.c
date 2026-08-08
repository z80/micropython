#include "transport_core.h"

#include <limits.h>
#include <string.h>

#include "nrf24_regs.h"

#define TRANSPORT_WIRE_UNASSIGNED_ID (0xffu)
#define TRANSPORT_IRQ_MAX_RX_PAYLOADS (3u)
#define TRANSPORT_CONTROL_RECORD_SIZE (TRANSPORT_CORE_CONTROL_RECORD_SIZE)

static void core_control_acked(transport_core_t *, const uint8_t *);
static void core_control_failed(transport_core_t *, const uint8_t *);
static void core_finish_outbound_command(transport_core_t *, uint8_t, bool,
    uint32_t);
static void core_finish_outbound_pipe(transport_core_t *, uint8_t, bool,
    uint32_t);
static void core_finish_registration_tx(transport_core_t *, bool, uint32_t);
static bool core_queue_packet(transport_core_t *, uint8_t, uint8_t, uint8_t,
    uint8_t, const uint8_t *, size_t);
static bool queue_pipe_tx_space_event(transport_core_t *, uint8_t);
static bool core_emit_pipe_terminal(transport_core_t *, uint8_t);
static bool core_queue_pipe_close_ack(transport_core_t *, uint8_t);
static int core_find_pipe_close_tombstone(transport_core_t *, uint8_t,
    uint8_t);
static bool core_queue_tombstone_close_ack(transport_core_t *, uint8_t);
static void core_maybe_queue_tx_credit_request(transport_core_t *, uint8_t,
    uint32_t);

static uint32_t core_now_ms(transport_core_t *core) {
    if (core->config.radio.ticks_ms == NULL) return 0;
    return core->config.radio.ticks_ms(core->config.radio.context);
}

static bool core_deadline_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static uint32_t core_pipe_close_retry_delay(const transport_core_t *core) {
    uint32_t delay = core->config.max_rt_window_ms;
    if (delay == 0) delay = 1;
    return delay + TRANSPORT_CORE_CTS_TURNAROUND_MS;
}

static bool core_radio_io_ok(transport_core_t *core) {
    return core->config.radio.io_ok == NULL ||
        core->config.radio.io_ok(core->config.radio.context);
}

static bool core_radio_irq_is_low(transport_core_t *core) {
    return core->config.radio.irq_is_low != NULL &&
        core->config.radio.irq_is_low(core->config.radio.context);
}

static uint32_t core_crc32_update(uint32_t crc, const uint8_t *data,
        size_t length) {
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

static uint16_t core_read_le16(const uint8_t *data) {
    return (uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8);
}

static uint32_t core_read_le32(const uint8_t *data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void core_write_le16(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void core_write_le32(uint8_t *data, uint32_t value) {
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static transport_critical_state_t core_enter(transport_core_t *core) {
    if (core->config.radio.critical_enter == NULL) return 0;
    return core->config.radio.critical_enter(core->config.radio.context);
}

static void core_exit(transport_core_t *core, transport_critical_state_t state) {
    if (core->config.radio.critical_exit != NULL)
        core->config.radio.critical_exit(core->config.radio.context, state);
}

static bool core_build_packet(transport_core_t *core, uint8_t message_type,
        uint8_t destination_id, uint8_t message_id, uint8_t wire_flags,
        const uint8_t *payload, size_t payload_length, uint8_t *packet) {
    uint32_t crc;
    if (core == NULL || packet == NULL || message_type > TRANSPORT_WIRE_TYPE_MASK ||
            payload_length > TRANSPORT_WIRE_MAX_DATA ||
            (payload == NULL && payload_length != 0) ||
            (wire_flags & 0x5fu) != 0) {
        return false;
    }
    memset(packet, 0, NRF24_MAX_PAYLOAD);
    packet[0] = TRANSPORT_WIRE_PROTOCOL_ID | message_type;
    packet[1] = core->config.node_id;
    packet[2] = destination_id;
    packet[3] = message_id;
    packet[4] = wire_flags | (uint8_t)payload_length;
    if (payload_length != 0) {
        memcpy(packet + TRANSPORT_WIRE_HEADER_SIZE, payload, payload_length);
    }
    crc = core_crc32_update(0, core->config.network_id,
        sizeof(core->config.network_id));
    crc = core_crc32_update(crc, packet, TRANSPORT_WIRE_HEADER_SIZE);
    crc = core_crc32_update(crc, packet + TRANSPORT_WIRE_HEADER_SIZE,
        payload_length);
    packet[TRANSPORT_WIRE_HEADER_SIZE + payload_length] = (uint8_t)crc;
    return true;
}

static void core_restore_rx(transport_core_t *core) {
    transport_critical_state_t critical = core_enter(core);
    nrf24_enter_rx_mode(core->config.radio.radio);
    core->radio_phase = TRANSPORT_RADIO_RX_IDLE;
    core->tx_campaign_active = false;
    core_exit(core, critical);
}

static void core_release_tx_owner(transport_core_t *core) {
    memset(&core->tx, 0, sizeof(core->tx));
    core->tx.kind = TRANSPORT_TX_NONE;
}

static void core_enter_forced_rx(transport_core_t *core, uint32_t now) {
    core_release_tx_owner(core);
    nrf24_enter_rx_mode(core->config.radio.radio);
    core->radio_phase = TRANSPORT_RADIO_RX_YIELD;
    core->tx_campaign_active = false;
    core->rx_yield_started_ms = now;
    core->rx_yield_deadline_ms = now + core->forced_rx_ms;
}

static bool core_tx_time_expired(transport_core_t *core, uint32_t now) {
    return core->tx_campaign_active && core->max_continuous_tx_ms != 0 &&
        core_deadline_reached(now,
            core->tx_campaign_started_ms + core->max_continuous_tx_ms);
}

static void core_commit_staged_tx(transport_core_t *core) {
    if (core->tx.kind == TRANSPORT_TX_COMMAND &&
            core->tx.slot < TRANSPORT_CORE_COMMAND_SLOTS) {
        core->commands[core->tx.slot].transferred_length +=
            (uint16_t)core->tx.staged_payload_bytes;
    } else if (core->tx.kind == TRANSPORT_TX_PIPE &&
            core->tx.slot < TRANSPORT_CORE_PIPE_SLOTS) {
        core->pipes[core->tx.slot].transferred_bytes +=
            core->tx.staged_payload_bytes;
        core->stats.pipe_tx_bytes += core->tx.staged_payload_bytes;
    }
    core->stats.tx_packets += core->tx.staged_packets;
    core->tx.staged_payload_bytes = 0;
    core->tx.staged_packets = 0;
}

static void core_fail_active_tx(transport_core_t *core, uint32_t detail) {
    transport_tx_kind_t kind = core->tx.kind;
    uint8_t slot = core->tx.slot;
    nrf24_abort_send(core->config.radio.radio);
    if (kind == TRANSPORT_TX_COMMAND && slot < TRANSPORT_CORE_COMMAND_SLOTS) {
        core_finish_outbound_command(core, slot, false,
            TRANSPORT_COMMAND_FAILURE_RADIO);
    } else if (kind == TRANSPORT_TX_PIPE &&
            slot < TRANSPORT_CORE_PIPE_SLOTS) {
        core_finish_outbound_pipe(core, slot, false,
            TRANSPORT_PIPE_FAILURE_RADIO);
    }
    core_release_tx_owner(core);
    core_restore_rx(core);
    transport_core_report_error(core, TRANSPORT_ERROR_RADIO,
        TRANSPORT_CORE_INVALID_ID, detail);
}

static bool core_application_waiting_cts(const transport_core_t *core) {
    uint8_t slot;
    for (slot = 0; slot < TRANSPORT_CORE_COMMAND_SLOTS; ++slot) {
        if (core->commands[slot].direction == TRANSPORT_DIRECTION_TX &&
                core->commands[slot].state ==
                    TRANSPORT_COMMAND_TX_WAIT_CTS) return true;
    }
    for (slot = 0; slot < TRANSPORT_CORE_PIPE_SLOTS; ++slot) {
        if (core->pipes[slot].direction == TRANSPORT_DIRECTION_TX &&
                core->pipes[slot].state ==
                    TRANSPORT_PIPE_TX_WAIT_CTS) return true;
    }
    return false;
}

/* Fill no more than one hardware FIFO depth per invocation.  CE may already
   be high, so an unbounded "until full" loop could otherwise race the radio. */
static bool core_fill_tx_owner(transport_core_t *core) {
    uint8_t writes;
    nrf24_t *radio = core->config.radio.radio;
    for (writes = 0; writes < 3 && !core->tx.drain_requested &&
            !core->tx.last_packet && !nrf24_tx_fifo_full(radio); ++writes) {
        uint8_t packet[NRF24_MAX_PAYLOAD];
        uint8_t fragment[TRANSPORT_WIRE_MAX_DATA];
        size_t payload_length = 0;
        bool last_packet = false;

        if (core->tx.kind == TRANSPORT_TX_COMMAND &&
                core->tx.slot < TRANSPORT_CORE_COMMAND_SLOTS) {
            transport_command_slot_t *command =
                &core->commands[core->tx.slot];
            size_t remaining = transport_ring_readable(&command->buffer);
            payload_length = remaining > TRANSPORT_WIRE_MAX_DATA ?
                TRANSPORT_WIRE_MAX_DATA : remaining;
            if (payload_length != 0 && transport_ring_peek(&command->buffer,
                    fragment, payload_length) != payload_length) return false;
            last_packet = remaining <= TRANSPORT_WIRE_MAX_DATA;
            if (!core_build_packet(core, command->message_type,
                    command->peer_id, (uint8_t)command->transaction_id,
                    last_packet ? TRANSPORT_WIRE_LAST_PACKET : 0,
                    fragment, payload_length, packet)) return false;
            if (!nrf24_write_payload(radio, packet, sizeof(packet))) return false;
            (void)transport_ring_discard(&command->buffer, payload_length);
        } else if (core->tx.kind == TRANSPORT_TX_PIPE &&
                core->tx.slot < TRANSPORT_CORE_PIPE_SLOTS) {
            transport_pipe_slot_t *pipe = &core->pipes[core->tx.slot];
            size_t remaining = transport_ring_readable(&pipe->buffer);
            uint32_t used_credit = pipe->transferred_bytes +
                core->tx.staged_payload_bytes;
            uint32_t credit = pipe->granted_bytes > used_credit ?
                pipe->granted_bytes - used_credit : 0;
            if (remaining != 0 && credit != 0) {
                payload_length = remaining > TRANSPORT_WIRE_MAX_DATA ?
                    TRANSPORT_WIRE_MAX_DATA : remaining;
                if (payload_length > credit) payload_length = (size_t)credit;
                if (transport_ring_peek(&pipe->buffer, fragment,
                        payload_length) != payload_length ||
                        !core_build_packet(core, TRANSPORT_WIRE_STREAM,
                            pipe->peer_id, (uint8_t)pipe->session_id, 0,
                            fragment, payload_length, packet)) return false;
                if (!nrf24_write_payload(radio, packet, sizeof(packet))) return false;
                (void)transport_ring_discard(&pipe->buffer, payload_length);
                (void)queue_pipe_tx_space_event(core, core->tx.slot);
            } else if (pipe->state == TRANSPORT_PIPE_TX_CLOSING &&
                    remaining == 0) {
                /* Never append the terminal packet behind pipe data already
                   in the hardware FIFO.  Drain and commit that data batch;
                   core_tx_kick() will then give the close packet its own
                   FIFO ownership period. */
                if (core->tx.staged_payload_bytes != 0) {
                    core->tx.drain_requested = true;
                    core->radio_phase = TRANSPORT_RADIO_TX_DRAINING;
                    break;
                }
                /* With an exhausted grant, the receiver may be turning
                   around to send replenishment.  Wait for that CTS before
                   placing the terminal packet in a new FIFO batch. */
                if (credit == 0) break;
                if (!core_build_packet(core, TRANSPORT_WIRE_STREAM,
                        pipe->peer_id, (uint8_t)pipe->session_id,
                        TRANSPORT_WIRE_LAST_PACKET, NULL, 0, packet) ||
                        !nrf24_write_payload(radio, packet, sizeof(packet))) {
                    return false;
                }
                last_packet = true;
            } else {
                break;
            }
        } else {
            break;
        }
        core->tx.payload_length = (uint8_t)payload_length;
        core->tx.staged_payload_bytes += (uint32_t)payload_length;
        core->tx.staged_packets++;
        core->tx.last_packet = last_packet;
    }
    return core_radio_io_ok(core);
}

static void core_tx_kick(transport_core_t *core) {
    uint8_t packet[NRF24_MAX_PAYLOAD];
    uint8_t address[NRF24_ADDR_LEN];
    uint8_t slot;
    bool application_tx_ready;
    transport_tx_kind_t kind = TRANSPORT_TX_NONE;
    transport_critical_state_t critical;
    nrf24_t *radio;
    if (core == NULL) return;
    critical = core_enter(core);
    if (!core->started || core->radio_phase == TRANSPORT_RADIO_RX_YIELD) {
        core_exit(core, critical);
        return;
    }
    if (core->tx.active) {
        bool expired = core_tx_time_expired(core, core_now_ms(core));
        if (!expired && !core->tx.drain_requested) {
            if (!core_fill_tx_owner(core)) {
                core_fail_active_tx(core, 0x46494cu);
            }
            core_exit(core, critical);
            return;
        }
        if (expired) {
            core->tx.drain_requested = true;
            core->radio_phase = TRANSPORT_RADIO_TX_DRAINING;
        }
        core_exit(core, critical);
        return;
    }
    if (transport_ring_readable(&core->control_tx) >=
            TRANSPORT_CONTROL_RECORD_SIZE &&
            transport_ring_peek(&core->control_tx, packet,
                sizeof(packet)) == sizeof(packet)) {
        kind = TRANSPORT_TX_CONTROL;
        slot = TRANSPORT_CORE_INVALID_ID;
    }
    if (kind == TRANSPORT_TX_NONE && core->registration_tx.pending) {
        memcpy(packet, core->registration_tx.packet, sizeof(packet));
        memcpy(address, core->registration_tx.address, sizeof(address));
        kind = TRANSPORT_TX_REGISTRATION;
        slot = TRANSPORT_CORE_INVALID_ID;
    }
    application_tx_ready = core_deadline_reached(core_now_ms(core),
        core->application_tx_not_before_ms) &&
        !core_application_waiting_cts(core);
    for (slot = 0; kind == TRANSPORT_TX_NONE && application_tx_ready &&
            slot < TRANSPORT_CORE_COMMAND_SLOTS; ++slot) {
        transport_command_slot_t *command = &core->commands[slot];
            if (command->direction == TRANSPORT_DIRECTION_TX &&
                command->state == TRANSPORT_COMMAND_TX_SENDING) {
            kind = TRANSPORT_TX_COMMAND;
            break;
        }
    }
    if (kind == TRANSPORT_TX_NONE && application_tx_ready) {
        for (slot = 0; slot < TRANSPORT_CORE_PIPE_SLOTS; ++slot) {
            transport_pipe_slot_t *pipe = &core->pipes[slot];
            if (pipe->direction == TRANSPORT_DIRECTION_TX &&
                    (pipe->state == TRANSPORT_PIPE_TX_OPEN ||
                     pipe->state == TRANSPORT_PIPE_TX_CLOSING)) {
                size_t remaining = transport_ring_readable(&pipe->buffer);
                uint32_t credit = pipe->granted_bytes > pipe->transferred_bytes ?
                    pipe->granted_bytes - pipe->transferred_bytes : 0;
                if (remaining != 0 && credit != 0) {
                    kind = TRANSPORT_TX_PIPE;
                    break;
                }
                if (pipe->state == TRANSPORT_PIPE_TX_CLOSING &&
                        remaining == 0 && credit != 0) {
                    kind = TRANSPORT_TX_PIPE;
                    break;
                }
            }
        }
    }
    if (kind == TRANSPORT_TX_NONE) {
        for (slot = 0; slot < TRANSPORT_CORE_COMMAND_SLOTS; ++slot) {
            if (core->commands[slot].direction == TRANSPORT_DIRECTION_TX &&
                    core->commands[slot].state ==
                        TRANSPORT_COMMAND_TX_WAIT_CTS) {
                core_exit(core, critical);
                return;
            }
        }
        for (slot = 0; slot < TRANSPORT_CORE_PIPE_SLOTS; ++slot) {
            if (core->pipes[slot].direction == TRANSPORT_DIRECTION_TX &&
                    core->pipes[slot].state ==
                        TRANSPORT_PIPE_TX_WAIT_CTS) {
                core_exit(core, critical);
                return;
            }
        }
        core_exit(core, critical);
        return;
    }
    if (kind == TRANSPORT_TX_COMMAND) {
        core->tx.destination_id = core->commands[slot].peer_id;
    } else if (kind == TRANSPORT_TX_PIPE) {
        core->tx.destination_id = core->pipes[slot].peer_id;
    } else {
        core->tx.destination_id = packet[2];
    }
    if (kind != TRANSPORT_TX_REGISTRATION) {
        address[0] = core->tx.destination_id;
        memcpy(address + 1, core->config.network_id,
            sizeof(core->config.network_id));
    }
    radio = core->config.radio.radio;
    nrf24_enter_tx_mode(radio);
    nrf24_open_tx_pipe(radio, address);
    core->tx.active = true;
    core->tx.kind = kind;
    core->tx.slot = slot;
    if (!core->tx_campaign_active) {
        core->tx_campaign_active = true;
        core->tx_campaign_started_ms = core_now_ms(core);
    }
    core->radio_phase = TRANSPORT_RADIO_TX_ACTIVE;
    if (kind == TRANSPORT_TX_CONTROL || kind == TRANSPORT_TX_REGISTRATION) {
        if (!nrf24_write_payload(radio, packet, sizeof(packet))) goto tx_failed;
        core->tx.last_packet = true;
        core->tx.staged_packets = 1;
    } else if (!core_fill_tx_owner(core)) {
tx_failed:
        nrf24_abort_send(radio);
        if (kind == TRANSPORT_TX_CONTROL) {
            core_control_failed(core, packet);
            (void)transport_ring_discard(&core->control_tx,
                TRANSPORT_CONTROL_RECORD_SIZE);
        } else if (kind == TRANSPORT_TX_REGISTRATION) {
            core_finish_registration_tx(core, false,
                TRANSPORT_REGISTRATION_FAILURE_RADIO);
        } else if (kind == TRANSPORT_TX_COMMAND) {
            core_finish_outbound_command(core, slot, false,
                TRANSPORT_COMMAND_FAILURE_RADIO);
        } else {
            core_finish_outbound_pipe(core, slot, false,
                TRANSPORT_PIPE_FAILURE_RADIO);
        }
        core_release_tx_owner(core);
        core_restore_rx(core);
        transport_core_report_error(core, TRANSPORT_ERROR_RADIO,
            TRANSPORT_CORE_INVALID_ID, 0x545851u);
        core_exit(core, critical);
        return;
    }
    memset(&core->retry, 0, sizeof(core->retry));
    core->retry.max_restarts = core->config.max_rt_restarts;
    nrf24_set_ce(radio, true);
    core_exit(core, critical);
}

static bool core_queue_packet(transport_core_t *core,
        uint8_t message_type, uint8_t destination_id, uint8_t message_id,
        uint8_t wire_flags, const uint8_t *payload, size_t payload_length) {
    uint8_t packet[NRF24_MAX_PAYLOAD];
    transport_critical_state_t critical;
    bool queued = false;
    if (core == NULL || !transport_ring_is_attached(&core->control_tx) ||
            !core_build_packet(core, message_type, destination_id, message_id,
                wire_flags, payload, payload_length, packet)) {
        return false;
    }
    critical = core_enter(core);
    if (transport_ring_writable(&core->control_tx) >= sizeof(packet) &&
            transport_ring_write(&core->control_tx, packet, sizeof(packet)) ==
                sizeof(packet)) {
        core->stats.control_packets_queued++;
        queued = true;
        if (core->tx.active && core->tx.kind != TRANSPORT_TX_CONTROL) {
            core->tx.drain_requested = true;
            core->radio_phase = TRANSPORT_RADIO_TX_DRAINING;
        }
    } else {
        core->sticky_errors |=
            (uint32_t)1u << TRANSPORT_ERROR_CONTROL_QUEUE_FULL;
    }
    core_exit(core, critical);
    return queued;
}

bool transport_core_queue_control(transport_core_t *core,
        uint8_t message_type, uint8_t destination_id, uint8_t message_id,
        const uint8_t *payload, size_t payload_length) {
    return core_queue_packet(core, message_type, destination_id, message_id,
        TRANSPORT_WIRE_LAST_PACKET, payload, payload_length);
}

static bool core_queue_cts(transport_core_t *core, uint8_t destination_id,
        uint8_t message_id, uint8_t original_type,
        transport_cts_result_t result, uint32_t value) {
    uint8_t payload[6];
    payload[0] = original_type;
    payload[1] = (uint8_t)result;
    core_write_le32(payload + 2, value);
    return transport_core_queue_control(core, TRANSPORT_WIRE_CTS,
        destination_id, message_id, payload, sizeof(payload));
}

static uint8_t queue_used(const transport_core_t *core) {
    return (uint8_t)(core->event_write - core->event_read);
}

static bool queue_descriptor(transport_core_t *core,
        const transport_event_descriptor_t *descriptor) {
    uint8_t write;
    bool queued = false;
    transport_critical_state_t critical = core_enter(core);
    if (queue_used(core) >= TRANSPORT_CORE_EVENT_QUEUE_SIZE) {
        core->stats.event_queue_overflows++;
        core->sticky_errors |= (uint32_t)1u << TRANSPORT_ERROR_EVENT_QUEUE_OVERFLOW;
    } else {
        write = core->event_write;
        core->events[write % TRANSPORT_CORE_EVENT_QUEUE_SIZE] = *descriptor;
        TRANSPORT_RING_MEMORY_BARRIER();
        core->event_write = (uint8_t)(write + 1u);
        core->stats.events_emitted++;
        queued = true;
    }
    core_exit(core, critical);
    return queued;
}

static bool core_is_registration_type(uint8_t message_type) {
    return message_type == TRANSPORT_WIRE_ENUM_HELLO ||
        message_type == TRANSPORT_WIRE_ENUM_ASSIGN ||
        message_type == TRANSPORT_WIRE_ENUM_CONFIRM;
}

static bool core_registration_shape_valid(uint8_t message_type,
        uint8_t source_id, uint8_t destination_id, size_t payload_length) {
    if (message_type == TRANSPORT_WIRE_ENUM_HELLO) {
        return source_id == TRANSPORT_CORE_INVALID_ID && destination_id == 0 &&
            payload_length == 8;
    }
    if (message_type == TRANSPORT_WIRE_ENUM_ASSIGN) {
        return source_id == 0 && destination_id == TRANSPORT_CORE_INVALID_ID &&
            payload_length == 9;
    }
    if (message_type == TRANSPORT_WIRE_ENUM_CONFIRM) {
        return source_id > 0 && source_id < 0xfe && destination_id == 0 &&
            payload_length == 9;
    }
    return false;
}

static void core_finish_registration_tx(transport_core_t *core, bool sent,
        uint32_t reason) {
    transport_event_descriptor_t descriptor;
    uint8_t *packet;
    if (core == NULL || !core->registration_tx.pending) return;
    packet = core->registration_tx.packet;
    memset(&descriptor, 0, sizeof(descriptor));
    descriptor.fields.type = sent ? TRANSPORT_EVENT_REGISTRATION_SENT :
        TRANSPORT_EVENT_REGISTRATION_FAILED;
    descriptor.fields.flags = TRANSPORT_EVENT_FLAG_TX;
    descriptor.fields.object_id = packet[0] & TRANSPORT_WIRE_TYPE_MASK;
    descriptor.fields.source_id = packet[2];
    descriptor.fields.transaction_id = packet[3];
    descriptor.fields.value0 = packet[2];
    descriptor.fields.value1 = sent ? 0 : reason;
    core->registration_tx.pending = false;
    if (sent) core->stats.registration_sent++;
    else core->stats.registration_failed++;
    (void)queue_descriptor(core, &descriptor);
}

static void core_receive_registration(transport_core_t *core,
        uint8_t message_type, uint8_t source_id, uint8_t destination_id,
        uint8_t message_id, const uint8_t *payload, size_t payload_length) {
    transport_event_descriptor_t descriptor;
    if (!core_registration_shape_valid(message_type, source_id,
            destination_id, payload_length)) {
        core->stats.protocol_errors++;
        return;
    }
    if ((message_type == TRANSPORT_WIRE_ENUM_HELLO ||
         message_type == TRANSPORT_WIRE_ENUM_CONFIRM) &&
            core->config.node_id != 0) {
        return;
    }
    if (message_type == TRANSPORT_WIRE_ENUM_ASSIGN &&
            core->config.node_id != TRANSPORT_CORE_INVALID_ID) {
        return;
    }
    memset(&descriptor, 0, sizeof(descriptor));
    descriptor.fields.type = TRANSPORT_EVENT_REGISTRATION_RECEIVED;
    descriptor.fields.object_id = message_type;
    descriptor.fields.source_id = source_id;
    descriptor.fields.transaction_id = message_id;
    descriptor.fields.data_length = (uint16_t)payload_length;
    descriptor.fields.value0 = destination_id;
    descriptor.payload_kind = TRANSPORT_EVENT_PAYLOAD_INLINE;
    memcpy(descriptor.inline_data, payload, payload_length);
    if (queue_descriptor(core, &descriptor)) core->stats.registration_rx++;
}

static void dequeue_descriptor(transport_core_t *core) {
    TRANSPORT_RING_MEMORY_BARRIER();
    core->event_read = (uint8_t)(core->event_read + 1u);
}

static bool queue_pipe_data_event(transport_core_t *core, uint8_t pipe_id) {
    transport_event_descriptor_t descriptor;
    transport_pipe_slot_t *pipe = &core->pipes[pipe_id];
    if (pipe->rx_event_pending) return true;
    memset(&descriptor, 0, sizeof(descriptor));
    descriptor.fields.type = TRANSPORT_EVENT_PIPE_RX_DATA;
    descriptor.fields.object_id = pipe_id;
    descriptor.fields.source_id = pipe->peer_id;
    descriptor.fields.transaction_id = pipe->session_id;
    descriptor.payload_kind = TRANSPORT_EVENT_PAYLOAD_PIPE_RX;
    descriptor.payload_index = pipe_id;
    if (!queue_descriptor(core, &descriptor)) return false;
    pipe->rx_event_pending = true;
    return true;
}

static bool queue_pipe_tx_space_event(transport_core_t *core,
        uint8_t pipe_id) {
    transport_event_fields_t event;
    transport_pipe_slot_t *pipe = &core->pipes[pipe_id];
    if (pipe->tx_space_event_pending) return true;
    memset(&event, 0, sizeof(event));
    event.type = TRANSPORT_EVENT_PIPE_TX_SPACE;
    event.flags = TRANSPORT_EVENT_FLAG_TX;
    event.object_id = pipe_id;
    event.source_id = pipe->peer_id;
    event.transaction_id = pipe->session_id;
    event.value0 = (uint32_t)transport_ring_writable(&pipe->buffer);
    event.value1 = pipe->granted_bytes - pipe->transferred_bytes;
    if (!transport_core_emit(core, &event)) return false;
    pipe->tx_space_event_pending = true;
    return true;
}

static void core_maybe_queue_rx_credit(transport_core_t *core,
        uint8_t pipe_id) {
    transport_pipe_slot_t *pipe = &core->pipes[pipe_id];
    uint32_t desired;
    if (pipe->direction != TRANSPORT_DIRECTION_RX ||
            pipe->state != TRANSPORT_PIPE_OPEN ||
            pipe->credit_update_pending || pipe->credit_wait_request ||
            pipe->rx_event_pending) {
        return;
    }
    /* Do not turn the half-duplex receiver around while the peer can still
       be transmitting against its current credit.  Replenish only after the
       granted window is exhausted and the sender has necessarily stopped. */
    if (pipe->transferred_bytes != pipe->granted_bytes) return;
    desired = pipe->transferred_bytes +
        (uint32_t)transport_ring_writable(&pipe->buffer);
    if (desired == pipe->granted_bytes) return;
    if (core_queue_cts(core, pipe->peer_id, (uint8_t)pipe->session_id,
            TRANSPORT_WIRE_STREAM, TRANSPORT_CTS_ACCEPTED, desired)) {
        /* Commit cumulative credit when the CTS is queued.  The peer may
           receive it even if the radio later reports MAX_RT because the
           hardware acknowledgement was lost.  A credit request makes the
           update idempotently repeatable in either case. */
        pipe->granted_bytes = desired;
        pipe->credit_update_pending = true;
    }
}

static void core_maybe_queue_tx_credit_request(transport_core_t *core,
        uint8_t pipe_id, uint32_t now) {
    transport_pipe_slot_t *pipe = &core->pipes[pipe_id];
    uint32_t used_credit;
    bool needs_credit;
    if (pipe->direction != TRANSPORT_DIRECTION_TX ||
            (pipe->state != TRANSPORT_PIPE_TX_OPEN &&
             pipe->state != TRANSPORT_PIPE_TX_CLOSING)) {
        return;
    }
    used_credit = pipe->transferred_bytes;
    if (core->tx.active && core->tx.kind == TRANSPORT_TX_PIPE &&
            core->tx.slot == pipe_id) {
        used_credit += core->tx.staged_payload_bytes;
    }
    needs_credit = transport_ring_readable(&pipe->buffer) != 0 ||
        pipe->state == TRANSPORT_PIPE_TX_CLOSING;
    if (!needs_credit || pipe->granted_bytes > used_credit) {
        if (!pipe->credit_request_queued) pipe->credit_request_at_ms = 0;
        return;
    }
    if (pipe->credit_request_queued) return;
    if (pipe->credit_request_at_ms == 0) {
        pipe->credit_request_at_ms = now +
            core_pipe_close_retry_delay(core);
        return;
    }
    if (!core_deadline_reached(now, pipe->credit_request_at_ms)) return;
    if (core_queue_packet(core, TRANSPORT_WIRE_PIPE_CREDIT_REQUEST,
            pipe->peer_id, (uint8_t)pipe->session_id,
            TRANSPORT_WIRE_LAST_PACKET, NULL, 0)) {
        pipe->credit_request_queued = true;
        pipe->credit_request_at_ms = 0;
    }
}

static bool core_is_command_type(uint8_t message_type) {
    return message_type == TRANSPORT_WIRE_COMMAND ||
        message_type == TRANSPORT_WIRE_COMMAND_REPLY ||
        message_type == TRANSPORT_WIRE_MANAGEMENT_REQUEST ||
        message_type == TRANSPORT_WIRE_MANAGEMENT_REPLY;
}

static int core_find_command_slot(transport_core_t *core, uint8_t message_type,
        uint8_t source_id, uint8_t message_id, bool allow_free) {
    int free_slot = -1;
    uint8_t i;
    for (i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        transport_command_slot_t *command = &core->commands[i];
        if (command->state == TRANSPORT_COMMAND_FREE) {
            if (free_slot < 0) free_slot = i;
        } else if (command->direction == TRANSPORT_DIRECTION_RX &&
                (command->state == TRANSPORT_COMMAND_CTS_SENT ||
                 command->state == TRANSPORT_COMMAND_RECEIVING) &&
                command->message_type == message_type &&
                command->peer_id == source_id &&
                command->transaction_id == message_id) {
            return i;
        }
    }
    return allow_free ? free_slot : -1;
}

static void core_discard_command(transport_command_slot_t *command) {
    (void)transport_ring_discard(&command->buffer,
        transport_ring_readable(&command->buffer));
    command->direction = TRANSPORT_DIRECTION_NONE;
    command->state = TRANSPORT_COMMAND_FREE;
    command->target_length = 0;
    command->transferred_length = 0;
}

static int core_find_outbound_command(transport_core_t *core,
        uint8_t message_type, uint8_t peer_id, uint8_t message_id) {
    uint8_t slot;
    for (slot = 0; slot < TRANSPORT_CORE_COMMAND_SLOTS; ++slot) {
        transport_command_slot_t *command = &core->commands[slot];
        if (command->direction == TRANSPORT_DIRECTION_TX &&
                command->message_type == message_type &&
                command->peer_id == peer_id &&
                command->transaction_id == message_id) {
            return slot;
        }
    }
    return -1;
}

static void core_finish_outbound_command(transport_core_t *core, uint8_t slot,
        bool sent, uint32_t value) {
    transport_event_fields_t event;
    transport_command_slot_t *command;
    if (core == NULL || slot >= TRANSPORT_CORE_COMMAND_SLOTS) return;
    command = &core->commands[slot];
    if (command->direction != TRANSPORT_DIRECTION_TX) return;
    memset(&event, 0, sizeof(event));
    event.type = sent ? TRANSPORT_EVENT_COMMAND_SENT :
        TRANSPORT_EVENT_COMMAND_FAILED;
    event.object_id = slot;
    event.source_id = command->peer_id;
    event.transaction_id = command->transaction_id;
    event.value0 = command->message_type;
    event.value1 = value;
    core_discard_command(command);
    if (sent) core->stats.commands_sent++;
    else core->stats.commands_failed++;
    (void)transport_core_emit(core, &event);
}

static bool core_outbound_command_busy(const transport_core_t *core) {
    uint8_t slot;
    for (slot = 0; slot < TRANSPORT_CORE_COMMAND_SLOTS; ++slot) {
        if (core->commands[slot].direction == TRANSPORT_DIRECTION_TX) {
            return true;
        }
    }
    return false;
}

static bool core_outbound_pipe_handshake_busy(const transport_core_t *core) {
    uint8_t slot;
    for (slot = 0; slot < TRANSPORT_CORE_PIPE_SLOTS; ++slot) {
        const transport_pipe_slot_t *pipe = &core->pipes[slot];
        if (pipe->direction == TRANSPORT_DIRECTION_TX &&
                (pipe->state == TRANSPORT_PIPE_TX_INTENT_QUEUED ||
                 pipe->state == TRANSPORT_PIPE_TX_WAIT_CTS)) {
            return true;
        }
    }
    return false;
}

static bool core_outbound_application_busy(const transport_core_t *core) {
    uint8_t slot;
    if (core_outbound_command_busy(core)) return true;
    for (slot = 0; slot < TRANSPORT_CORE_PIPE_SLOTS; ++slot) {
        if (core->pipes[slot].direction == TRANSPORT_DIRECTION_TX) return true;
    }
    return false;
}

static int core_find_outbound_pipe(transport_core_t *core, uint8_t peer_id,
        uint8_t stream_id) {
    uint8_t slot;
    for (slot = 0; slot < TRANSPORT_CORE_PIPE_SLOTS; ++slot) {
        transport_pipe_slot_t *pipe = &core->pipes[slot];
        if (pipe->direction == TRANSPORT_DIRECTION_TX &&
                pipe->peer_id == peer_id && pipe->session_id == stream_id) {
            return slot;
        }
    }
    return -1;
}

static void core_reset_pipe(transport_pipe_slot_t *pipe) {
    transport_ring_reset(&pipe->buffer);
    pipe->direction = TRANSPORT_DIRECTION_NONE;
    pipe->state = TRANSPORT_PIPE_FREE;
    pipe->peer_id = TRANSPORT_CORE_INVALID_ID;
    pipe->session_id = 0;
    pipe->granted_bytes = 0;
    pipe->transferred_bytes = 0;
    pipe->lease_deadline_ms = 0;
    pipe->close_retry_at_ms = 0;
    pipe->credit_request_at_ms = 0;
    pipe->rx_event_pending = false;
    pipe->tx_space_event_pending = false;
    pipe->credit_update_pending = false;
    pipe->close_event_pending = false;
    pipe->close_ack_pending = false;
    pipe->close_ack_queued = false;
    pipe->close_ack_wait_duplicate = false;
    pipe->credit_request_queued = false;
    pipe->credit_wait_request = false;
    pipe->terminal_event_polled = false;
    pipe->terminal_event_reason = 0;
}

static bool core_queue_pipe_close_ack(transport_core_t *core, uint8_t slot) {
    transport_pipe_slot_t *pipe;
    if (core == NULL || slot >= TRANSPORT_CORE_PIPE_SLOTS) return false;
    pipe = &core->pipes[slot];
    if (pipe->direction != TRANSPORT_DIRECTION_RX ||
            pipe->state != TRANSPORT_PIPE_CLOSED ||
            !pipe->close_ack_pending || pipe->close_ack_wait_duplicate) {
        return false;
    }
    if (pipe->close_ack_queued) return true;
    if (!core_queue_packet(core, TRANSPORT_WIRE_PIPE_CLOSE_ACK,
            pipe->peer_id, (uint8_t)pipe->session_id,
            TRANSPORT_WIRE_LAST_PACKET, NULL, 0)) {
        return false;
    }
    pipe->close_ack_queued = true;
    return true;
}

static int core_find_pipe_close_tombstone(transport_core_t *core,
        uint8_t peer_id, uint8_t session_id) {
    uint8_t i;
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        transport_pipe_close_tombstone_t *tombstone =
            &core->pipe_close_tombstones[i];
        if (tombstone->valid && tombstone->peer_id == peer_id &&
                tombstone->session_id == session_id) {
            return i;
        }
    }
    return -1;
}

static bool core_queue_tombstone_close_ack(transport_core_t *core,
        uint8_t index) {
    transport_pipe_close_tombstone_t *tombstone;
    if (core == NULL || index >= TRANSPORT_CORE_PIPE_SLOTS) return false;
    tombstone = &core->pipe_close_tombstones[index];
    if (!tombstone->valid || !tombstone->ack_pending ||
            tombstone->wait_duplicate) {
        return false;
    }
    if (tombstone->ack_queued) return true;
    if (!core_queue_packet(core, TRANSPORT_WIRE_PIPE_CLOSE_ACK,
            tombstone->peer_id, (uint8_t)tombstone->session_id,
            TRANSPORT_WIRE_LAST_PACKET, NULL, 0)) {
        return false;
    }
    tombstone->ack_queued = true;
    return true;
}

static void core_remember_closed_pipe(transport_core_t *core, uint8_t slot) {
    transport_pipe_slot_t *pipe;
    transport_pipe_close_tombstone_t *tombstone;
    uint8_t target = slot;
    uint8_t i;
    if (core == NULL || slot >= TRANSPORT_CORE_PIPE_SLOTS) return;
    pipe = &core->pipes[slot];
    if (pipe->direction != TRANSPORT_DIRECTION_RX ||
            pipe->state != TRANSPORT_PIPE_CLOSED) {
        return;
    }
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        tombstone = &core->pipe_close_tombstones[i];
        if (tombstone->valid && tombstone->peer_id == pipe->peer_id &&
                tombstone->session_id == pipe->session_id) {
            target = i;
            break;
        }
        if (!tombstone->valid) target = i;
    }
    tombstone = &core->pipe_close_tombstones[target];
    memset(tombstone, 0, sizeof(*tombstone));
    tombstone->valid = true;
    tombstone->peer_id = pipe->peer_id;
    tombstone->session_id = pipe->session_id;
    tombstone->deadline_ms = core->config.pipe_lease_ms != 0 ?
        core_now_ms(core) + core->config.pipe_lease_ms : 0;
}

static void core_finish_outbound_pipe(transport_core_t *core, uint8_t slot,
        bool closed, uint32_t reason) {
    transport_pipe_slot_t *pipe;
    if (core == NULL || slot >= TRANSPORT_CORE_PIPE_SLOTS) return;
    pipe = &core->pipes[slot];
    if (pipe->direction != TRANSPORT_DIRECTION_TX) return;
    pipe->state = closed ? TRANSPORT_PIPE_CLOSED : TRANSPORT_PIPE_FAILED;
    pipe->tx_space_event_pending = false;
    pipe->close_event_pending = true;
    pipe->terminal_event_reason = reason;
    if (closed) core->stats.pipes_closed++;
    else core->stats.pipes_failed++;
    (void)core_emit_pipe_terminal(core, slot);
}

static bool core_emit_pipe_terminal(transport_core_t *core, uint8_t slot) {
    transport_event_fields_t event;
    transport_pipe_slot_t *pipe;
    if (core == NULL || slot >= TRANSPORT_CORE_PIPE_SLOTS) return false;
    pipe = &core->pipes[slot];
    if ((pipe->direction != TRANSPORT_DIRECTION_RX &&
            pipe->direction != TRANSPORT_DIRECTION_TX) ||
            (pipe->state != TRANSPORT_PIPE_CLOSED &&
             pipe->state != TRANSPORT_PIPE_FAILED) ||
            !pipe->close_event_pending) return false;
    memset(&event, 0, sizeof(event));
    event.type = pipe->state == TRANSPORT_PIPE_CLOSED ?
        TRANSPORT_EVENT_PIPE_CLOSED : TRANSPORT_EVENT_PIPE_FAILED;
    if (pipe->direction == TRANSPORT_DIRECTION_TX)
        event.flags = TRANSPORT_EVENT_FLAG_TX;
    event.object_id = slot;
    event.source_id = pipe->peer_id;
    event.transaction_id = pipe->session_id;
    event.value0 = pipe->state == TRANSPORT_PIPE_CLOSED ?
        pipe->transferred_bytes : pipe->terminal_event_reason;
    event.value1 = pipe->state == TRANSPORT_PIPE_FAILED ?
        pipe->transferred_bytes : 0;
    if (!transport_core_emit(core, &event)) return false;
    pipe->close_event_pending = false;
    return true;
}

bool transport_core_send_command(transport_core_t *core,
        uint8_t destination_id, uint8_t message_type, uint8_t message_id,
        const uint8_t *data, size_t length) {
    transport_critical_state_t critical;
    transport_command_slot_t *command = NULL;
    uint8_t intent[2];
    uint8_t slot;
    if (core == NULL || !core->started ||
            destination_id == TRANSPORT_CORE_INVALID_ID ||
            destination_id == core->config.node_id ||
            !core_is_command_type(message_type) || length > UINT16_MAX ||
            (data == NULL && length != 0)) {
        return false;
    }
    critical = core_enter(core);
    if (core_outbound_command_busy(core) ||
            core_outbound_pipe_handshake_busy(core)) {
        core_exit(core, critical);
        return false;
    }
    for (slot = 0; slot < TRANSPORT_CORE_COMMAND_SLOTS; ++slot) {
        if (core->commands[slot].state == TRANSPORT_COMMAND_FREE &&
                core->commands[slot].buffer.capacity >= length) {
            command = &core->commands[slot];
            command->direction = TRANSPORT_DIRECTION_TX;
            command->state = TRANSPORT_COMMAND_TX_LOADING;
            command->peer_id = destination_id;
            command->message_type = message_type;
            command->transaction_id = message_id;
            command->target_length = (uint16_t)length;
            command->transferred_length = 0;
            break;
        }
    }
    core_exit(core, critical);
    if (command == NULL) return false;
    if (length != 0 &&
            transport_ring_write(&command->buffer, data, length) != length) {
        core_discard_command(command);
        return false;
    }
    command->state = TRANSPORT_COMMAND_TX_INTENT_QUEUED;
    core_write_le16(intent, (uint16_t)length);
    if (!core_queue_packet(core, message_type, destination_id, message_id,
            TRANSPORT_WIRE_INTENT | TRANSPORT_WIRE_LAST_PACKET,
            intent, sizeof(intent))) {
        core_discard_command(command);
        return false;
    }
    core_tx_kick(core);
    return true;
}

bool transport_core_send_registration(transport_core_t *core,
        uint8_t message_type, uint8_t destination_id, uint8_t message_id,
        const uint8_t address[NRF24_ADDR_LEN], const uint8_t *data,
        size_t length) {
    uint8_t packet[NRF24_MAX_PAYLOAD];
    transport_critical_state_t critical;
    if (core == NULL || !core->started || address == NULL ||
            (data == NULL && length != 0) ||
            !core_is_registration_type(message_type) ||
            !core_registration_shape_valid(message_type,
                core->config.node_id, destination_id, length) ||
            !core_build_packet(core, message_type, destination_id, message_id,
                TRANSPORT_WIRE_LAST_PACKET, data, length, packet)) {
        return false;
    }
    critical = core_enter(core);
    if (core->registration_tx.pending) {
        core_exit(core, critical);
        return false;
    }
    memcpy(core->registration_tx.address, address, NRF24_ADDR_LEN);
    memcpy(core->registration_tx.packet, packet, sizeof(packet));
    core->registration_tx.pending = true;
    if (core->tx.active && (core->tx.kind == TRANSPORT_TX_COMMAND ||
            core->tx.kind == TRANSPORT_TX_PIPE)) {
        core->tx.drain_requested = true;
        core->radio_phase = TRANSPORT_RADIO_TX_DRAINING;
    }
    core_exit(core, critical);
    core_tx_kick(core);
    return true;
}

int transport_core_open_pipe(transport_core_t *core, uint8_t destination_id,
        uint8_t stream_id) {
    transport_critical_state_t critical;
    transport_pipe_slot_t *pipe = NULL;
    uint8_t slot;
    if (core == NULL || !core->started ||
            destination_id == TRANSPORT_CORE_INVALID_ID ||
            destination_id == core->config.node_id) {
        return -1;
    }
    critical = core_enter(core);
    if (core_outbound_application_busy(core)) {
        core_exit(core, critical);
        return -1;
    }
    for (slot = 0; slot < TRANSPORT_CORE_PIPE_SLOTS; ++slot) {
        if (core->pipes[slot].state == TRANSPORT_PIPE_FREE) {
            pipe = &core->pipes[slot];
            transport_ring_reset(&pipe->buffer);
            pipe->peer_id = destination_id;
            pipe->session_id = stream_id;
            pipe->granted_bytes = 0;
            pipe->transferred_bytes = 0;
            pipe->direction = TRANSPORT_DIRECTION_TX;
            pipe->state = TRANSPORT_PIPE_TX_INTENT_QUEUED;
            pipe->rx_event_pending = false;
            pipe->tx_space_event_pending = false;
            pipe->credit_update_pending = false;
            pipe->close_event_pending = false;
            pipe->close_ack_pending = false;
            pipe->close_ack_queued = false;
            pipe->close_ack_wait_duplicate = false;
            pipe->terminal_event_polled = false;
            pipe->close_retry_at_ms = 0;
            pipe->credit_request_at_ms = 0;
            pipe->credit_request_queued = false;
            pipe->credit_wait_request = false;
            pipe->terminal_event_reason = 0;
            break;
        }
    }
    core_exit(core, critical);
    if (pipe == NULL) return -1;
    if (!core_queue_packet(core, TRANSPORT_WIRE_STREAM, destination_id,
            stream_id, TRANSPORT_WIRE_INTENT | TRANSPORT_WIRE_LAST_PACKET,
            NULL, 0)) {
        core_reset_pipe(pipe);
        return -1;
    }
    core_tx_kick(core);
    return slot;
}

bool transport_core_close_pipe(transport_core_t *core, uint8_t slot) {
    transport_critical_state_t critical;
    transport_pipe_slot_t *pipe;
    bool accepted = false;
    if (core == NULL || slot >= TRANSPORT_CORE_PIPE_SLOTS) return false;
    critical = core_enter(core);
    pipe = &core->pipes[slot];
    if (pipe->direction == TRANSPORT_DIRECTION_TX &&
            pipe->state == TRANSPORT_PIPE_TX_OPEN) {
        pipe->state = TRANSPORT_PIPE_TX_CLOSING;
        pipe->lease_deadline_ms = core_now_ms(core) + core->config.pipe_lease_ms;
        accepted = true;
    } else if (pipe->direction == TRANSPORT_DIRECTION_TX &&
            pipe->state == TRANSPORT_PIPE_TX_CLOSING) {
        accepted = true;
    }
    core_exit(core, critical);
    if (accepted) core_tx_kick(core);
    return accepted;
}

static void core_receive_command_intent(transport_core_t *core,
        uint8_t message_type, uint8_t source_id, uint8_t message_id,
        const uint8_t *payload, size_t payload_length) {
    int slot;
    uint16_t target_length;
    transport_command_slot_t *command;
    transport_cts_result_t result;
    if (payload_length != 2) {
        (void)core_queue_cts(core, source_id, message_id, message_type,
            TRANSPORT_CTS_INVALID, 0);
        return;
    }
    target_length = core_read_le16(payload);
    slot = core_find_command_slot(core, message_type, source_id, message_id,
        false);
    if (slot >= 0) {
        command = &core->commands[slot];
        result = command->target_length == target_length ?
            TRANSPORT_CTS_ACCEPTED : TRANSPORT_CTS_BUSY;
        (void)core_queue_cts(core, source_id, message_id, message_type, result,
            command->target_length);
        return;
    }
    slot = core_find_command_slot(core, message_type, source_id, message_id,
        true);
    if (slot < 0) {
        (void)core_queue_cts(core, source_id, message_id, message_type,
            TRANSPORT_CTS_NO_BUFFER, 0);
        return;
    }
    command = &core->commands[slot];
    if (target_length > command->buffer.capacity) {
        (void)core_queue_cts(core, source_id, message_id, message_type,
            TRANSPORT_CTS_TOO_LARGE, (uint32_t)command->buffer.capacity);
        return;
    }
    if (transport_ring_readable(&command->buffer) != 0) {
        (void)core_queue_cts(core, source_id, message_id, message_type,
            TRANSPORT_CTS_BUSY, 0);
        return;
    }
    command->peer_id = source_id;
    command->message_type = message_type;
    command->transaction_id = message_id;
    command->target_length = target_length;
    command->direction = TRANSPORT_DIRECTION_RX;
    command->state = TRANSPORT_COMMAND_INTENT_RECEIVED;
    command->lease_deadline_ms = core_now_ms(core) +
        core->config.command_lease_ms;
    if (!core_queue_cts(core, source_id, message_id, message_type,
            TRANSPORT_CTS_ACCEPTED, target_length)) {
        core_discard_command(command);
        transport_core_report_error(core, TRANSPORT_ERROR_CONTROL_QUEUE_FULL,
            (uint8_t)slot, message_id);
        return;
    }
    command->state = TRANSPORT_COMMAND_CTS_SENT;
}

static void core_receive_command_fragment(transport_core_t *core,
        uint8_t message_type, uint8_t source_id, uint8_t message_id,
        const uint8_t *payload, size_t payload_length, bool last_packet) {
    int slot = core_find_command_slot(core, message_type, source_id, message_id,
        false);
    transport_command_slot_t *command;
    size_t new_length;
    if (slot < 0) {
        transport_core_report_error(core, TRANSPORT_ERROR_PROTOCOL,
            TRANSPORT_CORE_INVALID_ID, message_id);
        return;
    }
    command = &core->commands[slot];
    new_length = transport_ring_readable(&command->buffer) + payload_length;
    if (new_length > command->target_length ||
            new_length > command->buffer.capacity) {
        transport_core_report_error(core, TRANSPORT_ERROR_COMMAND_TOO_LARGE,
            (uint8_t)slot, (uint32_t)new_length);
        (void)core_queue_cts(core, source_id, message_id, message_type,
            TRANSPORT_CTS_INVALID, (uint32_t)new_length);
        core_discard_command(command);
        return;
    }
    if (payload_length != 0 && transport_ring_write(&command->buffer, payload,
            payload_length) != payload_length) {
        transport_core_report_error(core, TRANSPORT_ERROR_RX_OVERRUN,
            (uint8_t)slot, (uint32_t)payload_length);
        core_discard_command(command);
        return;
    }
    command->state = TRANSPORT_COMMAND_RECEIVING;
    command->lease_deadline_ms = core_now_ms(core) +
        core->config.command_lease_ms;
    if (last_packet) {
        if (new_length != command->target_length) {
            transport_core_report_error(core, TRANSPORT_ERROR_PROTOCOL,
                (uint8_t)slot, (uint32_t)new_length);
            (void)core_queue_cts(core, source_id, message_id, message_type,
                TRANSPORT_CTS_INVALID, (uint32_t)new_length);
            core_discard_command(command);
            return;
        }
        (void)transport_core_commit_command(core, (uint8_t)slot, source_id,
            message_id, new_length);
    }
}

static int core_find_pipe_slot(transport_core_t *core, uint8_t source_id,
        uint8_t stream_id, bool allow_free) {
    int free_slot = -1;
    uint8_t i;
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        transport_pipe_slot_t *pipe = &core->pipes[i];
        if (pipe->direction == TRANSPORT_DIRECTION_RX &&
                (pipe->state == TRANSPORT_PIPE_CTS_SENT ||
                 pipe->state == TRANSPORT_PIPE_OPEN ||
                 pipe->state == TRANSPORT_PIPE_CLOSED) &&
                pipe->peer_id == source_id &&
                pipe->session_id == stream_id) {
            return i;
        }
        if (free_slot < 0 && pipe->state == TRANSPORT_PIPE_FREE) {
            free_slot = i;
        }
    }
    return allow_free ? free_slot : -1;
}

static void core_receive_pipe_intent(transport_core_t *core,
        uint8_t source_id, uint8_t stream_id, size_t payload_length) {
    int slot;
    int tombstone;
    transport_pipe_slot_t *pipe;
    if (payload_length != 0) {
        (void)core_queue_cts(core, source_id, stream_id, TRANSPORT_WIRE_STREAM,
            TRANSPORT_CTS_INVALID, 0);
        return;
    }
    tombstone = core_find_pipe_close_tombstone(core, source_id, stream_id);
    if (tombstone >= 0) {
        memset(&core->pipe_close_tombstones[tombstone], 0,
            sizeof(core->pipe_close_tombstones[tombstone]));
    }
    slot = core_find_pipe_slot(core, source_id, stream_id, false);
    if (slot >= 0) {
        pipe = &core->pipes[slot];
        if (pipe->state == TRANSPORT_PIPE_CLOSED) {
            pipe->close_ack_pending = true;
            (void)core_queue_pipe_close_ack(core, (uint8_t)slot);
            return;
        }
        (void)core_queue_cts(core, source_id, stream_id, TRANSPORT_WIRE_STREAM,
            TRANSPORT_CTS_ACCEPTED, pipe->granted_bytes);
        return;
    }
    slot = core_find_pipe_slot(core, source_id, stream_id, true);
    if (slot < 0) {
        (void)core_queue_cts(core, source_id, stream_id, TRANSPORT_WIRE_STREAM,
            TRANSPORT_CTS_NO_BUFFER, 0);
        return;
    }
    pipe = &core->pipes[slot];
    if (transport_ring_readable(&pipe->buffer) != 0) {
        (void)core_queue_cts(core, source_id, stream_id, TRANSPORT_WIRE_STREAM,
            TRANSPORT_CTS_BUSY, 0);
        return;
    }
    pipe->peer_id = source_id;
    pipe->session_id = stream_id;
    pipe->direction = TRANSPORT_DIRECTION_RX;
    pipe->state = TRANSPORT_PIPE_OPEN_INTENT_RECEIVED;
    pipe->granted_bytes = (uint32_t)pipe->buffer.capacity;
    pipe->transferred_bytes = 0;
    pipe->rx_event_pending = false;
    pipe->tx_space_event_pending = false;
    pipe->credit_update_pending = false;
    pipe->close_event_pending = false;
    pipe->close_ack_pending = false;
    pipe->close_ack_queued = false;
    pipe->close_ack_wait_duplicate = false;
    pipe->terminal_event_polled = false;
    pipe->close_retry_at_ms = 0;
    pipe->credit_request_at_ms = 0;
    pipe->credit_request_queued = false;
    pipe->credit_wait_request = false;
    pipe->terminal_event_reason = 0;
    pipe->lease_deadline_ms = core_now_ms(core) + core->config.pipe_lease_ms;
    if (!core_queue_cts(core, source_id, stream_id, TRANSPORT_WIRE_STREAM,
            TRANSPORT_CTS_ACCEPTED, pipe->granted_bytes)) {
        core_reset_pipe(pipe);
        transport_core_report_error(core, TRANSPORT_ERROR_CONTROL_QUEUE_FULL,
            (uint8_t)slot, stream_id);
        return;
    }
    pipe->state = TRANSPORT_PIPE_CTS_SENT;
}

static void core_control_acked(transport_core_t *core, const uint8_t *packet) {
    uint8_t packet_type, original_type, result, peer_id, message_id;
    int slot;
    packet_type = packet[0] & TRANSPORT_WIRE_TYPE_MASK;
    peer_id = packet[2];
    message_id = packet[3];
    if (packet_type == TRANSPORT_WIRE_PIPE_CREDIT_REQUEST) {
        slot = core_find_outbound_pipe(core, peer_id, message_id);
        if (slot >= 0) {
            core->pipes[slot].credit_request_queued = false;
            core->pipes[slot].credit_request_at_ms = core_now_ms(core) +
                core_pipe_close_retry_delay(core);
        }
        return;
    }
    if (packet_type == TRANSPORT_WIRE_PIPE_CLOSE_ACK) {
        slot = core_find_pipe_slot(core, peer_id, message_id, false);
        if (slot >= 0 && core->pipes[slot].direction == TRANSPORT_DIRECTION_RX &&
                core->pipes[slot].state == TRANSPORT_PIPE_CLOSED) {
            core->pipes[slot].close_ack_pending = false;
            core->pipes[slot].close_ack_queued = false;
            core->pipes[slot].close_ack_wait_duplicate = false;
            if (core->pipes[slot].terminal_event_polled) {
                core_remember_closed_pipe(core, (uint8_t)slot);
                core_reset_pipe(&core->pipes[slot]);
            }
        } else {
            slot = core_find_pipe_close_tombstone(core, peer_id, message_id);
            if (slot >= 0) {
                core->pipe_close_tombstones[slot].ack_pending = false;
                core->pipe_close_tombstones[slot].ack_queued = false;
                core->pipe_close_tombstones[slot].wait_duplicate = false;
            }
        }
        return;
    }
    if (core_is_command_type(packet_type) &&
            (packet[4] & TRANSPORT_WIRE_INTENT) != 0) {
        slot = core_find_outbound_command(core, packet_type, peer_id,
            message_id);
        if (slot >= 0 && core->commands[slot].state ==
                TRANSPORT_COMMAND_TX_INTENT_QUEUED) {
            core->commands[slot].state = TRANSPORT_COMMAND_TX_WAIT_CTS;
            core->commands[slot].lease_deadline_ms = core_now_ms(core) +
                core->config.command_lease_ms;
        }
        return;
    }
    if (packet_type == TRANSPORT_WIRE_STREAM &&
            (packet[4] & TRANSPORT_WIRE_INTENT) != 0) {
        slot = core_find_outbound_pipe(core, peer_id, message_id);
        if (slot >= 0 && core->pipes[slot].state ==
                TRANSPORT_PIPE_TX_INTENT_QUEUED) {
            core->pipes[slot].state = TRANSPORT_PIPE_TX_WAIT_CTS;
            core->pipes[slot].lease_deadline_ms = core_now_ms(core) +
                core->config.pipe_lease_ms;
        }
        return;
    }
    if (packet_type != TRANSPORT_WIRE_CTS ||
            (packet[4] & TRANSPORT_WIRE_LENGTH_MASK) < 2) {
        return;
    }
    original_type = packet[TRANSPORT_WIRE_HEADER_SIZE];
    result = packet[TRANSPORT_WIRE_HEADER_SIZE + 1];
    if (result != TRANSPORT_CTS_ACCEPTED) return;
    if (core_is_command_type(original_type)) {
        slot = core_find_command_slot(core, original_type, peer_id,
            message_id, false);
        if (slot >= 0 &&
                core->commands[slot].state == TRANSPORT_COMMAND_CTS_SENT) {
            core->commands[slot].state = TRANSPORT_COMMAND_RECEIVING;
        }
    } else if (original_type == TRANSPORT_WIRE_STREAM) {
        transport_event_fields_t event;
        slot = core_find_pipe_slot(core, peer_id, message_id, false);
        if (slot < 0) return;
        if (core->pipes[slot].state == TRANSPORT_PIPE_OPEN &&
                core->pipes[slot].credit_update_pending) {
            core->pipes[slot].granted_bytes =
                core_read_le32(packet + TRANSPORT_WIRE_HEADER_SIZE + 2);
            core->pipes[slot].credit_update_pending = false;
            core->pipes[slot].credit_wait_request = false;
            return;
        }
        if (core->pipes[slot].state != TRANSPORT_PIPE_CTS_SENT) return;
        core->pipes[slot].state = TRANSPORT_PIPE_OPEN;
        core->stats.pipes_opened++;
        memset(&event, 0, sizeof(event));
        event.type = TRANSPORT_EVENT_PIPE_OPENED;
        event.object_id = (uint8_t)slot;
        event.source_id = peer_id;
        event.transaction_id = message_id;
        if (!transport_core_emit(core, &event)) {
            core_reset_pipe(&core->pipes[slot]);
        }
    }
}

static void core_control_failed(transport_core_t *core,
        const uint8_t *packet) {
    uint8_t packet_type, original_type, result, peer_id, message_id;
    int slot;
    packet_type = packet[0] & TRANSPORT_WIRE_TYPE_MASK;
    peer_id = packet[2];
    message_id = packet[3];
    if (packet_type == TRANSPORT_WIRE_PIPE_CREDIT_REQUEST) {
        slot = core_find_outbound_pipe(core, peer_id, message_id);
        if (slot >= 0) {
            core->pipes[slot].credit_request_queued = false;
            core->pipes[slot].credit_request_at_ms = core_now_ms(core) +
                core_pipe_close_retry_delay(core);
        }
        return;
    }
    if (packet_type == TRANSPORT_WIRE_PIPE_CLOSE_ACK) {
        slot = core_find_pipe_slot(core, peer_id, message_id, false);
        if (slot >= 0 && core->pipes[slot].direction == TRANSPORT_DIRECTION_RX &&
                core->pipes[slot].state == TRANSPORT_PIPE_CLOSED) {
            core->pipes[slot].close_ack_queued = false;
            core->pipes[slot].close_ack_wait_duplicate = true;
        } else {
            slot = core_find_pipe_close_tombstone(core, peer_id, message_id);
            if (slot >= 0) {
                core->pipe_close_tombstones[slot].ack_queued = false;
                core->pipe_close_tombstones[slot].wait_duplicate = true;
            }
        }
        return;
    }
    if (core_is_command_type(packet_type) &&
            (packet[4] & TRANSPORT_WIRE_INTENT) != 0) {
        slot = core_find_outbound_command(core, packet_type, peer_id,
            message_id);
        if (slot >= 0) {
            core_finish_outbound_command(core, (uint8_t)slot, false,
                TRANSPORT_COMMAND_FAILURE_RADIO);
        }
        return;
    }
    if (packet_type == TRANSPORT_WIRE_STREAM &&
            (packet[4] & TRANSPORT_WIRE_INTENT) != 0) {
        slot = core_find_outbound_pipe(core, peer_id, message_id);
        if (slot >= 0) {
            core_finish_outbound_pipe(core, (uint8_t)slot, false,
                TRANSPORT_PIPE_FAILURE_RADIO);
        }
        return;
    }
    if (packet_type != TRANSPORT_WIRE_CTS ||
            (packet[4] & TRANSPORT_WIRE_LENGTH_MASK) < 2) {
        return;
    }
    original_type = packet[TRANSPORT_WIRE_HEADER_SIZE];
    result = packet[TRANSPORT_WIRE_HEADER_SIZE + 1];
    if (result != TRANSPORT_CTS_ACCEPTED) return;
    if (core_is_command_type(original_type)) {
        slot = core_find_command_slot(core, original_type, peer_id,
            message_id, false);
        if (slot >= 0) core_discard_command(&core->commands[slot]);
    } else if (original_type == TRANSPORT_WIRE_STREAM) {
        slot = core_find_pipe_slot(core, peer_id, message_id, false);
        if (slot >= 0) {
            if (core->pipes[slot].state == TRANSPORT_PIPE_OPEN) {
                core->pipes[slot].credit_update_pending = false;
                core->pipes[slot].credit_wait_request = true;
            } else {
                core_reset_pipe(&core->pipes[slot]);
            }
        }
    }
}

static void core_receive_cts(transport_core_t *core, uint8_t source_id,
        uint8_t message_id, const uint8_t *payload, size_t payload_length,
        bool last_packet) {
    uint8_t original_type, result;
    uint32_t value;
    int slot;
    if (!last_packet || payload_length != 6) {
        core->stats.protocol_errors++;
        return;
    }
    original_type = payload[0];
    result = payload[1];
    value = core_read_le32(payload + 2);
    if (original_type == TRANSPORT_WIRE_STREAM) {
        uint32_t delta;
        transport_pipe_slot_t *pipe;
        transport_event_fields_t event;
        slot = core_find_outbound_pipe(core, source_id, message_id);
        if (slot < 0) return;
        pipe = &core->pipes[slot];
        if (pipe->state == TRANSPORT_PIPE_TX_WAIT_CTS) {
            if (result != TRANSPORT_CTS_ACCEPTED || value == 0) {
                if (result == TRANSPORT_CTS_ACCEPTED ||
                        result > TRANSPORT_CTS_INVALID) {
                    result = TRANSPORT_CTS_INVALID;
                }
                core_finish_outbound_pipe(core, (uint8_t)slot, false,
                    TRANSPORT_PIPE_FAILURE_CTS_BASE + result);
                return;
            }
            pipe->granted_bytes = value;
            pipe->transferred_bytes = 0;
            pipe->credit_request_at_ms = 0;
            pipe->credit_request_queued = false;
            pipe->state = TRANSPORT_PIPE_TX_OPEN;
            core->application_tx_not_before_ms = core_now_ms(core) +
                TRANSPORT_CORE_CTS_TURNAROUND_MS;
            pipe->lease_deadline_ms = core->config.pipe_lease_ms != 0 ?
                core_now_ms(core) + core->config.pipe_lease_ms : 0;
            memset(&event, 0, sizeof(event));
            event.type = TRANSPORT_EVENT_PIPE_OPENED;
            event.flags = TRANSPORT_EVENT_FLAG_TX;
            event.object_id = (uint8_t)slot;
            event.source_id = source_id;
            event.transaction_id = message_id;
            event.value0 = (uint32_t)pipe->buffer.capacity;
            event.value1 = value;
            core->stats.pipes_opened++;
            if (!transport_core_emit(core, &event)) core_reset_pipe(pipe);
            return;
        }
        if (result != TRANSPORT_CTS_ACCEPTED ||
                (pipe->state != TRANSPORT_PIPE_TX_OPEN &&
                 pipe->state != TRANSPORT_PIPE_TX_CLOSING)) {
            return;
        }
        delta = value - pipe->granted_bytes;
        if (delta != 0 && delta < 0x80000000u &&
                delta <= pipe->buffer.capacity) {
            pipe->granted_bytes = value;
            pipe->credit_request_at_ms = 0;
            pipe->credit_request_queued = false;
            core->application_tx_not_before_ms = core_now_ms(core) +
                TRANSPORT_CORE_CTS_TURNAROUND_MS;
            if (core->config.pipe_lease_ms != 0) {
                pipe->lease_deadline_ms = core_now_ms(core) +
                    core->config.pipe_lease_ms;
            }
        } else if (delta == 0) {
            pipe->credit_request_at_ms = core_now_ms(core) +
                core_pipe_close_retry_delay(core);
            pipe->credit_request_queued = false;
            core->application_tx_not_before_ms = core_now_ms(core) +
                TRANSPORT_CORE_CTS_TURNAROUND_MS;
        }
        return;
    }
    if (!core_is_command_type(original_type)) return;
    slot = core_find_outbound_command(core, original_type, source_id,
        message_id);
    if (slot < 0 || core->commands[slot].state !=
            TRANSPORT_COMMAND_TX_WAIT_CTS) {
        return;
    }
    if (result == TRANSPORT_CTS_ACCEPTED &&
            value == core->commands[slot].target_length) {
        core->commands[slot].state = TRANSPORT_COMMAND_TX_SENDING;
        core->application_tx_not_before_ms = core_now_ms(core) +
            TRANSPORT_CORE_CTS_TURNAROUND_MS;
        core->commands[slot].lease_deadline_ms = core_now_ms(core) +
            core->config.command_lease_ms;
        return;
    }
    if (result == TRANSPORT_CTS_ACCEPTED) result = TRANSPORT_CTS_INVALID;
    if (result > TRANSPORT_CTS_INVALID) result = TRANSPORT_CTS_INVALID;
    core_finish_outbound_command(core, (uint8_t)slot, false,
        TRANSPORT_COMMAND_FAILURE_CTS_BASE + result);
}

static void core_receive_pipe_fragment(transport_core_t *core,
        uint8_t source_id, uint8_t stream_id, const uint8_t *payload,
        size_t payload_length, bool last_packet) {
    int slot = core_find_pipe_slot(core, source_id, stream_id, false);
    transport_pipe_slot_t *pipe;
    if (slot < 0) {
        int tombstone = core_find_pipe_close_tombstone(core, source_id,
            stream_id);
        if (tombstone >= 0 && last_packet && payload_length == 0) {
            transport_pipe_close_tombstone_t *closed =
                &core->pipe_close_tombstones[tombstone];
            closed->ack_pending = true;
            closed->wait_duplicate = false;
            closed->deadline_ms = core->config.pipe_lease_ms != 0 ?
                core_now_ms(core) + core->config.pipe_lease_ms : 0;
            (void)core_queue_tombstone_close_ack(core, (uint8_t)tombstone);
            return;
        }
        transport_core_report_error(core, TRANSPORT_ERROR_PROTOCOL,
            TRANSPORT_CORE_INVALID_ID, stream_id);
        return;
    }
    pipe = &core->pipes[slot];
    if (pipe->state == TRANSPORT_PIPE_CLOSED) {
        if (last_packet && payload_length == 0) {
            pipe->close_ack_pending = true;
            pipe->close_ack_wait_duplicate = false;
            pipe->lease_deadline_ms = core->config.pipe_lease_ms != 0 ?
                core_now_ms(core) + core->config.pipe_lease_ms : 0;
            (void)core_queue_pipe_close_ack(core, (uint8_t)slot);
        } else {
            transport_core_report_error(core, TRANSPORT_ERROR_PROTOCOL,
                (uint8_t)slot, stream_id);
        }
        return;
    }
    pipe->state = TRANSPORT_PIPE_OPEN;
    pipe->credit_wait_request = false;
    pipe->lease_deadline_ms = core_now_ms(core) + core->config.pipe_lease_ms;
    if ((uint32_t)payload_length >
            pipe->granted_bytes - pipe->transferred_bytes) {
        pipe->state = TRANSPORT_PIPE_FAILED;
        pipe->close_event_pending = true;
        pipe->terminal_event_reason = TRANSPORT_ERROR_RX_OVERRUN;
        core->stats.pipes_failed++;
        (void)core_emit_pipe_terminal(core, (uint8_t)slot);
        transport_core_report_error(core, TRANSPORT_ERROR_RX_OVERRUN,
            (uint8_t)slot, (uint32_t)payload_length);
        return;
    }
    if (payload_length != 0 && transport_core_pipe_rx_write(core,
            (uint8_t)slot, payload, payload_length) != payload_length) {
        pipe->state = TRANSPORT_PIPE_FAILED;
        pipe->close_event_pending = true;
        pipe->terminal_event_reason = TRANSPORT_ERROR_RX_OVERRUN;
        core->stats.pipes_failed++;
        (void)core_emit_pipe_terminal(core, (uint8_t)slot);
        return;
    }
    if (pipe->state == TRANSPORT_PIPE_FAILED) {
        pipe->close_event_pending = true;
        pipe->terminal_event_reason = TRANSPORT_ERROR_RX_OVERRUN;
        core->stats.pipes_failed++;
        (void)core_emit_pipe_terminal(core, (uint8_t)slot);
        return;
    }
    if (last_packet) {
        pipe->state = TRANSPORT_PIPE_CLOSED;
        pipe->close_ack_pending = true;
        pipe->close_ack_queued = false;
        pipe->close_ack_wait_duplicate = false;
        pipe->terminal_event_polled = false;
        pipe->close_event_pending = true;
        pipe->terminal_event_reason = 0;
        core->stats.pipes_closed++;
        (void)core_queue_pipe_close_ack(core, (uint8_t)slot);
        (void)core_emit_pipe_terminal(core, (uint8_t)slot);
    }
}

static void core_receive_pipe_close_ack(transport_core_t *core,
        uint8_t source_id, uint8_t stream_id, size_t payload_length,
        bool last_packet) {
    int slot;
    transport_pipe_slot_t *pipe;
    if (!last_packet || payload_length != 0) {
        core->stats.protocol_errors++;
        return;
    }
    slot = core_find_outbound_pipe(core, source_id, stream_id);
    if (slot < 0) return;
    pipe = &core->pipes[slot];
    if (pipe->state != TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK) return;
    core_finish_outbound_pipe(core, (uint8_t)slot, true, 0);
}

static void core_receive_pipe_credit_request(transport_core_t *core,
        uint8_t source_id, uint8_t stream_id, size_t payload_length,
        bool last_packet) {
    int slot;
    transport_pipe_slot_t *pipe;
    uint32_t desired, delta;
    if (!last_packet || payload_length != 0) {
        core->stats.protocol_errors++;
        return;
    }
    slot = core_find_pipe_slot(core, source_id, stream_id, false);
    if (slot < 0) return;
    pipe = &core->pipes[slot];
    if (pipe->direction != TRANSPORT_DIRECTION_RX ||
            pipe->state != TRANSPORT_PIPE_OPEN) {
        return;
    }
    if (core->config.pipe_lease_ms != 0) {
        pipe->lease_deadline_ms = core_now_ms(core) +
            core->config.pipe_lease_ms;
    }
    pipe->credit_wait_request = false;
    if (pipe->credit_update_pending) return;
    desired = pipe->transferred_bytes +
        (uint32_t)transport_ring_writable(&pipe->buffer);
    delta = desired - pipe->granted_bytes;
    if (delta >= 0x80000000u) desired = pipe->granted_bytes;
    if (core_queue_cts(core, pipe->peer_id, (uint8_t)pipe->session_id,
            TRANSPORT_WIRE_STREAM, TRANSPORT_CTS_ACCEPTED, desired)) {
        pipe->granted_bytes = desired;
        pipe->credit_update_pending = true;
    }
}

static void core_receive_packet(transport_core_t *core, const uint8_t *packet,
        size_t packet_length) {
    uint8_t wire_type, message_type, source_id, destination_id, message_id;
    uint8_t flags, payload_length;
    uint32_t crc;
    bool last_packet, intent;
    if (packet_length != NRF24_MAX_PAYLOAD) {
        return;
    }
    wire_type = packet[0];
    if ((wire_type & TRANSPORT_WIRE_PROTOCOL_MASK) !=
            TRANSPORT_WIRE_PROTOCOL_ID) {
        return;
    }
    message_type = wire_type & TRANSPORT_WIRE_TYPE_MASK;
    source_id = packet[1];
    destination_id = packet[2];
    message_id = packet[3];
    flags = packet[4];
    payload_length = flags & TRANSPORT_WIRE_LENGTH_MASK;
    last_packet = (flags & TRANSPORT_WIRE_LAST_PACKET) != 0;
    intent = (flags & TRANSPORT_WIRE_INTENT) != 0;
    if ((flags & TRANSPORT_WIRE_RESERVED_FLAG) != 0 ||
            payload_length > TRANSPORT_WIRE_MAX_DATA ||
            TRANSPORT_WIRE_HEADER_SIZE + payload_length >= packet_length) {
        core->stats.protocol_errors++;
        return;
    }
    crc = core_crc32_update(0, core->config.network_id,
        sizeof(core->config.network_id));
    crc = core_crc32_update(crc, packet, TRANSPORT_WIRE_HEADER_SIZE);
    crc = core_crc32_update(crc, packet + TRANSPORT_WIRE_HEADER_SIZE,
        payload_length);
    if (packet[TRANSPORT_WIRE_HEADER_SIZE + payload_length] !=
            (uint8_t)crc) {
        core->stats.protocol_errors++;
        return;
    }
    if (core_is_registration_type(message_type)) {
        if (!last_packet || intent) {
            core->stats.protocol_errors++;
            return;
        }
        core_receive_registration(core, message_type, source_id,
            destination_id, message_id,
            packet + TRANSPORT_WIRE_HEADER_SIZE, payload_length);
        return;
    }
    if (core->config.node_id == TRANSPORT_CORE_INVALID_ID ||
            destination_id != core->config.node_id ||
            source_id == TRANSPORT_WIRE_UNASSIGNED_ID) {
        return;
    }
    if (intent && !last_packet) {
        core->stats.protocol_errors++;
        return;
    }
    if (message_type == TRANSPORT_WIRE_CTS && !intent) {
        core_receive_cts(core, source_id, message_id,
            packet + TRANSPORT_WIRE_HEADER_SIZE, payload_length, last_packet);
    } else if (message_type == TRANSPORT_WIRE_PIPE_CLOSE_ACK && !intent) {
        core_receive_pipe_close_ack(core, source_id, message_id,
            payload_length, last_packet);
    } else if (message_type == TRANSPORT_WIRE_PIPE_CREDIT_REQUEST && !intent) {
        core_receive_pipe_credit_request(core, source_id, message_id,
            payload_length, last_packet);
    } else if (core_is_command_type(message_type) && intent) {
        core_receive_command_intent(core, message_type, source_id, message_id,
            packet + TRANSPORT_WIRE_HEADER_SIZE, payload_length);
    } else if (message_type == TRANSPORT_WIRE_STREAM && intent) {
        core_receive_pipe_intent(core, source_id, message_id, payload_length);
    } else if (core_is_command_type(message_type)) {
        core_receive_command_fragment(core, message_type, source_id,
            message_id, packet + TRANSPORT_WIRE_HEADER_SIZE, payload_length,
            last_packet);
    } else if (message_type == TRANSPORT_WIRE_STREAM) {
        core_receive_pipe_fragment(core, source_id, message_id,
            packet + TRANSPORT_WIRE_HEADER_SIZE, payload_length, last_packet);
    } else {
        core->stats.protocol_errors++;
    }
}

void transport_core_init(transport_core_t *core,
        const transport_core_config_t *config) {
    uint8_t i;
    if (core == NULL) return;
    memset(core, 0, sizeof(*core));
    if (config != NULL) core->config = *config;
    else core->config.node_id = TRANSPORT_CORE_INVALID_ID;
    if (core->config.preferred_pipe_event_bytes == 0)
        core->config.preferred_pipe_event_bytes = 512;
    core->max_continuous_tx_ms =
        TRANSPORT_CORE_DEFAULT_MAX_CONTINUOUS_TX_MS;
    core->forced_rx_ms = TRANSPORT_CORE_DEFAULT_FORCED_RX_MS;
    core->radio_phase = TRANSPORT_RADIO_RX_IDLE;
    if (core->config.node_id == 0 && !core->config.has_service_address) {
        core->config.service_address[0] = TRANSPORT_CORE_INVALID_ID;
        memcpy(core->config.service_address + 1, core->config.network_id,
            sizeof(core->config.network_id));
        core->config.has_service_address = true;
    }
    core->retry.max_restarts = core->config.max_rt_restarts;
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i)
        core->pipes[i].peer_id = TRANSPORT_CORE_INVALID_ID;
}

bool transport_core_attach_command_buffer(transport_core_t *core, uint8_t slot,
        uint8_t *buffer, size_t capacity) {
    transport_command_slot_t *command;
    if (core == NULL || slot >= TRANSPORT_CORE_COMMAND_SLOTS || buffer == NULL ||
            capacity == 0 || core->started) return false;
    command = &core->commands[slot];
    memset(command, 0, sizeof(*command));
    if (!transport_ring_attach(&command->buffer, buffer, capacity)) return false;
    command->state = TRANSPORT_COMMAND_FREE;
    return true;
}

bool transport_core_attach_pipe_buffer(transport_core_t *core, uint8_t pipe,
        uint8_t *buffer, size_t capacity) {
    return core != NULL && pipe < TRANSPORT_CORE_PIPE_SLOTS && !core->started &&
        transport_ring_attach(&core->pipes[pipe].buffer, buffer, capacity);
}

bool transport_core_attach_control_tx_buffer(transport_core_t *core,
        uint8_t *buffer, size_t capacity) {
    return core != NULL && !core->started &&
        capacity >= TRANSPORT_CORE_CONTROL_TX_SLOTS *
            TRANSPORT_CONTROL_RECORD_SIZE &&
        transport_ring_attach(&core->control_tx, buffer, capacity);
}

static bool core_configure_rx_addresses(transport_core_t *core,
        uint8_t node_id, bool resume_rx) {
    uint8_t i;
    uint8_t address[NRF24_ADDR_LEN];
    nrf24_t *radio = core->config.radio.radio;
    if (node_id == 0xfe || (node_id == TRANSPORT_CORE_INVALID_ID &&
            !core->config.has_service_address)) {
        return false;
    }
    if (node_id == 0 && (!core->config.has_service_address ||
            core->config.service_address[0] != TRANSPORT_CORE_INVALID_ID ||
            memcmp(core->config.service_address + 1, core->config.network_id,
                sizeof(core->config.network_id)) != 0)) {
        return false;
    }
    nrf24_stop_listening(radio);
    for (i = 0; i < 6; ++i) nrf24_close_rx_pipe(radio, i);
    if (node_id == TRANSPORT_CORE_INVALID_ID) {
        nrf24_open_rx_pipe(radio, 1, core->config.service_address);
    } else {
        address[0] = node_id;
        memcpy(address + 1, core->config.network_id,
            sizeof(core->config.network_id));
        nrf24_open_rx_pipe(radio, 1, address);
        if (node_id == 0) {
            nrf24_open_rx_pipe(radio, 2, core->config.service_address);
        }
    }
    if (resume_rx) nrf24_start_listening(radio);
    return core_radio_io_ok(core);
}

bool transport_core_start(transport_core_t *core) {
    uint8_t i;
    if (core == NULL || core->started || core->config.radio.radio == NULL) return false;
    for (i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        if (!transport_ring_is_attached(&core->commands[i].buffer))
            return false;
    }
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        if (!transport_ring_is_attached(&core->pipes[i].buffer))
            return false;
    }
    if (!transport_ring_is_attached(&core->control_tx)) return false;
    if (!core_configure_rx_addresses(core, core->config.node_id, false))
        return false;
    core->started = true;
    return true;
}

bool transport_core_set_node_id(transport_core_t *core, uint8_t node_id) {
    transport_critical_state_t critical;
    uint8_t i;
    bool configured;
    if (core == NULL || !core->started || node_id == 0xfe) return false;
    critical = core_enter(core);
    if (core->tx.active || core->registration_tx.pending ||
            transport_ring_readable(&core->control_tx) != 0 ||
            core_radio_irq_is_low(core)) {
        core_exit(core, critical);
        return false;
    }
    for (i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        if (core->commands[i].state != TRANSPORT_COMMAND_FREE) {
            core_exit(core, critical);
            return false;
        }
    }
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        if (core->pipes[i].state != TRANSPORT_PIPE_FREE) {
            core_exit(core, critical);
            return false;
        }
    }
    configured = core_configure_rx_addresses(core, node_id, true);
    if (configured) core->config.node_id = node_id;
    core_exit(core, critical);
    return configured;
}

void transport_core_stop(transport_core_t *core) {
    uint8_t i;
    transport_critical_state_t critical;
    if (core == NULL) return;
    critical = core_enter(core);
    core->started = false;
    core->event_read = 0;
    core->event_write = 0;
    memset(core->events, 0, sizeof(core->events));
    memset(&core->retry, 0, sizeof(core->retry));
    memset(&core->tx, 0, sizeof(core->tx));
    memset(&core->registration_tx, 0, sizeof(core->registration_tx));
    core->radio_phase = TRANSPORT_RADIO_RX_IDLE;
    core->rx_yield_started_ms = 0;
    core->rx_yield_deadline_ms = 0;
    core->tx_campaign_active = false;
    core->tx_campaign_started_ms = 0;
    core->retry.max_restarts = core->config.max_rt_restarts;
    for (i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        transport_ring_reset(&core->commands[i].buffer);
        core->commands[i].direction = TRANSPORT_DIRECTION_NONE;
        core->commands[i].state = TRANSPORT_COMMAND_FREE;
    }
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        core_reset_pipe(&core->pipes[i]);
    }
    memset(core->pipe_close_tombstones, 0,
        sizeof(core->pipe_close_tombstones));
    transport_ring_reset(&core->control_tx);
    core_exit(core, critical);
}

void transport_core_on_radio_irq(transport_core_t *core) {
    nrf24_t *radio;
    uint8_t iteration;
    if (core == NULL || !core->started || core->config.radio.radio == NULL) {
        return;
    }
    radio = core->config.radio.radio;
    core->stats.radio_irqs++;

    for (iteration = 0; iteration < TRANSPORT_CORE_IRQ_SERVICE_LIMIT;
            ++iteration) {
        uint8_t status;
        bool handled = false;
        if (!core_radio_io_ok(core)) {
            transport_core_report_error(core, TRANSPORT_ERROR_RADIO,
                TRANSPORT_CORE_INVALID_ID, 1u);
            return;
        }
        status = nrf24_read_status(radio);
        if (!core_radio_io_ok(core)) {
            transport_core_report_error(core, TRANSPORT_ERROR_RADIO,
                TRANSPORT_CORE_INVALID_ID, 2u);
            return;
        }

        if ((status & NRF24_STATUS_RX_DR) != 0) {
            uint8_t received = 0;
            while (received < TRANSPORT_IRQ_MAX_RX_PAYLOADS &&
                    !nrf24_rx_fifo_empty(radio)) {
                uint8_t packet[NRF24_MAX_PAYLOAD];
                size_t packet_length = 0;
                if (!core_radio_io_ok(core) || !nrf24_read_payload(radio,
                        packet, sizeof(packet), &packet_length) ||
                        !core_radio_io_ok(core)) {
                    transport_core_report_error(core, TRANSPORT_ERROR_RADIO,
                        TRANSPORT_CORE_INVALID_ID, 3u);
                    return;
                }
                core->stats.rx_packets++;
                core_receive_packet(core, packet, packet_length);
                received++;
            }
            if (nrf24_rx_fifo_empty(radio)) {
                nrf24_clear_irq(radio, NRF24_STATUS_RX_DR);
            }
            handled = true;
        }

        /* MAX_RT refers to the current FIFO head.  TX_DS may describe one or
           more earlier entries, so MAX_RT must win when both bits are latched. */
        if ((status & NRF24_STATUS_MAX_RT) != 0) {
            uint32_t now = core_now_ms(core);
            core->stats.max_rt_events++;
            if ((status & NRF24_STATUS_TX_DS) != 0) {
                /* A previous FIFO head succeeded before the next head reached
                   MAX_RT.  Retry accounting belongs to that new head. */
                memset(&core->retry, 0, sizeof(core->retry));
                core->retry.max_restarts = core->config.max_rt_restarts;
            }
            if (core->retry.state == TRANSPORT_RETRY_IDLE) {
                core->retry.state = TRANSPORT_RETRY_MAX_RT;
                core->retry.deadline_ms = now + core->config.max_rt_window_ms;
            }
            if ((core->retry.max_restarts ==
                    TRANSPORT_CORE_RESTARTS_UNBOUNDED ||
                 core->retry.restarts < core->retry.max_restarts) &&
                    !core_deadline_reached(now, core->retry.deadline_ms)) {
                core->retry.restarts++;
                core->stats.max_rt_restarts++;
                core->retry.state = TRANSPORT_RETRY_HW_ACTIVE;
                nrf24_set_ce(radio, false);
                nrf24_clear_irq(radio,
                    NRF24_STATUS_MAX_RT | NRF24_STATUS_TX_DS);
                nrf24_set_ce(radio, true);
            } else {
                bool retry_control = false;
                core->retry.state = TRANSPORT_RETRY_EXHAUSTED;
                nrf24_abort_send(radio);
                if (core->tx.active) {
                    if (core->tx.kind == TRANSPORT_TX_CONTROL) {
                        uint8_t packet[NRF24_MAX_PAYLOAD];
                        if (transport_ring_peek(&core->control_tx, packet,
                                sizeof(packet)) == sizeof(packet)) {
                            core_control_failed(core, packet);
                            if ((packet[0] & TRANSPORT_WIRE_TYPE_MASK) ==
                                    TRANSPORT_WIRE_PIPE_CLOSE_ACK ||
                                    (packet[0] & TRANSPORT_WIRE_TYPE_MASK) ==
                                    TRANSPORT_WIRE_PIPE_CREDIT_REQUEST) {
                                retry_control = true;
                            }
                        }
                        (void)transport_ring_discard(&core->control_tx,
                            TRANSPORT_CONTROL_RECORD_SIZE);
                    } else if (core->tx.kind ==
                            TRANSPORT_TX_REGISTRATION) {
                        core_finish_registration_tx(core, false,
                            TRANSPORT_REGISTRATION_FAILURE_RADIO);
                    } else if (core->tx.kind == TRANSPORT_TX_COMMAND &&
                            core->tx.slot < TRANSPORT_CORE_COMMAND_SLOTS) {
                        core_finish_outbound_command(core, core->tx.slot,
                            false, TRANSPORT_COMMAND_FAILURE_RADIO);
                    } else if (core->tx.kind == TRANSPORT_TX_PIPE &&
                            core->tx.slot < TRANSPORT_CORE_PIPE_SLOTS) {
                        transport_pipe_slot_t *pipe =
                            &core->pipes[core->tx.slot];
                        if (pipe->state == TRANSPORT_PIPE_TX_CLOSING &&
                                core->tx.last_packet &&
                                core->tx.staged_payload_bytes == 0) {
                            pipe->state = TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK;
                            pipe->close_retry_at_ms = now +
                                core_pipe_close_retry_delay(core);
                            retry_control = true;
                        } else {
                            core_finish_outbound_pipe(core, core->tx.slot,
                                false, TRANSPORT_PIPE_FAILURE_RADIO);
                        }
                    }
                    core_release_tx_owner(core);
                }
                if (!retry_control) {
                    transport_core_report_error(core, TRANSPORT_ERROR_RADIO,
                        TRANSPORT_CORE_INVALID_ID, NRF24_STATUS_MAX_RT);
                }
                core_tx_kick(core);
                if (!core->tx.active) core_restore_rx(core);
            }
            handled = true;
        } else if ((status & NRF24_STATUS_TX_DS) != 0) {
            uint32_t now = core_now_ms(core);
            nrf24_clear_irq(radio, NRF24_STATUS_TX_DS);
            memset(&core->retry, 0, sizeof(core->retry));
            core->retry.max_restarts = core->config.max_rt_restarts;
            if (core->tx.active) {
                if (core_tx_time_expired(core, now)) {
                    core->tx.drain_requested = true;
                    core->radio_phase = TRANSPORT_RADIO_TX_DRAINING;
                }
                if (nrf24_tx_fifo_empty(radio)) {
                    transport_tx_kind_t kind = core->tx.kind;
                    uint8_t slot = core->tx.slot;
                    bool last_packet = core->tx.last_packet;
                    bool forced_rx = core_tx_time_expired(core, now);
                    bool priority_drain = core->tx.drain_requested && !forced_rx;
                    nrf24_set_ce(radio, false);
                    core_commit_staged_tx(core);
                    if (kind == TRANSPORT_TX_CONTROL) {
                        uint8_t sent_packet[NRF24_MAX_PAYLOAD];
                        if (transport_ring_peek(&core->control_tx,
                                sent_packet, sizeof(sent_packet)) ==
                                sizeof(sent_packet)) {
                            core_control_acked(core, sent_packet);
                        }
                        (void)transport_ring_discard(&core->control_tx,
                            TRANSPORT_CONTROL_RECORD_SIZE);
                        core->stats.control_packets_acked++;
                    } else if (kind == TRANSPORT_TX_REGISTRATION) {
                        core_finish_registration_tx(core, true, 0);
                    } else if (kind == TRANSPORT_TX_COMMAND &&
                            slot < TRANSPORT_CORE_COMMAND_SLOTS) {
                        transport_command_slot_t *command =
                            &core->commands[slot];
                        if (last_packet) {
                            core_finish_outbound_command(core, slot, true,
                                command->target_length);
                        } else if (core->config.command_lease_ms != 0) {
                            command->lease_deadline_ms = now +
                                core->config.command_lease_ms;
                        }
                    } else if (kind == TRANSPORT_TX_PIPE &&
                            slot < TRANSPORT_CORE_PIPE_SLOTS) {
                        transport_pipe_slot_t *pipe = &core->pipes[slot];
                        if (last_packet) {
                            pipe->state = TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK;
                            pipe->close_retry_at_ms = now +
                                core_pipe_close_retry_delay(core);
                        } else if (core->config.pipe_lease_ms != 0) {
                            pipe->lease_deadline_ms = now +
                                core->config.pipe_lease_ms;
                        } else {
                            pipe->lease_deadline_ms = 0;
                        }
                    }
                    if (forced_rx && core->forced_rx_ms != 0) {
                        core_enter_forced_rx(core, now);
                    } else {
                        if (forced_rx) core->tx_campaign_active = false;
                        core_release_tx_owner(core);
                        core->radio_phase = TRANSPORT_RADIO_RX_IDLE;
                        core_tx_kick(core);
                        if (!core->tx.active) core_restore_rx(core);
                    }
                    (void)priority_drain;
                } else if (!core->tx.drain_requested) {
                    if (!core_fill_tx_owner(core)) {
                        nrf24_abort_send(radio);
                        if (core->tx.kind == TRANSPORT_TX_COMMAND &&
                                core->tx.slot < TRANSPORT_CORE_COMMAND_SLOTS) {
                            core_finish_outbound_command(core, core->tx.slot,
                                false, TRANSPORT_COMMAND_FAILURE_RADIO);
                        } else if (core->tx.kind == TRANSPORT_TX_PIPE &&
                                core->tx.slot < TRANSPORT_CORE_PIPE_SLOTS) {
                            core_finish_outbound_pipe(core, core->tx.slot,
                                false, TRANSPORT_PIPE_FAILURE_RADIO);
                        }
                        core_release_tx_owner(core);
                        core_restore_rx(core);
                        transport_core_report_error(core,
                            TRANSPORT_ERROR_RADIO,
                            TRANSPORT_CORE_INVALID_ID, 0x46494cu);
                    }
                }
            }
            handled = true;
        }

        if (!core_radio_io_ok(core)) {
            transport_core_report_error(core, TRANSPORT_ERROR_RADIO,
                TRANSPORT_CORE_INVALID_ID, 4u);
            return;
        }
        if (!core_radio_irq_is_low(core)) {
            transport_core_service(core);
            return;
        }
        if (!handled) {
            transport_core_report_error(core, TRANSPORT_ERROR_RADIO,
                TRANSPORT_CORE_INVALID_ID, status);
            return;
        }
    }
    transport_core_report_error(core, TRANSPORT_ERROR_RADIO,
        TRANSPORT_CORE_INVALID_ID, 0x495251u);
    transport_core_service(core);
}

void transport_core_service(transport_core_t *core) {
    uint32_t now;
    uint8_t i;
    transport_critical_state_t critical;
    if (core == NULL || !core->started) return;
    now = core_now_ms(core);
    if (core->radio_phase == TRANSPORT_RADIO_RX_YIELD) {
        if (!core_deadline_reached(now, core->rx_yield_deadline_ms)) return;
        core->radio_phase = TRANSPORT_RADIO_RX_IDLE;
    }
    critical = core_enter(core);
    if (core->tx.active && core_tx_time_expired(core, now)) {
        core->tx.drain_requested = true;
        core->radio_phase = TRANSPORT_RADIO_TX_DRAINING;
    }
    core_exit(core, critical);
    for (i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        transport_command_slot_t *command = &core->commands[i];
        if (core->config.command_lease_ms != 0 &&
                command->direction == TRANSPORT_DIRECTION_RX &&
                (command->state == TRANSPORT_COMMAND_CTS_SENT ||
                 command->state == TRANSPORT_COMMAND_RECEIVING) &&
                core_deadline_reached(now, command->lease_deadline_ms)) {
            core_discard_command(command);
            transport_core_report_error(core, TRANSPORT_ERROR_COMMAND_TIMEOUT,
                i, command->transaction_id);
        } else if (core->config.command_lease_ms != 0 &&
                command->direction == TRANSPORT_DIRECTION_TX &&
                (command->state == TRANSPORT_COMMAND_TX_WAIT_CTS ||
                 command->state == TRANSPORT_COMMAND_TX_SENDING) &&
                core_deadline_reached(now, command->lease_deadline_ms)) {
            if (core->tx.active && core->tx.kind == TRANSPORT_TX_COMMAND &&
                    core->tx.slot == i) {
                transport_critical_state_t abort_critical = core_enter(core);
                nrf24_abort_send(core->config.radio.radio);
                core_release_tx_owner(core);
                core_exit(core, abort_critical);
            }
            core_finish_outbound_command(core, i, false,
                TRANSPORT_COMMAND_FAILURE_TIMEOUT);
            core_restore_rx(core);
        }
    }
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        transport_pipe_slot_t *pipe = &core->pipes[i];
        if (pipe->direction == TRANSPORT_DIRECTION_RX &&
                pipe->state == TRANSPORT_PIPE_CLOSED &&
                pipe->close_ack_pending) {
            if (core->config.pipe_lease_ms != 0 &&
                    core_deadline_reached(now, pipe->lease_deadline_ms)) {
                pipe->close_ack_pending = false;
                if (pipe->terminal_event_polled) core_reset_pipe(pipe);
            } else if (!pipe->close_ack_queued &&
                    !pipe->close_ack_wait_duplicate) {
                (void)core_queue_pipe_close_ack(core, i);
            }
        }
        if ((pipe->state == TRANSPORT_PIPE_CLOSED ||
                pipe->state == TRANSPORT_PIPE_FAILED) &&
                pipe->close_event_pending) {
            (void)core_emit_pipe_terminal(core, i);
        } else if (core->config.pipe_lease_ms != 0 &&
                pipe->direction == TRANSPORT_DIRECTION_RX &&
                (pipe->state == TRANSPORT_PIPE_CTS_SENT ||
                 pipe->state == TRANSPORT_PIPE_OPEN) &&
                core_deadline_reached(now, pipe->lease_deadline_ms)) {
            transport_ring_reset(&pipe->buffer);
            pipe->rx_event_pending = false;
            pipe->state = TRANSPORT_PIPE_FAILED;
            pipe->close_event_pending = true;
            pipe->terminal_event_reason = TRANSPORT_ERROR_PIPE_TIMEOUT;
            core->stats.pipes_failed++;
            (void)core_emit_pipe_terminal(core, i);
        } else if (core->config.pipe_lease_ms != 0 &&
                pipe->direction == TRANSPORT_DIRECTION_TX &&
                (pipe->state == TRANSPORT_PIPE_TX_WAIT_CTS ||
                 pipe->state == TRANSPORT_PIPE_TX_CLOSING ||
                 pipe->state == TRANSPORT_PIPE_TX_OPEN ||
                 pipe->state == TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK) &&
                core_deadline_reached(now, pipe->lease_deadline_ms)) {
            if (core->tx.active && core->tx.kind == TRANSPORT_TX_PIPE &&
                    core->tx.slot == i) {
                transport_critical_state_t abort_critical = core_enter(core);
                nrf24_abort_send(core->config.radio.radio);
                core_release_tx_owner(core);
                core_exit(core, abort_critical);
            } else if (core->tx.active &&
                    core->tx.kind == TRANSPORT_TX_CONTROL) {
                uint8_t packet[NRF24_MAX_PAYLOAD];
                if (transport_ring_peek(&core->control_tx, packet,
                        sizeof(packet)) == sizeof(packet) &&
                        (packet[0] & TRANSPORT_WIRE_TYPE_MASK) ==
                            TRANSPORT_WIRE_PIPE_CREDIT_REQUEST &&
                        packet[2] == pipe->peer_id &&
                        packet[3] == (uint8_t)pipe->session_id) {
                    transport_critical_state_t abort_critical =
                        core_enter(core);
                    nrf24_abort_send(core->config.radio.radio);
                    (void)transport_ring_discard(&core->control_tx,
                        TRANSPORT_CONTROL_RECORD_SIZE);
                    core_release_tx_owner(core);
                    core_exit(core, abort_critical);
                }
            }
            core_finish_outbound_pipe(core, i, false,
                TRANSPORT_PIPE_FAILURE_TIMEOUT);
            if (!core->tx.active) core_restore_rx(core);
        } else if (pipe->direction == TRANSPORT_DIRECTION_TX &&
                pipe->state == TRANSPORT_PIPE_TX_WAIT_CLOSE_ACK &&
                core_deadline_reached(now, pipe->close_retry_at_ms)) {
            pipe->state = TRANSPORT_PIPE_TX_CLOSING;
        } else {
            core_maybe_queue_rx_credit(core, i);
            core_maybe_queue_tx_credit_request(core, i, now);
        }
    }
    for (i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        transport_pipe_close_tombstone_t *tombstone =
            &core->pipe_close_tombstones[i];
        if (!tombstone->valid) continue;
        if (core->config.pipe_lease_ms != 0 &&
                core_deadline_reached(now, tombstone->deadline_ms)) {
            memset(tombstone, 0, sizeof(*tombstone));
        } else if (tombstone->ack_pending && !tombstone->ack_queued &&
                !tombstone->wait_duplicate) {
            (void)core_queue_tombstone_close_ack(core, i);
        }
    }
    core_tx_kick(core);
}

bool transport_core_set_radio_schedule(transport_core_t *core,
        uint16_t max_continuous_tx_ms, uint16_t forced_rx_ms) {
    transport_critical_state_t critical;
    uint32_t now;
    if (core == NULL) return false;
    critical = core_enter(core);
    core->max_continuous_tx_ms = max_continuous_tx_ms;
    core->forced_rx_ms = forced_rx_ms;
    now = core_now_ms(core);
    if (core->radio_phase == TRANSPORT_RADIO_RX_YIELD) {
        core->rx_yield_deadline_ms = core->rx_yield_started_ms + forced_rx_ms;
    }
    if (core->tx.active && core_tx_time_expired(core, now)) {
        core->tx.drain_requested = true;
        core->radio_phase = TRANSPORT_RADIO_TX_DRAINING;
    }
    core_exit(core, critical);
    return true;
}

void transport_core_get_radio_schedule(const transport_core_t *core,
        uint16_t *max_continuous_tx_ms, uint16_t *forced_rx_ms) {
    transport_critical_state_t critical;
    uint16_t max_tx_snapshot, rx_snapshot;
    if (core == NULL) return;
    critical = core_enter((transport_core_t *)core);
    max_tx_snapshot = core->max_continuous_tx_ms;
    rx_snapshot = core->forced_rx_ms;
    core_exit((transport_core_t *)core, critical);
    if (max_continuous_tx_ms != NULL) *max_continuous_tx_ms = max_tx_snapshot;
    if (forced_rx_ms != NULL) *forced_rx_ms = rx_snapshot;
}

bool transport_core_commit_command(transport_core_t *core, uint8_t slot,
        uint8_t source_id, uint16_t transaction_id, size_t length) {
    transport_event_descriptor_t descriptor;
    transport_command_slot_t *command;
    transport_critical_state_t critical;
    bool queued;
    if (core == NULL || slot >= TRANSPORT_CORE_COMMAND_SLOTS) return false;
    critical = core_enter(core);
    command = &core->commands[slot];
    if (!transport_ring_is_attached(&command->buffer) ||
            length > command->buffer.capacity || length > UINT16_MAX ||
            transport_ring_readable(&command->buffer) != length ||
            command->state == TRANSPORT_COMMAND_COMPLETE) {
        if (transport_ring_is_attached(&command->buffer) &&
                length > command->buffer.capacity)
            transport_core_report_error(core, TRANSPORT_ERROR_COMMAND_TOO_LARGE,
                slot, (uint32_t)length);
        core_exit(core, critical);
        return false;
    }
    command->peer_id = source_id;
    command->transaction_id = transaction_id;
    command->state = TRANSPORT_COMMAND_COMPLETE;
    memset(&descriptor, 0, sizeof(descriptor));
    descriptor.fields.type = TRANSPORT_EVENT_COMMAND_READY;
    descriptor.fields.object_id = slot;
    descriptor.fields.source_id = source_id;
    descriptor.fields.transaction_id = transaction_id;
    descriptor.fields.value0 = command->message_type;
    descriptor.payload_kind = TRANSPORT_EVENT_PAYLOAD_COMMAND_SLOT;
    descriptor.payload_index = slot;
    queued = queue_descriptor(core, &descriptor);
    if (!queued) core_discard_command(command);
    core_exit(core, critical);
    return queued;
}

size_t transport_core_pipe_rx_write(transport_core_t *core, uint8_t pipe_id,
        const uint8_t *data, size_t length) {
    size_t written;
    transport_critical_state_t critical;
    if (core == NULL || pipe_id >= TRANSPORT_CORE_PIPE_SLOTS || data == NULL) return 0;
    critical = core_enter(core);
    /* A radio fragment is committed atomically: never place a truncated
       fragment in the stream ring. */
    if (transport_ring_writable(&core->pipes[pipe_id].buffer) < length ||
            (!core->pipes[pipe_id].rx_event_pending &&
             queue_used(core) >= TRANSPORT_CORE_EVENT_QUEUE_SIZE)) {
        core->stats.rx_overruns++;
        transport_core_report_error(core, TRANSPORT_ERROR_RX_OVERRUN, pipe_id,
            (uint32_t)length);
        core_exit(core, critical);
        return 0;
    }
    written = transport_ring_write(&core->pipes[pipe_id].buffer, data, length);
    core->pipes[pipe_id].transferred_bytes += (uint32_t)written;
    core->stats.pipe_rx_bytes += (uint32_t)written;
    if (written != length) {
        core->stats.rx_overruns++;
        transport_core_report_error(core, TRANSPORT_ERROR_RX_OVERRUN, pipe_id,
            (uint32_t)(length - written));
    }
    if (written != 0 && !queue_pipe_data_event(core, pipe_id))
        core->pipes[pipe_id].state = TRANSPORT_PIPE_FAILED;
    core_exit(core, critical);
    return written;
}

size_t transport_core_pipe_tx_write(transport_core_t *core, uint8_t pipe_id,
        const uint8_t *data, size_t length) {
    transport_pipe_slot_t *pipe;
    transport_critical_state_t critical;
    size_t written;
    if (core == NULL || pipe_id >= TRANSPORT_CORE_PIPE_SLOTS ||
            (data == NULL && length != 0)) {
        return 0;
    }
    critical = core_enter(core);
    pipe = &core->pipes[pipe_id];
    if (pipe->direction != TRANSPORT_DIRECTION_TX ||
            (pipe->state != TRANSPORT_PIPE_TX_INTENT_QUEUED &&
             pipe->state != TRANSPORT_PIPE_TX_WAIT_CTS &&
             pipe->state != TRANSPORT_PIPE_TX_OPEN)) {
        core_exit(core, critical);
        return 0;
    }
    written = transport_ring_write(&pipe->buffer, data, length);
    if (written != 0 && core->config.pipe_lease_ms != 0) {
        pipe->lease_deadline_ms = core_now_ms(core) +
            core->config.pipe_lease_ms;
    }
    core_exit(core, critical);
    if (written != 0) core_tx_kick(core);
    return written;
}

size_t transport_core_pipe_tx_read(transport_core_t *core, uint8_t pipe_id,
        uint8_t *data, size_t length) {
    size_t count;
    if (core == NULL || pipe_id >= TRANSPORT_CORE_PIPE_SLOTS) return 0;
    count = transport_ring_read(&core->pipes[pipe_id].buffer, data, length);
    if (count == 0 && length != 0) core->stats.tx_underruns++;
    return count;
}

bool transport_core_emit(transport_core_t *core,
        const transport_event_fields_t *fields) {
    transport_event_descriptor_t descriptor;
    if (core == NULL || fields == NULL || fields->data_length != 0) return false;
    memset(&descriptor, 0, sizeof(descriptor));
    descriptor.fields = *fields;
    return queue_descriptor(core, &descriptor);
}

void transport_core_report_error(transport_core_t *core, transport_error_t error,
        uint8_t object_id, uint32_t detail) {
    transport_event_fields_t fields;
    transport_critical_state_t critical;
    if (core == NULL || error == TRANSPORT_ERROR_NONE) return;
    critical = core_enter(core);
    if ((unsigned)error < 32u)
        core->sticky_errors |= (uint32_t)1u << (unsigned)error;
    if (error == TRANSPORT_ERROR_PROTOCOL) core->stats.protocol_errors++;
    memset(&fields, 0, sizeof(fields));
    fields.type = TRANSPORT_EVENT_CORE_ERROR;
    fields.flags = TRANSPORT_EVENT_FLAG_STICKY;
    fields.object_id = object_id;
    fields.value0 = (uint32_t)error;
    fields.value1 = detail;
    (void)transport_core_emit(core, &fields);
    core_exit(core, critical);
}

transport_poll_result_t transport_core_poll_into(transport_core_t *core,
        uint8_t *buffer, size_t capacity, size_t *event_length,
        size_t *required_capacity) {
    transport_event_descriptor_t *descriptor;
    transport_event_fields_t fields;
    size_t payload_capacity, payload_length = 0, record_length;
    uint8_t payload_kind, payload_index;
    bool keep_pipe_event;
    transport_poll_result_t result = TRANSPORT_POLL_EMPTY;
    if (event_length != NULL) *event_length = 0;
    if (required_capacity != NULL) *required_capacity = TRANSPORT_EVENT_HEADER_SIZE;
    if (core == NULL || buffer == NULL || event_length == NULL)
        return TRANSPORT_POLL_INVALID_ARGUMENT;
    if (capacity < TRANSPORT_EVENT_HEADER_SIZE) return TRANSPORT_POLL_BUFFER_TOO_SMALL;

    transport_core_service(core);

    for (;;) {
        keep_pipe_event = false;
        payload_length = 0;
        if (core->event_read == core->event_write) break;
        descriptor = &core->events[core->event_read % TRANSPORT_CORE_EVENT_QUEUE_SIZE];
        fields = descriptor->fields;
        payload_kind = descriptor->payload_kind;
        payload_index = descriptor->payload_index;
        payload_capacity = capacity - TRANSPORT_EVENT_HEADER_SIZE;

    if (payload_kind == TRANSPORT_EVENT_PAYLOAD_COMMAND_SLOT) {
        transport_command_slot_t *command;
        if (payload_index >= TRANSPORT_CORE_COMMAND_SLOTS)
            { result = TRANSPORT_POLL_INVALID_ARGUMENT; break; }
        command = &core->commands[payload_index];
        payload_length = transport_ring_readable(&command->buffer);
        if (required_capacity != NULL)
            *required_capacity = TRANSPORT_EVENT_HEADER_SIZE + payload_length;
        if (payload_length > payload_capacity || payload_length > UINT16_MAX)
            { result = TRANSPORT_POLL_BUFFER_TOO_SMALL; break; }
        (void)transport_ring_read(&command->buffer,
            transport_event_get_data(buffer), payload_length);
        command->direction = TRANSPORT_DIRECTION_NONE;
        command->target_length = 0;
        command->state = TRANSPORT_COMMAND_FREE;
    } else if (payload_kind == TRANSPORT_EVENT_PAYLOAD_PIPE_RX) {
        transport_pipe_slot_t *pipe;
        size_t preferred;
        if (payload_index >= TRANSPORT_CORE_PIPE_SLOTS)
            { result = TRANSPORT_POLL_INVALID_ARGUMENT; break; }
        pipe = &core->pipes[payload_index];
        preferred = core->config.preferred_pipe_event_bytes;
        payload_length = transport_ring_readable(&pipe->buffer);
        if (payload_length > preferred) payload_length = preferred;
        if (required_capacity != NULL)
            *required_capacity = TRANSPORT_EVENT_HEADER_SIZE + payload_length;
        if (payload_length != 0 && payload_capacity == 0)
            { result = TRANSPORT_POLL_BUFFER_TOO_SMALL; break; }
        if (payload_length > payload_capacity) payload_length = payload_capacity;
        if (payload_length == 0) {
            pipe->rx_event_pending = false;
            dequeue_descriptor(core);
            continue;
        }
        (void)transport_ring_read(&pipe->buffer,
            transport_event_get_data(buffer), payload_length);
        keep_pipe_event = transport_ring_readable(&pipe->buffer) != 0;
        if (keep_pipe_event) fields.flags |= TRANSPORT_EVENT_FLAG_MORE_DATA;
        else pipe->rx_event_pending = false;
        core_maybe_queue_rx_credit(core, payload_index);
    } else if (payload_kind == TRANSPORT_EVENT_PAYLOAD_INLINE) {
        payload_length = fields.data_length;
        if (payload_length > sizeof(descriptor->inline_data)) {
            result = TRANSPORT_POLL_INVALID_ARGUMENT;
            break;
        }
        if (required_capacity != NULL) {
            *required_capacity = TRANSPORT_EVENT_HEADER_SIZE + payload_length;
        }
        if (payload_length > payload_capacity) {
            result = TRANSPORT_POLL_BUFFER_TOO_SMALL;
            break;
        }
        memcpy(transport_event_get_data(buffer), descriptor->inline_data,
            payload_length);
    }

    fields.data_length = (uint16_t)payload_length;
    record_length = transport_event_record_size(fields.data_length);
    if (!transport_event_write_header(buffer, capacity, &fields))
        { result = TRANSPORT_POLL_BUFFER_TOO_SMALL; break; }
    if (!keep_pipe_event) dequeue_descriptor(core);
    if (fields.type == TRANSPORT_EVENT_PIPE_TX_SPACE &&
            fields.object_id < TRANSPORT_CORE_PIPE_SLOTS) {
        core->pipes[fields.object_id].tx_space_event_pending = false;
    }
    if ((fields.type == TRANSPORT_EVENT_PIPE_FAILED ||
         fields.type == TRANSPORT_EVENT_PIPE_CLOSED) &&
            fields.object_id < TRANSPORT_CORE_PIPE_SLOTS) {
        transport_pipe_slot_t *pipe = &core->pipes[fields.object_id];
        if (fields.type == TRANSPORT_EVENT_PIPE_CLOSED &&
                pipe->direction == TRANSPORT_DIRECTION_RX &&
                pipe->close_ack_pending) {
            pipe->terminal_event_polled = true;
        } else {
            if (fields.type == TRANSPORT_EVENT_PIPE_CLOSED &&
                    pipe->direction == TRANSPORT_DIRECTION_RX) {
                core_remember_closed_pipe(core, fields.object_id);
            }
            core_reset_pipe(pipe);
        }
    }
    core_tx_kick(core);
    core->stats.events_polled++;
    *event_length = record_length;
    result = TRANSPORT_POLL_EVENT;
    break;
    }
    return result;
}

const transport_core_stats_t *transport_core_get_stats(const transport_core_t *core) {
    return core == NULL ? NULL : &core->stats;
}

uint32_t transport_core_get_sticky_errors(const transport_core_t *core) {
    uint32_t errors;
    transport_critical_state_t critical;
    if (core == NULL) return 0;
    critical = core_enter((transport_core_t *)core);
    errors = core->sticky_errors;
    core_exit((transport_core_t *)core, critical);
    return errors;
}

void transport_core_clear_sticky_errors(transport_core_t *core, uint32_t mask) {
    transport_critical_state_t critical;
    if (core == NULL) return;
    critical = core_enter(core);
    core->sticky_errors &= ~mask;
    core_exit(core, critical);
}
