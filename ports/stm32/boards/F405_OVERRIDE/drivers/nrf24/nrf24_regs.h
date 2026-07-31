#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_EN_AA       0x01
#define NRF24_REG_EN_RXADDR   0x02
#define NRF24_REG_SETUP_AW    0x03
#define NRF24_REG_SETUP_RETR  0x04
#define NRF24_REG_RF_CH       0x05
#define NRF24_REG_RF_SETUP    0x06
#define NRF24_REG_STATUS      0x07
#define NRF24_REG_OBSERVE_TX  0x08
#define NRF24_REG_RX_ADDR_P0  0x0A
#define NRF24_REG_TX_ADDR     0x10
#define NRF24_REG_RX_PW_P0    0x11
#define NRF24_REG_FIFO_STATUS 0x17
#define NRF24_REG_DYNPD       0x1C

#define NRF24_CMD_R_RX_PAYLOAD 0x61
#define NRF24_CMD_W_TX_PAYLOAD 0xA0
#define NRF24_CMD_FLUSH_TX     0xE1
#define NRF24_CMD_FLUSH_RX     0xE2
#define NRF24_CMD_NOP          0xFF

#define NRF24_STATUS_RX_DR  0x40
#define NRF24_STATUS_TX_DS  0x20
#define NRF24_STATUS_MAX_RT 0x10

#define NRF24_FIFO_RX_EMPTY 0x01
#define NRF24_FIFO_TX_FULL  0x20

#define NRF24_CFG_EN_CRC  0x08
#define NRF24_CFG_CRCO    0x04
#define NRF24_CFG_PWR_UP  0x02
#define NRF24_CFG_PRIM_RX 0x01

#define NRF24_SPEED_1M    0x00
#define NRF24_SPEED_2M    0x08
#define NRF24_SPEED_250K  0x20
#define NRF24_POWER_0     0x00
#define NRF24_POWER_1     0x02
#define NRF24_POWER_2     0x04
#define NRF24_POWER_3     0x06

#define NRF24_MAX_PAYLOAD 32
#define NRF24_ADDR_LEN    5


