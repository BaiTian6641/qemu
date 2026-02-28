/*
 * ESP32-S3 I2C Controller
 *
 * Register layout follows ESP32-S3 TRM v1.2 Chapter 28.
 * Key differences from ESP32: only 8 command registers (vs 16),
 * CLK_CONF at 0x54, FILTER_CFG at 0x50, FIFO memory-mapped at
 * 0x100 (TX) / 0x180 (RX), total register space 0x200.
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"
#include "qemu/fifo8.h"
#include "hw/i2c/i2c.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_I2C "esp32s3.i2c"
#define ESP32S3_I2C(obj) OBJECT_CHECK(ESP32S3I2CState, (obj), TYPE_ESP32S3_I2C)

#define ESP32S3_I2C_REG_SIZE    0x200
#define ESP32S3_I2C_FIFO_LEN   32
#define ESP32S3_I2C_CMD_COUNT  8

/* ------------------------------------------------------------------ */
/*  Register offsets (ESP32-S3 specific)                               */
/* ------------------------------------------------------------------ */

#define ESP32S3_I2C_SCL_LOW_PERIOD_OFF   0x00
#define ESP32S3_I2C_CTR_OFF              0x04
#define ESP32S3_I2C_SR_OFF               0x08
#define ESP32S3_I2C_TO_OFF               0x0C
#define ESP32S3_I2C_SLAVE_ADDR_OFF       0x10
#define ESP32S3_I2C_FIFO_ST_OFF          0x14
#define ESP32S3_I2C_FIFO_CONF_OFF        0x18
#define ESP32S3_I2C_DATA_OFF             0x1C
#define ESP32S3_I2C_INT_RAW_OFF          0x20
#define ESP32S3_I2C_INT_CLR_OFF          0x24
#define ESP32S3_I2C_INT_ENA_OFF          0x28
#define ESP32S3_I2C_INT_ST_OFF           0x2C
#define ESP32S3_I2C_SDA_HOLD_OFF         0x30
#define ESP32S3_I2C_SDA_SAMPLE_OFF       0x34
#define ESP32S3_I2C_SCL_HIGH_PERIOD_OFF  0x38
#define ESP32S3_I2C_SCL_START_HOLD_OFF   0x40
#define ESP32S3_I2C_SCL_RSTART_SETUP_OFF 0x44
#define ESP32S3_I2C_SCL_STOP_HOLD_OFF    0x48
#define ESP32S3_I2C_SCL_STOP_SETUP_OFF   0x4C
#define ESP32S3_I2C_FILTER_CFG_OFF       0x50
#define ESP32S3_I2C_CLK_CONF_OFF         0x54
#define ESP32S3_I2C_COMD0_OFF            0x58
/* COMD1..COMD7 at 0x5C..0x74 (stride 4) */
#define ESP32S3_I2C_SCL_ST_TIME_OUT_OFF      0x78
#define ESP32S3_I2C_SCL_MAIN_ST_TIME_OUT_OFF 0x7C
#define ESP32S3_I2C_SCL_SP_CONF_OFF          0x80
#define ESP32S3_I2C_SCL_STRETCH_CONF_OFF     0x84
#define ESP32S3_I2C_DATE_OFF                 0xF8
#define ESP32S3_I2C_TXFIFO_START_OFF         0x100
#define ESP32S3_I2C_RXFIFO_START_OFF         0x180

/* CTR register bits */
#define I2C_S3_CTR_MS_MODE_BIT       4
#define I2C_S3_CTR_TRANS_START_BIT   5
#define I2C_S3_CTR_CONF_UPGATE_BIT   6
#define I2C_S3_CTR_CLK_EN_BIT        8

/* FIFO_CONF register bits */
#define I2C_S3_FIFO_CONF_NONFIFO_EN_BIT   10
#define I2C_S3_FIFO_CONF_RX_FIFO_RST_BIT  12
#define I2C_S3_FIFO_CONF_TX_FIFO_RST_BIT  13
#define I2C_S3_FIFO_CONF_FIFO_PRT_EN_BIT  14

/* INT_RAW / INT_ST / INT_ENA bits */
#define I2C_S3_INT_RXFIFO_WM       (1 << 0)
#define I2C_S3_INT_TXFIFO_WM       (1 << 1)
#define I2C_S3_INT_RXFIFO_OVF      (1 << 2)
#define I2C_S3_INT_END_DETECT      (1 << 3)
#define I2C_S3_INT_BYTE_TRANS_DONE (1 << 4)
#define I2C_S3_INT_ARBITRATION_LOST (1 << 5)
#define I2C_S3_INT_MST_TXFIFO_UDF  (1 << 6)
#define I2C_S3_INT_TRANS_COMPLETE  (1 << 7)
#define I2C_S3_INT_TIME_OUT        (1 << 8)
#define I2C_S3_INT_TRANS_START     (1 << 9)
#define I2C_S3_INT_NACK            (1 << 10)
#define I2C_S3_INT_TXFIFO_OVF     (1 << 11)
#define I2C_S3_INT_RXFIFO_UDF     (1 << 12)
#define I2C_S3_INT_SCL_ST_TO      (1 << 13)
#define I2C_S3_INT_SCL_MAIN_ST_TO (1 << 14)
#define I2C_S3_INT_DET_START      (1 << 15)

/* CMD register fields */
#define I2C_S3_CMD_BYTE_NUM_SHIFT  0
#define I2C_S3_CMD_BYTE_NUM_MASK   0xFF
#define I2C_S3_CMD_ACK_CHECK_EN    (1 << 8)
#define I2C_S3_CMD_ACK_EXP         (1 << 9)
#define I2C_S3_CMD_ACK_VAL         (1 << 10)
#define I2C_S3_CMD_OPCODE_SHIFT    11
#define I2C_S3_CMD_OPCODE_MASK     (0x7 << 11)
#define I2C_S3_CMD_DONE            (1U << 31)

/* Opcodes */
typedef enum {
    I2C_S3_OPCODE_RSTART = 6,
    I2C_S3_OPCODE_WRITE  = 1,
    I2C_S3_OPCODE_READ   = 3,
    I2C_S3_OPCODE_STOP   = 2,
    I2C_S3_OPCODE_END    = 4,
} Esp32s3I2cOpcode;

/* STATUS register fields */
#define I2C_S3_SR_RESP_REC_BIT    0
#define I2C_S3_SR_ACK_REC_BIT     3
#define I2C_S3_SR_BUS_BUSY_BIT    4
#define I2C_S3_SR_RXFIFO_CNT_SHIFT 8
#define I2C_S3_SR_TXFIFO_CNT_SHIFT 18

/* ------------------------------------------------------------------ */
/*  Device state                                                       */
/* ------------------------------------------------------------------ */

typedef struct ESP32S3I2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;
    Fifo8 rx_fifo;
    Fifo8 tx_fifo;
    bool trans_ongoing;

    /* Timing / config registers (store only, not functionally simulated) */
    uint32_t scl_low_period;
    uint32_t ctr;
    uint32_t timeout;
    uint32_t slave_addr;
    uint32_t fifo_conf;
    uint32_t sda_hold;
    uint32_t sda_sample;
    uint32_t scl_high_period;
    uint32_t scl_start_hold;
    uint32_t scl_rstart_setup;
    uint32_t scl_stop_hold;
    uint32_t scl_stop_setup;
    uint32_t filter_cfg;
    uint32_t clk_conf;
    uint32_t scl_st_time_out;
    uint32_t scl_main_st_time_out;
    uint32_t scl_sp_conf;
    uint32_t scl_stretch_conf;

    /* Interrupt registers */
    uint32_t int_raw;
    uint32_t int_ena;

    /* Command registers (8 for ESP32-S3) */
    uint32_t cmd[ESP32S3_I2C_CMD_COUNT];
} ESP32S3I2CState;

typedef struct ESP32S3I2CClass {
    SysBusDeviceClass parent_class;
} ESP32S3I2CClass;
