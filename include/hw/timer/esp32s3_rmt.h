/*
 * ESP32-S3 RMT (Remote Control Transceiver) peripheral
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_RMT "esp32s3.rmt"
#define ESP32S3_RMT(obj)           OBJECT_CHECK(ESP32S3RmtState, (obj), TYPE_ESP32S3_RMT)
#define ESP32S3_RMT_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3RmtClass, obj, TYPE_ESP32S3_RMT)
#define ESP32S3_RMT_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32S3RmtClass, klass, TYPE_ESP32S3_RMT)

/* ESP32-S3 RMT has 4 TX channels (CH0-CH3) and 4 RX channels (CH4-CH7) */
#define ESP32S3_RMT_TX_CHANNELS    4
#define ESP32S3_RMT_RX_CHANNELS    4
#define ESP32S3_RMT_TOTAL_CHANNELS 8

/* RAM: 48 x 32-bit entries per channel block, total 384 entries */
#define ESP32S3_RMT_RAM_WORDS_PER_BLOCK  48
#define ESP32S3_RMT_RAM_TOTAL_WORDS      (ESP32S3_RMT_RAM_WORDS_PER_BLOCK * ESP32S3_RMT_TOTAL_CHANNELS)

/* ---------- Register offsets ---------- */

/* TX channel data FIFO access (write-only from CPU side in FIFO mode) */
REG32(RMT_CH0DATA, 0x0000)
REG32(RMT_CH1DATA, 0x0004)
REG32(RMT_CH2DATA, 0x0008)
REG32(RMT_CH3DATA, 0x000C)

/* RX channel data FIFO access (read-only from CPU side in FIFO mode) */
REG32(RMT_RX_CH0DATA, 0x0010)
REG32(RMT_RX_CH1DATA, 0x0014)
REG32(RMT_RX_CH2DATA, 0x0018)
REG32(RMT_RX_CH3DATA, 0x001C)

/* TX channel config registers (2 per channel) */
REG32(RMT_CH0CONF0, 0x0020)
    FIELD(RMT_CH0CONF0, TX_START,          0,  1)
    FIELD(RMT_CH0CONF0, MEM_RD_RST,        1,  1)
    FIELD(RMT_CH0CONF0, APB_MEM_RST,       2,  1)
    FIELD(RMT_CH0CONF0, TX_CONTI_MODE,     3,  1)
    FIELD(RMT_CH0CONF0, MEM_TX_WRAP_EN,    4,  1)
    FIELD(RMT_CH0CONF0, IDLE_OUT_LVL,      5,  1)
    FIELD(RMT_CH0CONF0, IDLE_OUT_EN,       6,  1)
    FIELD(RMT_CH0CONF0, TX_STOP,           7,  1)
    FIELD(RMT_CH0CONF0, DIV_CNT,           8,  8)
    FIELD(RMT_CH0CONF0, MEM_SIZE,         16,  4)
    FIELD(RMT_CH0CONF0, CARRIER_EFF_EN,   20,  1)
    FIELD(RMT_CH0CONF0, CARRIER_EN,       21,  1)
    FIELD(RMT_CH0CONF0, CARRIER_OUT_LVL,  22,  1)
    FIELD(RMT_CH0CONF0, AFIFO_RST,        23,  1)
    FIELD(RMT_CH0CONF0, CONF_UPDATE,      24,  1)

/* TX CONF0 stride: 8 bytes per TX channel */
#define RMT_TX_CHn_CONF0(n) (A_RMT_CH0CONF0 + (n) * 8)
#define RMT_TX_CHn_CONF1(n) (A_RMT_CH0CONF0 + (n) * 8 + 4)

/* RX channel config registers (base offset 0x0040) */
REG32(RMT_CH4CONF0, 0x0040)
    FIELD(RMT_CH4CONF0, DIV_CNT,      0,  8)
    FIELD(RMT_CH4CONF0, IDLE_THRES,   8, 15)
    FIELD(RMT_CH4CONF0, MEM_SIZE,    23,  4)
    FIELD(RMT_CH4CONF0, CARRIER_EN,  27,  1)
    FIELD(RMT_CH4CONF0, CARRIER_OUT_LVL, 28, 1)

REG32(RMT_CH4CONF1, 0x0044)
    FIELD(RMT_CH4CONF1, RX_EN,            0,  1)
    FIELD(RMT_CH4CONF1, MEM_WR_RST,       1,  1)
    FIELD(RMT_CH4CONF1, APB_MEM_RST,      2,  1)
    FIELD(RMT_CH4CONF1, MEM_OWNER,        3,  1)
    FIELD(RMT_CH4CONF1, RX_FILTER_EN,     4,  1)
    FIELD(RMT_CH4CONF1, RX_FILTER_THRES,  5,  8)
    FIELD(RMT_CH4CONF1, MEM_RX_WRAP_EN,  13,  1)
    FIELD(RMT_CH4CONF1, AFIFO_RST,       14,  1)
    FIELD(RMT_CH4CONF1, CONF_UPDATE,     15,  1)

/* RX CONF stride: 8 bytes per RX channel */
#define RMT_RX_CHn_CONF0(n) (A_RMT_CH4CONF0 + (n) * 8)
#define RMT_RX_CHn_CONF1(n) (A_RMT_CH4CONF0 + (n) * 8 + 4)

/* Status registers */
REG32(RMT_CH0STATUS, 0x0060)
/* Stride: 4 bytes */
#define RMT_CHn_STATUS(n) (A_RMT_CH0STATUS + (n) * 4)

/* Interrupt registers */
REG32(RMT_INT_RAW, 0x0080)
REG32(RMT_INT_ST,  0x0084)
REG32(RMT_INT_ENA, 0x0088)
REG32(RMT_INT_CLR, 0x008C)

/* Interrupt bit positions */
#define RMT_INT_CH_TX_END(n)       (1 << (n))          /* bits [3:0] */
#define RMT_INT_CH_RX_END(n)       (1 << ((n) + 4))    /* bits [7:4] */
#define RMT_INT_CH_TX_ERR(n)       (1 << ((n) + 8))    /* bits [11:8] */
#define RMT_INT_CH_RX_ERR(n)       (1 << ((n) + 12))   /* bits [15:12] */
#define RMT_INT_CH_TX_THR(n)       (1 << ((n) + 16))   /* bits [19:16] */
#define RMT_INT_CH_TX_LOOP(n)      (1 << ((n) + 20))   /* bits [23:20] */

/* Carrier duty registers */
REG32(RMT_CH0CARRIER_DUTY, 0x0090)
#define RMT_CHn_CARRIER_DUTY(n) (A_RMT_CH0CARRIER_DUTY + (n) * 4)

REG32(RMT_RX_CH0CARRIER_RM, 0x00A0)
#define RMT_RX_CHn_CARRIER_RM(n) (A_RMT_RX_CH0CARRIER_RM + (n) * 4)

/* TX limit registers */
REG32(RMT_CH0_TX_LIM, 0x00B0)
#define RMT_CHn_TX_LIM(n) (A_RMT_CH0_TX_LIM + (n) * 4)

/* RX limit registers */
REG32(RMT_CH4_RX_LIM, 0x00C0)
#define RMT_RX_CHn_RX_LIM(n) (A_RMT_CH4_RX_LIM + (n) * 4)

/* System / global config */
REG32(RMT_SYS_CONF, 0x00D0)
    FIELD(RMT_SYS_CONF, APB_FIFO_MASK,     0,  1)
    FIELD(RMT_SYS_CONF, MEM_CLK_FORCE_ON,  1,  1)
    FIELD(RMT_SYS_CONF, MEM_FORCE_PD,      2,  1)
    FIELD(RMT_SYS_CONF, MEM_FORCE_PU,      3,  1)
    FIELD(RMT_SYS_CONF, SCLK_DIV_NUM,      4,  8)
    FIELD(RMT_SYS_CONF, SCLK_DIV_A,       12,  6)
    FIELD(RMT_SYS_CONF, SCLK_DIV_B,       18,  6)
    FIELD(RMT_SYS_CONF, SCLK_SEL,         24,  2)
    FIELD(RMT_SYS_CONF, SCLK_ACTIVE,      26,  1)
    FIELD(RMT_SYS_CONF, CLK_EN,           31,  1)

REG32(RMT_TX_SIM, 0x00D4)
REG32(RMT_REF_CNT_RST, 0x00D8)

/* Version register */
REG32(RMT_DATE, 0x00FC)

/* Register region size */
#define ESP32S3_RMT_REGS_SIZE      0x100

/* RAM accessible at offset 0x0400 when APB_FIFO_MASK=1 */
#define ESP32S3_RMT_RAM_BASE_OFF   0x0400
#define ESP32S3_RMT_RAM_SIZE       (ESP32S3_RMT_RAM_TOTAL_WORDS * 4)

/* Total MMIO region: registers + RAM */
#define ESP32S3_RMT_TOTAL_SIZE     (ESP32S3_RMT_RAM_BASE_OFF + ESP32S3_RMT_RAM_SIZE)

#define ESP32S3_RMT_DATE_VERSION   0x2108232


/* ---------- TX channel state ---------- */
typedef struct ESP32S3RmtTxChannel {
    uint32_t conf0;
    uint32_t conf1;      /* reserved for TX in S3 but register exists */
    uint32_t status;
    uint32_t carrier_duty;
    uint32_t tx_lim;
    /* FIFO read pointer */
    uint32_t rd_ptr;
    bool     active;
} ESP32S3RmtTxChannel;

/* ---------- RX channel state ---------- */
typedef struct ESP32S3RmtRxChannel {
    uint32_t conf0;
    uint32_t conf1;
    uint32_t status;
    uint32_t carrier_rm;
    uint32_t rx_lim;
    /* FIFO write pointer */
    uint32_t wr_ptr;
    bool     active;
} ESP32S3RmtRxChannel;


typedef struct ESP32S3RmtState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq     irq;

    /* TX channels 0-3 */
    ESP32S3RmtTxChannel tx_ch[ESP32S3_RMT_TX_CHANNELS];

    /* RX channels 0-3 (physical CH4-CH7) */
    ESP32S3RmtRxChannel rx_ch[ESP32S3_RMT_RX_CHANNELS];

    /* Interrupt registers */
    uint32_t int_raw;
    uint32_t int_ena;

    /* System/global config */
    uint32_t sys_conf;
    uint32_t tx_sim;
    uint32_t ref_cnt_rst;

    /* Date/version register */
    uint32_t date_reg;

    /* Channel RAM (48 words per channel × 8 channels = 384 words) */
    uint32_t ram[ESP32S3_RMT_RAM_TOTAL_WORDS];

    /* TX FIFO write pointers (for FIFO-mode access via CHnDATA regs) */
    uint32_t tx_fifo_wr_ptr[ESP32S3_RMT_TX_CHANNELS];

    /* RX FIFO read pointers (for FIFO-mode access via RX_CHnDATA regs) */
    uint32_t rx_fifo_rd_ptr[ESP32S3_RMT_RX_CHANNELS];

} ESP32S3RmtState;

typedef struct ESP32S3RmtClass {
    SysBusDeviceClass parent_class;
} ESP32S3RmtClass;
