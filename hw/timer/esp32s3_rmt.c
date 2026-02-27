/*
 * ESP32-S3 RMT (Remote Control Transceiver) peripheral
 *
 * Implements the RMT register model for ESP32-S3. This provides:
 * - 4 TX channels (CH0-CH3) and 4 RX channels (CH4-CH7)
 * - Per-channel configuration, status, and interrupt registers
 * - Channel RAM accessible via FIFO or direct APB access
 * - Basic TX start/end interrupt generation
 *
 * Limitations (MVP):
 * - No real waveform generation or capture timing
 * - TX completion is simulated immediately on TX_START
 * - RX is stubbed (registers accept writes but no real capture)
 * - Carrier modulation/demodulation not implemented
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/timer/esp32s3_rmt.h"
#include "trace.h"

#define RMT_WARNING 0

/* ---------- Interrupt management ---------- */

static void esp32s3_rmt_update_irq(ESP32S3RmtState *s)
{
    uint32_t pending = s->int_raw & s->int_ena;
    trace_esp32s3_rmt_irq_update(s->int_raw, s->int_ena, pending);
    qemu_set_irq(s->irq, pending ? 1 : 0);
}

static void esp32s3_rmt_set_int(ESP32S3RmtState *s, uint32_t mask)
{
    s->int_raw |= mask;
    esp32s3_rmt_update_irq(s);
}

/* ---------- TX simulation ---------- */

/*
 * MVP TX behavior: When TX_START is written, we immediately mark the
 * transmission as complete and raise the TX_END interrupt. This allows
 * ESP-IDF drivers to proceed through the normal TX flow without hanging.
 *
 * A more detailed model would use a QEMUTimer to simulate actual
 * waveform timing based on DIV_CNT and the RAM data.
 */
static void esp32s3_rmt_tx_start(ESP32S3RmtState *s, int ch)
{
    ESP32S3RmtTxChannel *tx = &s->tx_ch[ch];

    trace_esp32s3_rmt_tx_start(ch);

    /* Mark TX as started then immediately completed */
    tx->active = true;

    /* Scan RAM for end marker (entry with duration0=0 or level+duration = 0).
     * For MVP we just signal TX_END immediately. */
    tx->active = false;

    /* Raise TX_END interrupt */
    trace_esp32s3_rmt_tx_end(ch);
    esp32s3_rmt_set_int(s, RMT_INT_CH_TX_END(ch));
}

/* ---------- Register read ---------- */

static uint64_t esp32s3_rmt_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3RmtState *s = ESP32S3_RMT(opaque);
    uint64_t r = 0;

    /* RAM region (0x0400 .. 0x09FF) */
    if (addr >= ESP32S3_RMT_RAM_BASE_OFF &&
        addr < ESP32S3_RMT_RAM_BASE_OFF + ESP32S3_RMT_RAM_SIZE) {
        uint32_t word_idx = (addr - ESP32S3_RMT_RAM_BASE_OFF) / 4;
        if (word_idx < ESP32S3_RMT_RAM_TOTAL_WORDS) {
            return s->ram[word_idx];
        }
        return 0;
    }

    switch (addr) {
    /* TX FIFO data read (generally write-only from CPU, but return 0) */
    case A_RMT_CH0DATA ... A_RMT_CH3DATA:
        r = 0;
        break;

    /* RX FIFO data read */
    case A_RMT_RX_CH0DATA ... A_RMT_RX_CH3DATA: {
        int rx_ch = (addr - A_RMT_RX_CH0DATA) / 4;
        /* In FIFO mode, return data from RX channel RAM at current read pointer */
        if (!FIELD_EX32(s->sys_conf, RMT_SYS_CONF, APB_FIFO_MASK)) {
            uint32_t base = (ESP32S3_RMT_TX_CHANNELS + rx_ch) * ESP32S3_RMT_RAM_WORDS_PER_BLOCK;
            uint32_t ptr = s->rx_fifo_rd_ptr[rx_ch];
            if (ptr < ESP32S3_RMT_RAM_WORDS_PER_BLOCK) {
                r = s->ram[base + ptr];
                s->rx_fifo_rd_ptr[rx_ch] = ptr + 1;
            }
        }
        break;
    }

    /* TX channel CONF0 */
    case RMT_TX_CHn_CONF0(0) ... RMT_TX_CHn_CONF0(3): {
        int ch = (addr - RMT_TX_CHn_CONF0(0)) / 8;
        if ((addr & 0x7) == 0) {
            r = s->tx_ch[ch].conf0;
        } else {
            r = s->tx_ch[ch].conf1;
        }
        break;
    }

    /* RX channel CONF0/CONF1 */
    case RMT_RX_CHn_CONF0(0) ... RMT_RX_CHn_CONF1(3): {
        int ch = (addr - RMT_RX_CHn_CONF0(0)) / 8;
        if ((addr & 0x7) == 0) {
            r = s->rx_ch[ch].conf0;
        } else {
            r = s->rx_ch[ch].conf1;
        }
        break;
    }

    /* Status registers (TX CH0-CH3, RX CH4-CH7) */
    case RMT_CHn_STATUS(0) ... RMT_CHn_STATUS(7): {
        int ch = (addr - A_RMT_CH0STATUS) / 4;
        if (ch < ESP32S3_RMT_TX_CHANNELS) {
            r = s->tx_ch[ch].status;
        } else {
            r = s->rx_ch[ch - ESP32S3_RMT_TX_CHANNELS].status;
        }
        break;
    }

    /* Interrupt registers */
    case A_RMT_INT_RAW:
        r = s->int_raw;
        break;
    case A_RMT_INT_ST:
        r = s->int_raw & s->int_ena;
        break;
    case A_RMT_INT_ENA:
        r = s->int_ena;
        break;
    case A_RMT_INT_CLR:
        r = 0; /* Write-only */
        break;

    /* Carrier duty (TX) */
    case RMT_CHn_CARRIER_DUTY(0) ... RMT_CHn_CARRIER_DUTY(3): {
        int ch = (addr - A_RMT_CH0CARRIER_DUTY) / 4;
        r = s->tx_ch[ch].carrier_duty;
        break;
    }

    /* Carrier remove (RX) */
    case RMT_RX_CHn_CARRIER_RM(0) ... RMT_RX_CHn_CARRIER_RM(3): {
        int ch = (addr - A_RMT_RX_CH0CARRIER_RM) / 4;
        r = s->rx_ch[ch].carrier_rm;
        break;
    }

    /* TX limit */
    case RMT_CHn_TX_LIM(0) ... RMT_CHn_TX_LIM(3): {
        int ch = (addr - A_RMT_CH0_TX_LIM) / 4;
        r = s->tx_ch[ch].tx_lim;
        break;
    }

    /* RX limit */
    case RMT_RX_CHn_RX_LIM(0) ... RMT_RX_CHn_RX_LIM(3): {
        int ch = (addr - A_RMT_CH4_RX_LIM) / 4;
        r = s->rx_ch[ch].rx_lim;
        break;
    }

    /* System config */
    case A_RMT_SYS_CONF:
        r = s->sys_conf;
        break;
    case A_RMT_TX_SIM:
        r = s->tx_sim;
        break;
    case A_RMT_REF_CNT_RST:
        r = s->ref_cnt_rst;
        break;
    case A_RMT_DATE:
        r = s->date_reg;
        break;

    default:
#if RMT_WARNING
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read at offset 0x%04" HWADDR_PRIx "\n",
                      __func__, addr);
#endif
        break;
    }

    return r;
}

/* ---------- Register write ---------- */

static void esp32s3_rmt_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned int size)
{
    ESP32S3RmtState *s = ESP32S3_RMT(opaque);

    /* RAM region (0x0400 .. 0x09FF) */
    if (addr >= ESP32S3_RMT_RAM_BASE_OFF &&
        addr < ESP32S3_RMT_RAM_BASE_OFF + ESP32S3_RMT_RAM_SIZE) {
        uint32_t word_idx = (addr - ESP32S3_RMT_RAM_BASE_OFF) / 4;
        if (word_idx < ESP32S3_RMT_RAM_TOTAL_WORDS) {
            s->ram[word_idx] = (uint32_t)value;
        }
        return;
    }

    switch (addr) {
    /* TX FIFO data write */
    case A_RMT_CH0DATA ... A_RMT_CH3DATA: {
        int ch = (addr - A_RMT_CH0DATA) / 4;
        /* In FIFO mode, write to TX channel RAM at current write pointer */
        if (!FIELD_EX32(s->sys_conf, RMT_SYS_CONF, APB_FIFO_MASK)) {
            uint32_t base = ch * ESP32S3_RMT_RAM_WORDS_PER_BLOCK;
            uint32_t ptr = s->tx_fifo_wr_ptr[ch];
            if (ptr < ESP32S3_RMT_RAM_WORDS_PER_BLOCK) {
                s->ram[base + ptr] = (uint32_t)value;
                s->tx_fifo_wr_ptr[ch] = ptr + 1;
            }
        }
        break;
    }

    /* RX FIFO data - generally read-only from CPU side */
    case A_RMT_RX_CH0DATA ... A_RMT_RX_CH3DATA:
        break;

    /* TX channel CONF0/CONF1 */
    case RMT_TX_CHn_CONF0(0) ... RMT_TX_CHn_CONF1(3): {
        int ch = (addr - RMT_TX_CHn_CONF0(0)) / 8;
        if ((addr & 0x7) == 0) {
            /* CONF0 */
            uint32_t old_conf0 = s->tx_ch[ch].conf0;
            s->tx_ch[ch].conf0 = (uint32_t)value;

            /* Handle self-clearing bits */
            if (value & R_RMT_CH0CONF0_MEM_RD_RST_MASK) {
                s->tx_ch[ch].rd_ptr = 0;
                s->tx_ch[ch].conf0 &= ~R_RMT_CH0CONF0_MEM_RD_RST_MASK;
            }
            if (value & R_RMT_CH0CONF0_APB_MEM_RST_MASK) {
                s->tx_fifo_wr_ptr[ch] = 0;
                s->tx_ch[ch].conf0 &= ~R_RMT_CH0CONF0_APB_MEM_RST_MASK;
            }
            if (value & R_RMT_CH0CONF0_AFIFO_RST_MASK) {
                s->tx_ch[ch].conf0 &= ~R_RMT_CH0CONF0_AFIFO_RST_MASK;
            }
            if (value & R_RMT_CH0CONF0_CONF_UPDATE_MASK) {
                /* Config update acknowledged */
                s->tx_ch[ch].conf0 &= ~R_RMT_CH0CONF0_CONF_UPDATE_MASK;
            }
            if (value & R_RMT_CH0CONF0_TX_STOP_MASK) {
                s->tx_ch[ch].active = false;
                s->tx_ch[ch].conf0 &= ~R_RMT_CH0CONF0_TX_STOP_MASK;
            }

            /* TX_START trigger */
            if ((value & R_RMT_CH0CONF0_TX_START_MASK) &&
                !(old_conf0 & R_RMT_CH0CONF0_TX_START_MASK)) {
                s->tx_ch[ch].conf0 &= ~R_RMT_CH0CONF0_TX_START_MASK;
                esp32s3_rmt_tx_start(s, ch);
            }
        } else {
            /* CONF1 (reserved for TX in ESP32-S3 but writable) */
            s->tx_ch[ch].conf1 = (uint32_t)value;
        }
        break;
    }

    /* RX channel CONF0/CONF1 */
    case RMT_RX_CHn_CONF0(0) ... RMT_RX_CHn_CONF1(3): {
        int ch = (addr - RMT_RX_CHn_CONF0(0)) / 8;
        if ((addr & 0x7) == 0) {
            s->rx_ch[ch].conf0 = (uint32_t)value;
        } else {
            /* CONF1 */
            s->rx_ch[ch].conf1 = (uint32_t)value;

            /* Handle self-clearing bits */
            if (value & R_RMT_CH4CONF1_MEM_WR_RST_MASK) {
                s->rx_ch[ch].wr_ptr = 0;
                s->rx_ch[ch].conf1 &= ~R_RMT_CH4CONF1_MEM_WR_RST_MASK;
            }
            if (value & R_RMT_CH4CONF1_APB_MEM_RST_MASK) {
                s->rx_fifo_rd_ptr[ch] = 0;
                s->rx_ch[ch].conf1 &= ~R_RMT_CH4CONF1_APB_MEM_RST_MASK;
            }
            if (value & R_RMT_CH4CONF1_AFIFO_RST_MASK) {
                s->rx_ch[ch].conf1 &= ~R_RMT_CH4CONF1_AFIFO_RST_MASK;
            }
            if (value & R_RMT_CH4CONF1_CONF_UPDATE_MASK) {
                s->rx_ch[ch].conf1 &= ~R_RMT_CH4CONF1_CONF_UPDATE_MASK;
            }

            /* RX_EN */
            if (value & R_RMT_CH4CONF1_RX_EN_MASK) {
                s->rx_ch[ch].active = true;
            } else {
                s->rx_ch[ch].active = false;
            }
        }
        break;
    }

    /* Interrupt registers */
    case A_RMT_INT_RAW:
        /* RAW bits can be set by software in some implementations */
        s->int_raw |= (uint32_t)value;
        esp32s3_rmt_update_irq(s);
        break;
    case A_RMT_INT_ENA:
        s->int_ena = (uint32_t)value;
        esp32s3_rmt_update_irq(s);
        break;
    case A_RMT_INT_CLR:
        s->int_raw &= ~(uint32_t)value;
        esp32s3_rmt_update_irq(s);
        break;

    /* Carrier duty (TX) */
    case RMT_CHn_CARRIER_DUTY(0) ... RMT_CHn_CARRIER_DUTY(3): {
        int ch = (addr - A_RMT_CH0CARRIER_DUTY) / 4;
        s->tx_ch[ch].carrier_duty = (uint32_t)value;
        break;
    }

    /* Carrier remove (RX) */
    case RMT_RX_CHn_CARRIER_RM(0) ... RMT_RX_CHn_CARRIER_RM(3): {
        int ch = (addr - A_RMT_RX_CH0CARRIER_RM) / 4;
        s->rx_ch[ch].carrier_rm = (uint32_t)value;
        break;
    }

    /* TX limit */
    case RMT_CHn_TX_LIM(0) ... RMT_CHn_TX_LIM(3): {
        int ch = (addr - A_RMT_CH0_TX_LIM) / 4;
        s->tx_ch[ch].tx_lim = (uint32_t)value;
        break;
    }

    /* RX limit */
    case RMT_RX_CHn_RX_LIM(0) ... RMT_RX_CHn_RX_LIM(3): {
        int ch = (addr - A_RMT_CH4_RX_LIM) / 4;
        s->rx_ch[ch].rx_lim = (uint32_t)value;
        break;
    }

    /* System config */
    case A_RMT_SYS_CONF:
        s->sys_conf = (uint32_t)value;
        break;
    case A_RMT_TX_SIM:
        s->tx_sim = (uint32_t)value;
        break;
    case A_RMT_REF_CNT_RST:
        s->ref_cnt_rst = (uint32_t)value;
        break;
    case A_RMT_DATE:
        s->date_reg = (uint32_t)value;
        break;

    default:
#if RMT_WARNING
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write at offset 0x%04" HWADDR_PRIx "\n",
                      __func__, addr);
#endif
        break;
    }
}

static const MemoryRegionOps esp32s3_rmt_ops = {
    .read = esp32s3_rmt_read,
    .write = esp32s3_rmt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32s3_rmt_reset_hold(Object *obj, ResetType type)
{
    ESP32S3RmtState *s = ESP32S3_RMT(obj);

    /* Clear all interrupt state */
    s->int_raw = 0;
    s->int_ena = 0;

    /* System config defaults: SCLK_ACTIVE=1, MEM_FORCE_PU=1 */
    s->sys_conf = (1 << 26) | (1 << 3);  /* SCLK_ACTIVE=1, MEM_FORCE_PU=1 */
    s->tx_sim = 0;
    s->ref_cnt_rst = 0;
    s->date_reg = ESP32S3_RMT_DATE_VERSION;

    /* Reset TX channels */
    for (int i = 0; i < ESP32S3_RMT_TX_CHANNELS; i++) {
        memset(&s->tx_ch[i], 0, sizeof(s->tx_ch[i]));
        /* Default: MEM_SIZE=1, DIV_CNT=2 */
        s->tx_ch[i].conf0 = (1 << 16) | (2 << 8);
        s->tx_fifo_wr_ptr[i] = 0;
    }

    /* Reset RX channels */
    for (int i = 0; i < ESP32S3_RMT_RX_CHANNELS; i++) {
        memset(&s->rx_ch[i], 0, sizeof(s->rx_ch[i]));
        /* Default: MEM_SIZE=1, IDLE_THRES=0x7FFF, DIV_CNT=2 */
        s->rx_ch[i].conf0 = (1 << 23) | (0x7FFF << 8) | 2;
        s->rx_fifo_rd_ptr[i] = 0;
    }

    /* Clear RAM */
    memset(s->ram, 0, sizeof(s->ram));

    esp32s3_rmt_update_irq(s);
}

static void esp32s3_rmt_init(Object *obj)
{
    ESP32S3RmtState *s = ESP32S3_RMT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_rmt_ops, s,
                          TYPE_ESP32S3_RMT, ESP32S3_RMT_TOTAL_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_rmt_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_rmt_reset_hold;
}

static const TypeInfo esp32s3_rmt_info = {
    .name = TYPE_ESP32S3_RMT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3RmtState),
    .instance_init = esp32s3_rmt_init,
    .class_init = esp32s3_rmt_class_init,
    .class_size = sizeof(ESP32S3RmtClass),
};

static void esp32s3_rmt_register_types(void)
{
    type_register_static(&esp32s3_rmt_info);
}

type_init(esp32s3_rmt_register_types);
