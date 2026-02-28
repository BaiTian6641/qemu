/*
 * ESP32-S3 I2C Controller
 *
 * Master-mode I2C controller with command-based transaction engine,
 * 32-byte TX/RX FIFOs, and interrupt support.
 *
 * Register layout follows ESP32-S3 TRM v1.2 Chapter 28.
 * Key differences from ESP32: 8 command registers (vs 16), different
 * opcode encoding (RSTART=6, WRITE=1, READ=3, STOP=2, END=4),
 * CLK_CONF at 0x54, FILTER_CFG at 0x50, FIFO memory region 0x100-0x1FF.
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/i2c/esp32s3_i2c.h"
#include "hw/irq.h"

#define I2C_DEBUG 0

#if I2C_DEBUG
#define I2C_DPRINTF(fmt, ...) \
    qemu_log_mask(LOG_UNIMP, "esp32s3_i2c: " fmt, ## __VA_ARGS__)
#else
#define I2C_DPRINTF(fmt, ...) do {} while (0)
#endif

/* ================================================================== */
/*  IRQ and status helpers                                             */
/* ================================================================== */

static void esp32s3_i2c_update_irq(ESP32S3I2CState *s)
{
    int level = !!(s->int_raw & s->int_ena);
    qemu_set_irq(s->irq, level);
}

static uint32_t esp32s3_i2c_get_status(ESP32S3I2CState *s)
{
    uint32_t sr = 0;
    if (s->trans_ongoing) {
        sr |= (1 << I2C_S3_SR_BUS_BUSY_BIT);
    }
    sr |= (fifo8_num_used(&s->rx_fifo) & 0x3F) << I2C_S3_SR_RXFIFO_CNT_SHIFT;
    sr |= (fifo8_num_used(&s->tx_fifo) & 0x3F) << I2C_S3_SR_TXFIFO_CNT_SHIFT;
    return sr;
}

/* ================================================================== */
/*  Command-based transaction engine                                   */
/* ================================================================== */

static void esp32s3_i2c_do_transaction(ESP32S3I2CState *s)
{
    bool stop_or_end = false;

    for (int i = 0; i < ESP32S3_I2C_CMD_COUNT && !stop_or_end; i++) {
        uint32_t cmd = s->cmd[i];
        uint32_t opcode = (cmd & I2C_S3_CMD_OPCODE_MASK) >> I2C_S3_CMD_OPCODE_SHIFT;
        uint32_t byte_num = (cmd & I2C_S3_CMD_BYTE_NUM_MASK);

        switch (opcode) {
        case I2C_S3_OPCODE_RSTART:
            I2C_DPRINTF("cmd[%d] RSTART\n", i);
            if (s->trans_ongoing) {
                i2c_end_transfer(s->bus);
                s->trans_ongoing = false;
            }
            break;

        case I2C_S3_OPCODE_WRITE: {
            I2C_DPRINTF("cmd[%d] WRITE byte_num=%u\n", i, byte_num);
            size_t length = byte_num;
            if (!s->trans_ongoing) {
                s->trans_ongoing = true;
                if (fifo8_num_used(&s->tx_fifo) == 0) {
                    I2C_DPRINTF("WRITE: TX FIFO empty\n");
                    s->int_raw |= I2C_S3_INT_MST_TXFIFO_UDF;
                    stop_or_end = true;
                    break;
                }
                uint8_t data = fifo8_pop(&s->tx_fifo);
                uint8_t addr = data >> 1;
                uint8_t is_read = data & 1;
                if (i2c_start_transfer(s->bus, addr, is_read) != 0) {
                    /* NACK */
                    if (cmd & I2C_S3_CMD_ACK_CHECK_EN) {
                        s->int_raw |= I2C_S3_INT_NACK;
                        stop_or_end = true;
                    }
                    s->trans_ongoing = false;
                    break;
                }
                length -= 1;
            }
            for (size_t n = 0; n < length; n++) {
                if (fifo8_num_used(&s->tx_fifo) == 0) {
                    I2C_DPRINTF("WRITE: TX FIFO underflow\n");
                    s->int_raw |= I2C_S3_INT_MST_TXFIFO_UDF;
                    break;
                }
                uint8_t data = fifo8_pop(&s->tx_fifo);
                if (i2c_send(s->bus, data) != 0) {
                    if (cmd & I2C_S3_CMD_ACK_CHECK_EN) {
                        s->int_raw |= I2C_S3_INT_NACK;
                        stop_or_end = true;
                        break;
                    }
                }
            }
            break;
        }

        case I2C_S3_OPCODE_READ: {
            I2C_DPRINTF("cmd[%d] READ byte_num=%u\n", i, byte_num);
            for (size_t n = 0; n < byte_num; n++) {
                if (fifo8_num_free(&s->rx_fifo) == 0) {
                    I2C_DPRINTF("READ: RX FIFO overflow\n");
                    s->int_raw |= I2C_S3_INT_RXFIFO_OVF;
                    break;
                }
                uint8_t data = i2c_recv(s->bus);
                fifo8_push(&s->rx_fifo, data);
                /* Send ACK/NACK for last byte based on ACK_VAL */
                if (n == byte_num - 1 && (cmd & I2C_S3_CMD_ACK_VAL)) {
                    i2c_nack(s->bus);
                }
            }
            break;
        }

        case I2C_S3_OPCODE_STOP:
            I2C_DPRINTF("cmd[%d] STOP\n", i);
            if (s->trans_ongoing) {
                i2c_end_transfer(s->bus);
                s->trans_ongoing = false;
            }
            s->int_raw |= I2C_S3_INT_TRANS_COMPLETE;
            stop_or_end = true;
            break;

        case I2C_S3_OPCODE_END:
            I2C_DPRINTF("cmd[%d] END\n", i);
            s->int_raw |= I2C_S3_INT_END_DETECT;
            stop_or_end = true;
            break;

        default:
            I2C_DPRINTF("cmd[%d] unknown opcode %u\n", i, opcode);
            break;
        }

        /* Mark command as done */
        s->cmd[i] |= I2C_S3_CMD_DONE;
    }

    s->int_raw |= I2C_S3_INT_TRANS_START;
    esp32s3_i2c_update_irq(s);
}

/* ================================================================== */
/*  MMIO read/write                                                    */
/* ================================================================== */

static uint64_t esp32s3_i2c_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3I2CState *s = ESP32S3_I2C(opaque);

    switch (addr) {
    case ESP32S3_I2C_SCL_LOW_PERIOD_OFF:
        return s->scl_low_period;
    case ESP32S3_I2C_CTR_OFF:
        return s->ctr;
    case ESP32S3_I2C_SR_OFF:
        return esp32s3_i2c_get_status(s);
    case ESP32S3_I2C_TO_OFF:
        return s->timeout;
    case ESP32S3_I2C_SLAVE_ADDR_OFF:
        return s->slave_addr;
    case ESP32S3_I2C_FIFO_ST_OFF: {
        uint32_t st = 0;
        st |= (fifo8_num_used(&s->rx_fifo) & 0x3F);      /* rx_fifo_raddr */
        st |= (fifo8_num_used(&s->tx_fifo) & 0x3F) << 15; /* tx_fifo_raddr */
        return st;
    }
    case ESP32S3_I2C_FIFO_CONF_OFF:
        return s->fifo_conf;
    case ESP32S3_I2C_DATA_OFF:
        if (fifo8_num_used(&s->rx_fifo) == 0) {
            I2C_DPRINTF("read DATA: RX FIFO empty\n");
            return 0xFF;
        }
        return fifo8_pop(&s->rx_fifo);
    case ESP32S3_I2C_INT_RAW_OFF:
        return s->int_raw;
    case ESP32S3_I2C_INT_CLR_OFF:
        return 0;  /* write-only */
    case ESP32S3_I2C_INT_ENA_OFF:
        return s->int_ena;
    case ESP32S3_I2C_INT_ST_OFF:
        return s->int_raw & s->int_ena;
    case ESP32S3_I2C_SDA_HOLD_OFF:
        return s->sda_hold;
    case ESP32S3_I2C_SDA_SAMPLE_OFF:
        return s->sda_sample;
    case ESP32S3_I2C_SCL_HIGH_PERIOD_OFF:
        return s->scl_high_period;
    case ESP32S3_I2C_SCL_START_HOLD_OFF:
        return s->scl_start_hold;
    case ESP32S3_I2C_SCL_RSTART_SETUP_OFF:
        return s->scl_rstart_setup;
    case ESP32S3_I2C_SCL_STOP_HOLD_OFF:
        return s->scl_stop_hold;
    case ESP32S3_I2C_SCL_STOP_SETUP_OFF:
        return s->scl_stop_setup;
    case ESP32S3_I2C_FILTER_CFG_OFF:
        return s->filter_cfg;
    case ESP32S3_I2C_CLK_CONF_OFF:
        return s->clk_conf;
    case ESP32S3_I2C_COMD0_OFF ... (ESP32S3_I2C_COMD0_OFF + (ESP32S3_I2C_CMD_COUNT - 1) * 4): {
        int idx = (addr - ESP32S3_I2C_COMD0_OFF) / 4;
        return s->cmd[idx];
    }
    case ESP32S3_I2C_SCL_ST_TIME_OUT_OFF:
        return s->scl_st_time_out;
    case ESP32S3_I2C_SCL_MAIN_ST_TIME_OUT_OFF:
        return s->scl_main_st_time_out;
    case ESP32S3_I2C_SCL_SP_CONF_OFF:
        return s->scl_sp_conf;
    case ESP32S3_I2C_SCL_STRETCH_CONF_OFF:
        return s->scl_stretch_conf;
    case ESP32S3_I2C_DATE_OFF:
        return 0x20200616;  /* VERSION register */
    case ESP32S3_I2C_RXFIFO_START_OFF ... (ESP32S3_I2C_RXFIFO_START_OFF + 0x7F):
        /* Memory-mapped RX FIFO read */
        if (fifo8_num_used(&s->rx_fifo) == 0) {
            return 0xFF;
        }
        return fifo8_pop(&s->rx_fifo);
    default:
        I2C_DPRINTF("read: unhandled addr 0x%03" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void esp32s3_i2c_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    ESP32S3I2CState *s = ESP32S3_I2C(opaque);

    switch (addr) {
    case ESP32S3_I2C_SCL_LOW_PERIOD_OFF:
        s->scl_low_period = value;
        break;
    case ESP32S3_I2C_CTR_OFF:
        s->ctr = value;
        if (value & (1 << I2C_S3_CTR_TRANS_START_BIT)) {
            esp32s3_i2c_do_transaction(s);
            s->ctr &= ~(1 << I2C_S3_CTR_TRANS_START_BIT);
        }
        break;
    case ESP32S3_I2C_TO_OFF:
        s->timeout = value;
        break;
    case ESP32S3_I2C_SLAVE_ADDR_OFF:
        s->slave_addr = value;
        break;
    case ESP32S3_I2C_FIFO_CONF_OFF:
        s->fifo_conf = value;
        if (value & (1 << I2C_S3_FIFO_CONF_RX_FIFO_RST_BIT)) {
            fifo8_reset(&s->rx_fifo);
        }
        if (value & (1 << I2C_S3_FIFO_CONF_TX_FIFO_RST_BIT)) {
            fifo8_reset(&s->tx_fifo);
        }
        break;
    case ESP32S3_I2C_DATA_OFF:
        if (fifo8_num_free(&s->tx_fifo) == 0) {
            I2C_DPRINTF("write DATA: TX FIFO full\n");
            s->int_raw |= I2C_S3_INT_TXFIFO_OVF;
        } else {
            fifo8_push(&s->tx_fifo, (uint8_t)value);
        }
        break;
    case ESP32S3_I2C_INT_CLR_OFF:
        s->int_raw &= ~(uint32_t)value;
        esp32s3_i2c_update_irq(s);
        break;
    case ESP32S3_I2C_INT_ENA_OFF:
        s->int_ena = value;
        esp32s3_i2c_update_irq(s);
        break;
    case ESP32S3_I2C_SDA_HOLD_OFF:
        s->sda_hold = value;
        break;
    case ESP32S3_I2C_SDA_SAMPLE_OFF:
        s->sda_sample = value;
        break;
    case ESP32S3_I2C_SCL_HIGH_PERIOD_OFF:
        s->scl_high_period = value;
        break;
    case ESP32S3_I2C_SCL_START_HOLD_OFF:
        s->scl_start_hold = value;
        break;
    case ESP32S3_I2C_SCL_RSTART_SETUP_OFF:
        s->scl_rstart_setup = value;
        break;
    case ESP32S3_I2C_SCL_STOP_HOLD_OFF:
        s->scl_stop_hold = value;
        break;
    case ESP32S3_I2C_SCL_STOP_SETUP_OFF:
        s->scl_stop_setup = value;
        break;
    case ESP32S3_I2C_FILTER_CFG_OFF:
        s->filter_cfg = value;
        break;
    case ESP32S3_I2C_CLK_CONF_OFF:
        s->clk_conf = value;
        break;
    case ESP32S3_I2C_COMD0_OFF ... (ESP32S3_I2C_COMD0_OFF + (ESP32S3_I2C_CMD_COUNT - 1) * 4): {
        int idx = (addr - ESP32S3_I2C_COMD0_OFF) / 4;
        s->cmd[idx] = value;
        break;
    }
    case ESP32S3_I2C_SCL_ST_TIME_OUT_OFF:
        s->scl_st_time_out = value;
        break;
    case ESP32S3_I2C_SCL_MAIN_ST_TIME_OUT_OFF:
        s->scl_main_st_time_out = value;
        break;
    case ESP32S3_I2C_SCL_SP_CONF_OFF:
        s->scl_sp_conf = value;
        break;
    case ESP32S3_I2C_SCL_STRETCH_CONF_OFF:
        s->scl_stretch_conf = value;
        break;
    case ESP32S3_I2C_TXFIFO_START_OFF ... (ESP32S3_I2C_TXFIFO_START_OFF + 0x7F):
        /* Memory-mapped TX FIFO write */
        if (fifo8_num_free(&s->tx_fifo) == 0) {
            I2C_DPRINTF("write TXFIFO: full\n");
        } else {
            fifo8_push(&s->tx_fifo, (uint8_t)value);
        }
        break;
    default:
        I2C_DPRINTF("write: unhandled addr 0x%03" HWADDR_PRIx " = 0x%08" PRIx64 "\n",
                    addr, value);
        break;
    }
}

/* ================================================================== */
/*  Lifecycle                                                          */
/* ================================================================== */

static const MemoryRegionOps esp32s3_i2c_ops = {
    .read  = esp32s3_i2c_read,
    .write = esp32s3_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_i2c_reset_hold(Object *obj, ResetType type)
{
    ESP32S3I2CState *s = ESP32S3_I2C(obj);

    fifo8_reset(&s->rx_fifo);
    fifo8_reset(&s->tx_fifo);
    s->trans_ongoing = false;

    s->scl_low_period = 0;
    s->ctr = 0;
    s->timeout = 0;
    s->slave_addr = 0;
    s->fifo_conf = 0;
    s->sda_hold = 0;
    s->sda_sample = 0;
    s->scl_high_period = 0;
    s->scl_start_hold = 0;
    s->scl_rstart_setup = 0;
    s->scl_stop_hold = 0;
    s->scl_stop_setup = 0;
    s->filter_cfg = 0x0B;  /* Default: SCL filter enabled, SDA filter enabled */
    s->clk_conf = 0;
    s->scl_st_time_out = 0;
    s->scl_main_st_time_out = 0;
    s->scl_sp_conf = 0;
    s->scl_stretch_conf = 0;

    s->int_raw = 0;
    s->int_ena = 0;

    memset(s->cmd, 0, sizeof(s->cmd));
}

static void esp32s3_i2c_realize(DeviceState *dev, Error **errp)
{
    esp32s3_i2c_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}

static void esp32s3_i2c_init(Object *obj)
{
    ESP32S3I2CState *s = ESP32S3_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_i2c_ops, s,
                          TYPE_ESP32S3_I2C, ESP32S3_I2C_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->bus = i2c_init_bus(DEVICE(s), "i2c");

    fifo8_create(&s->tx_fifo, ESP32S3_I2C_FIFO_LEN);
    fifo8_create(&s->rx_fifo, ESP32S3_I2C_FIFO_LEN);
}

static void esp32s3_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_i2c_reset_hold;
    dc->realize = esp32s3_i2c_realize;
}

static const TypeInfo esp32s3_i2c_info = {
    .name          = TYPE_ESP32S3_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3I2CState),
    .instance_init = esp32s3_i2c_init,
    .class_init    = esp32s3_i2c_class_init,
    .class_size    = sizeof(ESP32S3I2CClass),
};

static void esp32s3_i2c_register_types(void)
{
    type_register_static(&esp32s3_i2c_info);
}

type_init(esp32s3_i2c_register_types)
