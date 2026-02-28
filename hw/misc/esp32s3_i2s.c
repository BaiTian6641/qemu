/*
 * ESP32-S3 I2S Controller Model
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Sprint S7: R/W register store with interrupt model.
 * MVP: TX immediate-completion via self-clearing TX_START bit.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/dma/esp_gdma.h"
#include "hw/misc/esp32s3_i2s.h"

#define I2S_DMA_KICK_BYTES 256

static void esp32s3_i2s_try_dma(ESP32S3I2SState *s, bool tx)
{
    uint32_t chan;
    uint8_t buffer[I2S_DMA_KICK_BYTES] = { 0 };

    if (!s->gdma) {
        return;
    }

    if (tx) {
        if (!esp_gdma_get_channel_periph(s->gdma, s->gdma_periph,
                                         ESP_GDMA_OUT_IDX, &chan)) {
            return;
        }
        (void)esp_gdma_read_channel(s->gdma, chan, buffer, sizeof(buffer));
    } else {
        if (!esp_gdma_get_channel_periph(s->gdma, s->gdma_periph,
                                         ESP_GDMA_IN_IDX, &chan)) {
            return;
        }
        (void)esp_gdma_write_channel(s->gdma, chan, buffer, sizeof(buffer));
    }
}

static void esp32s3_i2s_update_irq(ESP32S3I2SState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) ? 1 : 0);
}

static uint64_t esp32s3_i2s_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3I2SState *s = ESP32S3_I2S(opaque);

    switch (addr) {
    case I2S_INT_RAW_REG:
        return s->int_raw;
    case I2S_INT_ST_REG:
        return s->int_raw & s->int_ena;
    case I2S_INT_ENA_REG:
        return s->int_ena;
    case I2S_STATE_REG:
        /* TX idle = 1 (bit 0) */
        return 0x01;
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_I2S_REGS_COUNT) {
            return s->regs[idx];
        }
        return 0;
    }
    }
}

static void esp32s3_i2s_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    ESP32S3I2SState *s = ESP32S3_I2S(opaque);

    switch (addr) {
    case I2S_INT_CLR_REG:
        s->int_raw &= ~(uint32_t)value;
        esp32s3_i2s_update_irq(s);
        return;
    case I2S_INT_ENA_REG:
        s->int_ena = (uint32_t)value;
        esp32s3_i2s_update_irq(s);
        return;
    case I2S_TX_CONF_REG: {
        uint32_t val = (uint32_t)value;
        /* Self-clearing bits */
        if (val & I2S_TX_RESET) {
            val &= ~I2S_TX_RESET;
        }
        if (val & I2S_TX_FIFO_RESET) {
            val &= ~I2S_TX_FIFO_RESET;
        }
        if (val & I2S_TX_START) {
            /* Kick GDMA channel if one is assigned to this I2S instance */
            esp32s3_i2s_try_dma(s, true);

            /* MVP: immediate TX completion */
            s->int_raw |= I2S_INT_TX_DONE;
            esp32s3_i2s_update_irq(s);
            val &= ~I2S_TX_START;
        }
        if (val & I2S_TX_UPDATE) {
            val &= ~I2S_TX_UPDATE;
        }
        s->regs[addr / 4] = val;
        return;
    }
    case I2S_RX_CONF_REG: {
        uint32_t val = (uint32_t)value;
        if (val & I2S_RX_RESET) {
            val &= ~I2S_RX_RESET;
        }
        if (val & I2S_RX_FIFO_RESET) {
            val &= ~I2S_RX_FIFO_RESET;
        }
        if (val & I2S_RX_START) {
            esp32s3_i2s_try_dma(s, false);
            s->int_raw |= I2S_INT_RX_DONE;
            esp32s3_i2s_update_irq(s);
            val &= ~I2S_RX_START;
        }
        if (val & I2S_RX_UPDATE) {
            val &= ~I2S_RX_UPDATE;
        }
        s->regs[addr / 4] = val;
        return;
    }
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_I2S_REGS_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        return;
    }
    }
}

static const MemoryRegionOps esp32s3_i2s_ops = {
    .read  = esp32s3_i2s_read,
    .write = esp32s3_i2s_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_i2s_init(Object *obj)
{
    ESP32S3I2SState *s = ESP32S3_I2S(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_i2s_ops, s,
                          TYPE_ESP32S3_I2S, ESP32S3_I2S_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_i2s_reset_hold(Object *obj, ResetType type)
{
    ESP32S3I2SState *s = ESP32S3_I2S(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->int_raw = 0;
    s->int_ena = 0;

    /* DATE register default */
    s->regs[I2S_DATE_REG / 4] = 0x02002060;

    qemu_set_irq(s->irq, 0);
}

static void esp32s3_i2s_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_i2s_reset_hold;
}

static const TypeInfo esp32s3_i2s_info = {
    .name          = TYPE_ESP32S3_I2S,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3I2SState),
    .instance_init = esp32s3_i2s_init,
    .class_init    = esp32s3_i2s_class_init,
};

static void esp32s3_i2s_register_types(void)
{
    type_register_static(&esp32s3_i2s_info);
}

type_init(esp32s3_i2s_register_types);
