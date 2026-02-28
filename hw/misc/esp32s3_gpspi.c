/*
 * ESP32-S3 GP-SPI Controller Model (SPI2/SPI3)
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Sprint S7: R/W register store with immediate transaction completion.
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
#include "hw/misc/esp32s3_gpspi.h"

#define GPSPI_DMA_KICK_BYTES 256

static void esp32s3_gpspi_try_dma(ESP32S3GpSpiState *s)
{
    uint32_t chan;
    uint8_t buffer[GPSPI_DMA_KICK_BYTES] = { 0 };

    if (!s->gdma) {
        return;
    }

    if (esp_gdma_get_channel_periph(s->gdma, s->gdma_periph,
                                    ESP_GDMA_OUT_IDX, &chan)) {
        if (esp_gdma_read_channel(s->gdma, chan, buffer, sizeof(buffer))) {
            s->int_raw |= SPI_INT_WR_DMA_DONE;
        }
    }

    if (esp_gdma_get_channel_periph(s->gdma, s->gdma_periph,
                                    ESP_GDMA_IN_IDX, &chan)) {
        if (esp_gdma_write_channel(s->gdma, chan, buffer, sizeof(buffer))) {
            s->int_raw |= SPI_INT_RD_DMA_DONE;
        }
    }
}

static void esp32s3_gpspi_update_irq(ESP32S3GpSpiState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) ? 1 : 0);
}

static uint64_t esp32s3_gpspi_read(void *opaque, hwaddr addr,
                                   unsigned int size)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(opaque);

    switch (addr) {
    case SPI_DMA_INT_RAW_REG:
        return s->int_raw;
    case SPI_DMA_INT_ST_REG:
        return s->int_raw & s->int_ena;
    case SPI_DMA_INT_ENA_REG:
        return s->int_ena;
    case SPI_CMD_REG:
        /* USR bit is self-clearing — always returns 0 for busy poll */
        return s->regs[addr / 4] & ~SPI_CMD_USR;
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_GPSPI_REGS_COUNT) {
            return s->regs[idx];
        }
        return 0;
    }
    }
}

static void esp32s3_gpspi_write(void *opaque, hwaddr addr,
                                uint64_t value, unsigned int size)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(opaque);

    switch (addr) {
    case SPI_DMA_INT_CLR_REG:
        s->int_raw &= ~(uint32_t)value;
        esp32s3_gpspi_update_irq(s);
        return;
    case SPI_DMA_INT_ENA_REG:
        s->int_ena = (uint32_t)value;
        esp32s3_gpspi_update_irq(s);
        return;
    case SPI_DMA_INT_SET_REG:
        /* Software interrupt set */
        s->int_raw |= (uint32_t)value;
        esp32s3_gpspi_update_irq(s);
        return;
    case SPI_CMD_REG: {
        uint32_t val = (uint32_t)value;
        if (val & SPI_CMD_USR) {
            esp32s3_gpspi_try_dma(s);

            /* MVP: immediate transaction completion */
            s->int_raw |= SPI_INT_TRANS_DONE;
            esp32s3_gpspi_update_irq(s);
            val &= ~SPI_CMD_USR;  /* Self-clear USR bit */
        }
        if (val & SPI_CMD_UPDATE) {
            val &= ~SPI_CMD_UPDATE;  /* Self-clear UPDATE bit */
        }
        s->regs[addr / 4] = val;
        return;
    }
    case SPI_SLAVE_REG: {
        uint32_t val = (uint32_t)value;
        /* SOFT_RESET bit (BIT[2]) is self-clearing */
        if (val & BIT(2)) {
            val &= ~BIT(2);
        }
        s->regs[addr / 4] = val;
        return;
    }
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_GPSPI_REGS_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        return;
    }
    }
}

static const MemoryRegionOps esp32s3_gpspi_ops = {
    .read  = esp32s3_gpspi_read,
    .write = esp32s3_gpspi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_gpspi_init(Object *obj)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_gpspi_ops, s,
                          TYPE_ESP32S3_GPSPI, ESP32S3_GPSPI_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_gpspi_reset_hold(Object *obj, ResetType type)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->int_raw = 0;
    s->int_ena = 0;

    /* DATE register default */
    s->regs[SPI_DATE_REG / 4] = 0x02003020;

    qemu_set_irq(s->irq, 0);
}

static void esp32s3_gpspi_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_gpspi_reset_hold;
}

static const TypeInfo esp32s3_gpspi_info = {
    .name          = TYPE_ESP32S3_GPSPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3GpSpiState),
    .instance_init = esp32s3_gpspi_init,
    .class_init    = esp32s3_gpspi_class_init,
};

static void esp32s3_gpspi_register_types(void)
{
    type_register_static(&esp32s3_gpspi_info);
}

type_init(esp32s3_gpspi_register_types);
