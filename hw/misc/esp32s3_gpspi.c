/*
 * ESP32-S3 GP-SPI Controller Model (SPI2/SPI3)
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Sprint S7: R/W register store with immediate transaction completion.
 * Sprint S9: SPI bridge — emit [PERIPH][SPI] on stderr for GUI display bridge.
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
#include "hw/qdev-properties.h"
#include "exec/address-spaces.h"

#define GPSPI_DMA_KICK_BYTES GPSPI_BRIDGE_DMA_MAX

/* GPIO_OUT register physical address (ESP32-S3 GPIO matrix) */
#define GPIO_OUT_PHYS_ADDR  0x60004004u

/*
 * Read the state of a single GPIO output pin.
 * Reads GPIO_OUT_REG via physical memory so no extra wiring is needed.
 */
static int gpspi_read_gpio(uint8_t gpio_num)
{
    uint32_t gpio_out = 0;
    if (gpio_num > 31) {
        return -1;
    }
    cpu_physical_memory_read(GPIO_OUT_PHYS_ADDR, &gpio_out, sizeof(gpio_out));
    return (gpio_out >> gpio_num) & 1;
}

/*
 * Emit a [PERIPH][SPI] bridge event on stderr.
 * The GUI intercepts these lines to forward to the Python device sim.
 */
static void esp32s3_gpspi_bridge_emit(ESP32S3GpSpiState *s,
                                      const uint8_t *tx, uint32_t tx_len,
                                      uint32_t rx_len)
{
    if (!s->bridge_enabled || !s->controller_name) {
        return;
    }
    if (tx_len == 0 && rx_len == 0) {
        return;
    }

    /* Read DC pin state (-1 if not configured) */
    int dc = gpspi_read_gpio(s->dc_gpio);

    /* Build JSON manually (no json-c dependency, single line for parser) */
    /* Worst case: 6 chars per byte ("255,") × 4096 ≈ 25 KB */
    size_t bufsz = 256 + tx_len * 5;
    char *buf = g_malloc(bufsz);
    int pos = 0;

    pos += snprintf(buf + pos, bufsz - pos,
                    "[PERIPH][SPI] {\"controller\":\"%s\"",
                    s->controller_name);

    if (dc >= 0) {
        pos += snprintf(buf + pos, bufsz - pos, ",\"dc\":%d", dc);
    }

    if (tx_len > 0) {
        pos += snprintf(buf + pos, bufsz - pos, ",\"tx\":[");
        for (uint32_t i = 0; i < tx_len; i++) {
            if (i > 0) {
                buf[pos++] = ',';
            }
            pos += snprintf(buf + pos, bufsz - pos, "%u", tx[i]);
        }
        buf[pos++] = ']';
    }

    if (rx_len > 0) {
        pos += snprintf(buf + pos, bufsz - pos, ",\"rx_len\":%u", rx_len);
    }

    pos += snprintf(buf + pos, bufsz - pos, "}\n");

    fwrite(buf, 1, pos, stderr);
    fflush(stderr);
    g_free(buf);
}

static void esp32s3_gpspi_try_dma(ESP32S3GpSpiState *s)
{
    uint32_t chan;
    uint8_t buffer[GPSPI_DMA_KICK_BYTES] = { 0 };

    s->dma_out_len = 0;   /* Reset DMA capture */

    /* Only capture DMA TX payload when SPI DMA TX path is enabled. */
    if ((s->regs[SPI_DMA_CONF_REG / 4] & SPI_DMA_TX_ENA) == 0) {
        return;
    }

    if (!s->gdma) {
        return;
    }

    if (esp_gdma_get_channel_periph(s->gdma, s->gdma_periph,
                                    ESP_GDMA_OUT_IDX, &chan)) {
        if (esp_gdma_read_channel(s->gdma, chan, buffer, sizeof(buffer))) {
            s->int_raw |= SPI_INT_WR_DMA_DONE;
            /* Save DMA TX data for bridge emission */
            uint32_t ms_dlen = s->regs[SPI_MS_DLEN_REG / 4];
            uint32_t bit_len = (ms_dlen & 0x3FFFF) + 1;  /* MS_DATA_BITLEN + 1 */
            uint32_t byte_len = (bit_len + 7) / 8;
            if (byte_len > GPSPI_BRIDGE_DMA_MAX) {
                byte_len = GPSPI_BRIDGE_DMA_MAX;
            }
            memcpy(s->dma_out_buf, buffer, byte_len);
            s->dma_out_len = byte_len;
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

            /* ---- Bridge emission ---- */
            if (s->bridge_enabled) {
                uint32_t user_reg = s->regs[SPI_USER_REG / 4];
                uint32_t ms_dlen  = s->regs[SPI_MS_DLEN_REG / 4];
                uint32_t bit_len  = (ms_dlen & 0x3FFFF) + 1;
                uint32_t byte_len = (bit_len + 7) / 8;

                uint32_t tx_len = 0;
                uint32_t rx_len = 0;
                uint8_t tx_buf[64];  /* Max PIO: 16×4 = 64 bytes */

                if (user_reg & SPI_USR_MOSI) {
                    if (s->dma_out_len > 0) {
                        /* DMA path — data already in dma_out_buf */
                        tx_len = s->dma_out_len;
                    } else {
                        /* PIO path — extract from W0–W15 */
                        tx_len = (byte_len > 64) ? 64 : byte_len;
                        for (uint32_t i = 0; i < tx_len; i++) {
                            uint32_t reg_idx = (SPI_W0_REG / 4) + (i / 4);
                            uint32_t shift = (i % 4) * 8;
                            tx_buf[i] = (s->regs[reg_idx] >> shift) & 0xFF;
                        }
                    }
                }
                if (user_reg & SPI_USR_MISO) {
                    rx_len = byte_len;
                }

                const uint8_t *emit_ptr = (s->dma_out_len > 0)
                    ? s->dma_out_buf : tx_buf;
                esp32s3_gpspi_bridge_emit(s, emit_ptr, tx_len, rx_len);
            }

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

/* ---- Runtime-writable QOM property: "bridge-dc-gpio" ---- */

static char *gpspi_get_dc_gpio(Object *obj, Error **errp)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(obj);
    if (s->dc_gpio == 0xFF) {
        return g_strdup("");
    }
    return g_strdup_printf("%u", s->dc_gpio);
}

static void gpspi_set_dc_gpio(Object *obj, const char *value, Error **errp)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(obj);
    if (!value || !*value) {
        s->dc_gpio = 0xFF;
        return;
    }
    unsigned long pin = strtoul(value, NULL, 10);
    s->dc_gpio = (pin <= 48) ? (uint8_t)pin : 0xFF;
}

static void esp32s3_gpspi_init(Object *obj)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_gpspi_ops, s,
                          TYPE_ESP32S3_GPSPI, ESP32S3_GPSPI_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->dc_gpio = 0xFF;          /* No DC pin by default */
    s->bridge_enabled = false;
    s->controller_name = NULL;
    s->dma_out_len = 0;

    /* Runtime-writable property for DC GPIO (set by GUI via QMP) */
    object_property_add_str(obj, "bridge-dc-gpio",
                            gpspi_get_dc_gpio, gpspi_set_dc_gpio);
}

static void esp32s3_gpspi_reset_hold(Object *obj, ResetType type)
{
    ESP32S3GpSpiState *s = ESP32S3_GPSPI(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->int_raw = 0;
    s->int_ena = 0;
    s->dma_out_len = 0;

    /* DATE register default */
    s->regs[SPI_DATE_REG / 4] = 0x02003020;

    qemu_set_irq(s->irq, 0);
}

static Property esp32s3_gpspi_properties[] = {
    DEFINE_PROP_STRING("controller-name", ESP32S3GpSpiState, controller_name),
    DEFINE_PROP_BOOL("bridge-enabled", ESP32S3GpSpiState, bridge_enabled, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_gpspi_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);
    rc->phases.hold = esp32s3_gpspi_reset_hold;
    device_class_set_props(dc, esp32s3_gpspi_properties);
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
