/*
 * ESP32-S3 RTC IO MUX
 *
 * R/W register store for the 22 RTC GPIO pads.
 * Supports OUT/ENABLE/STATUS with W1TS/W1TC atomic operations,
 * per-pin config (interrupt, wakeup), per-pad config (pull-up/down,
 * drive strength, analog mux), and miscellaneous registers.
 *
 * This is a stub model: register values are stored and returned
 * faithfully, but no actual analog or deep-sleep GPIO behavior
 * is simulated.
 *
 * Register layout follows ESP32-S3 TRM v1.2 Chapter 6.13/39.
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
#include "qapi/error.h"
#include "hw/misc/esp32s3_rtc_io.h"

#define RTC_IO_DEBUG 0

#if RTC_IO_DEBUG
#define RTC_IO_DPRINTF(fmt, ...) \
    qemu_log_mask(LOG_UNIMP, "esp32s3_rtc_io: " fmt, ## __VA_ARGS__)
#else
#define RTC_IO_DPRINTF(fmt, ...) do {} while (0)
#endif

/* Mask for the 22 RTC GPIO pins (bits 0-21) */
#define RTC_IO_PIN_MASK  0x003FFFFF

/* ================================================================== */
/*  MMIO read/write                                                    */
/* ================================================================== */

static uint64_t esp32s3_rtc_io_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3RtcIoState *s = ESP32S3_RTC_IO(opaque);

    /* GPIO pin config registers */
    if (addr >= 0x28 && addr <= RTC_IO_PIN_LAST_OFF) {
        int idx = (addr - 0x28) / 4;
        if (idx < ESP32S3_RTC_IO_GPIO_COUNT) {
            return s->pin[idx];
        }
        return 0;
    }

    /* Per-pad config registers */
    if (addr >= 0x80 && addr <= RTC_IO_PAD_LAST_OFF) {
        int idx = (addr - 0x80) / 4;
        if (idx < ESP32S3_RTC_IO_GPIO_COUNT) {
            return s->pad[idx];
        }
        return 0;
    }

    switch (addr) {
    case RTC_IO_OUT_OFF:
        return s->out;
    case RTC_IO_OUT_W1TS_OFF:
    case RTC_IO_OUT_W1TC_OFF:
        return 0;  /* write-only */
    case RTC_IO_ENABLE_OFF:
        return s->enable;
    case RTC_IO_ENABLE_W1TS_OFF:
    case RTC_IO_ENABLE_W1TC_OFF:
        return 0;
    case RTC_IO_STATUS_OFF:
        return s->status;
    case RTC_IO_STATUS_W1TS_OFF:
    case RTC_IO_STATUS_W1TC_OFF:
        return 0;
    case RTC_IO_IN_OFF:
        return s->in;
    case RTC_IO_EXT_WAKEUP0_OFF:
        return s->ext_wakeup0;
    case RTC_IO_XTL_EXT_CTR_OFF:
        return s->xtl_ext_ctr;
    case RTC_IO_SAR_I2C_IO_OFF:
        return s->sar_i2c_io;
    case RTC_IO_TOUCH_CTRL_OFF:
        return s->touch_ctrl;
    case RTC_IO_DATE_OFF:
        return 0x19052600;  /* VERSION */
    default:
        RTC_IO_DPRINTF("read: unhandled 0x%03" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void esp32s3_rtc_io_write(void *opaque, hwaddr addr,
                                 uint64_t value, unsigned int size)
{
    ESP32S3RtcIoState *s = ESP32S3_RTC_IO(opaque);

    /* GPIO pin config registers */
    if (addr >= 0x28 && addr <= RTC_IO_PIN_LAST_OFF) {
        int idx = (addr - 0x28) / 4;
        if (idx < ESP32S3_RTC_IO_GPIO_COUNT) {
            s->pin[idx] = (uint32_t)value;
        }
        return;
    }

    /* Per-pad config registers */
    if (addr >= 0x80 && addr <= RTC_IO_PAD_LAST_OFF) {
        int idx = (addr - 0x80) / 4;
        if (idx < ESP32S3_RTC_IO_GPIO_COUNT) {
            s->pad[idx] = (uint32_t)value;
        }
        return;
    }

    switch (addr) {
    case RTC_IO_OUT_OFF:
        s->out = (uint32_t)value & RTC_IO_PIN_MASK;
        break;
    case RTC_IO_OUT_W1TS_OFF:
        s->out |= (uint32_t)value & RTC_IO_PIN_MASK;
        break;
    case RTC_IO_OUT_W1TC_OFF:
        s->out &= ~((uint32_t)value & RTC_IO_PIN_MASK);
        break;
    case RTC_IO_ENABLE_OFF:
        s->enable = (uint32_t)value & RTC_IO_PIN_MASK;
        break;
    case RTC_IO_ENABLE_W1TS_OFF:
        s->enable |= (uint32_t)value & RTC_IO_PIN_MASK;
        break;
    case RTC_IO_ENABLE_W1TC_OFF:
        s->enable &= ~((uint32_t)value & RTC_IO_PIN_MASK);
        break;
    case RTC_IO_STATUS_OFF:
        s->status = (uint32_t)value & RTC_IO_PIN_MASK;
        break;
    case RTC_IO_STATUS_W1TS_OFF:
        s->status |= (uint32_t)value & RTC_IO_PIN_MASK;
        break;
    case RTC_IO_STATUS_W1TC_OFF:
        s->status &= ~((uint32_t)value & RTC_IO_PIN_MASK);
        break;
    case RTC_IO_IN_OFF:
        /* IN is typically read-only, but accept writes for testing */
        s->in = (uint32_t)value & RTC_IO_PIN_MASK;
        break;
    case RTC_IO_EXT_WAKEUP0_OFF:
        s->ext_wakeup0 = (uint32_t)value;
        break;
    case RTC_IO_XTL_EXT_CTR_OFF:
        s->xtl_ext_ctr = (uint32_t)value;
        break;
    case RTC_IO_SAR_I2C_IO_OFF:
        s->sar_i2c_io = (uint32_t)value;
        break;
    case RTC_IO_TOUCH_CTRL_OFF:
        s->touch_ctrl = (uint32_t)value;
        break;
    case RTC_IO_DATE_OFF:
        /* Version register — ignore writes */
        break;
    default:
        RTC_IO_DPRINTF("write: unhandled 0x%03" HWADDR_PRIx " = 0x%08" PRIx64 "\n",
                       addr, value);
        break;
    }
}

/* ================================================================== */
/*  Lifecycle                                                          */
/* ================================================================== */

static const MemoryRegionOps esp32s3_rtc_io_ops = {
    .read  = esp32s3_rtc_io_read,
    .write = esp32s3_rtc_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_rtc_io_reset_hold(Object *obj, ResetType type)
{
    ESP32S3RtcIoState *s = ESP32S3_RTC_IO(obj);

    s->out = 0;
    s->enable = 0;
    s->status = 0;
    s->in = 0;

    memset(s->pin, 0, sizeof(s->pin));
    memset(s->pad, 0, sizeof(s->pad));

    s->ext_wakeup0 = 0;
    s->xtl_ext_ctr = 0;
    s->sar_i2c_io = 0;
    s->touch_ctrl = 0;
}

static void esp32s3_rtc_io_realize(DeviceState *dev, Error **errp)
{
    esp32s3_rtc_io_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}

static void esp32s3_rtc_io_init(Object *obj)
{
    ESP32S3RtcIoState *s = ESP32S3_RTC_IO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_rtc_io_ops, s,
                          TYPE_ESP32S3_RTC_IO, ESP32S3_RTC_IO_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    /* No dedicated IRQ — RTC_IO uses the shared RTC core interrupt */
}

static void esp32s3_rtc_io_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_rtc_io_reset_hold;
    dc->realize = esp32s3_rtc_io_realize;
}

static const TypeInfo esp32s3_rtc_io_info = {
    .name          = TYPE_ESP32S3_RTC_IO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3RtcIoState),
    .instance_init = esp32s3_rtc_io_init,
    .class_init    = esp32s3_rtc_io_class_init,
    .class_size    = sizeof(ESP32S3RtcIoClass),
};

static void esp32s3_rtc_io_register_types(void)
{
    type_register_static(&esp32s3_rtc_io_info);
}

type_init(esp32s3_rtc_io_register_types)
