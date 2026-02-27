/*
 * ESP32-S3 IO MUX peripheral
 *
 * Implements the IO MUX register model for ESP32-S3. This handles pad
 * configuration (drive strength, pull-up/down, function selection, etc.)
 * for all 49 GPIO pads.
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
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32s3_iomux.h"
#include "trace.h"

#define IOMUX_WARNING 0

static uint64_t esp32s3_iomux_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3IOMuxState *s = ESP32S3_IOMUX(opaque);
    uint64_t r = 0;

    switch (addr) {
    case A_IO_MUX_PIN_CTRL:
        r = s->pin_ctrl;
        break;

    case A_IO_MUX_DATE:
        r = s->date_reg;
        break;

    default:
        trace_esp32s3_iomux_read(addr, r);
        /* Check if this is a per-GPIO pad register */
        if (addr >= IO_MUX_GPIOn_REG_OFFSET(0) &&
            addr <= IO_MUX_GPIOn_REG_OFFSET(ESP32S3_IOMUX_GPIO_COUNT - 1) &&
            ((addr - IO_MUX_GPIOn_REG_OFFSET(0)) % 4 == 0)) {
            int gpio_num = (addr - IO_MUX_GPIOn_REG_OFFSET(0)) / 4;
            r = s->gpio_reg[gpio_num];
        } else {
#if IOMUX_WARNING
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: bad read at offset 0x%04" HWADDR_PRIx "\n",
                          __func__, addr);
#endif
            r = 0;
        }
        break;
    }

    trace_esp32s3_iomux_read(addr, (uint32_t)r);
    return r;
}

static void esp32s3_iomux_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    ESP32S3IOMuxState *s = ESP32S3_IOMUX(opaque);

    trace_esp32s3_iomux_write(addr, (uint32_t)value);

    switch (addr) {
    case A_IO_MUX_PIN_CTRL:
        s->pin_ctrl = value & 0xFFF;  /* Only bits [11:0] are valid */
        break;

    case A_IO_MUX_DATE:
        s->date_reg = value;
        break;

    default:
        /* Check if this is a per-GPIO pad register */
        if (addr >= IO_MUX_GPIOn_REG_OFFSET(0) &&
            addr <= IO_MUX_GPIOn_REG_OFFSET(ESP32S3_IOMUX_GPIO_COUNT - 1) &&
            ((addr - IO_MUX_GPIOn_REG_OFFSET(0)) % 4 == 0)) {
            int gpio_num = (addr - IO_MUX_GPIOn_REG_OFFSET(0)) / 4;
            /* Mask: bits [15:0] are defined fields */
            s->gpio_reg[gpio_num] = value & 0xFFFF;
        } else {
#if IOMUX_WARNING
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: bad write at offset 0x%04" HWADDR_PRIx "\n",
                          __func__, addr);
#endif
        }
        break;
    }
}

static const MemoryRegionOps esp32s3_iomux_ops = {
    .read = esp32s3_iomux_read,
    .write = esp32s3_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32s3_iomux_reset_hold(Object *obj, ResetType type)
{
    ESP32S3IOMuxState *s = ESP32S3_IOMUX(obj);

    trace_esp32s3_iomux_reset();

    s->pin_ctrl = 0;
    s->date_reg = ESP32S3_IOMUX_DATE_VERSION;

    for (int i = 0; i < ESP32S3_IOMUX_GPIO_COUNT; i++) {
        s->gpio_reg[i] = ESP32S3_IOMUX_GPIO_REG_DEFAULT;
    }
}

static void esp32s3_iomux_init(Object *obj)
{
    ESP32S3IOMuxState *s = ESP32S3_IOMUX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_iomux_ops, s,
                          TYPE_ESP32S3_IOMUX, ESP32S3_IOMUX_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32s3_iomux_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_iomux_reset_hold;
}

static const TypeInfo esp32s3_iomux_info = {
    .name = TYPE_ESP32S3_IOMUX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3IOMuxState),
    .instance_init = esp32s3_iomux_init,
    .class_init = esp32s3_iomux_class_init,
};

static void esp32s3_iomux_register_types(void)
{
    type_register_static(&esp32s3_iomux_info);
}

type_init(esp32s3_iomux_register_types);
