/*
 * ESP32-S3 LCD_CAM Controller Model
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Sprint S7: R/W register store with interrupt model stub.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32s3_lcd_cam.h"

static void esp32s3_lcd_cam_update_irq(ESP32S3LcdCamState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) ? 1 : 0);
}

static uint64_t esp32s3_lcd_cam_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    ESP32S3LcdCamState *s = ESP32S3_LCD_CAM(opaque);

    switch (addr) {
    case LCD_CAM_LC_DMA_INT_RAW_REG:
        return s->int_raw;
    case LCD_CAM_LC_DMA_INT_ST_REG:
        return s->int_raw & s->int_ena;
    case LCD_CAM_LC_DMA_INT_ENA_REG:
        return s->int_ena;
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_LCD_CAM_REGS_COUNT) {
            return s->regs[idx];
        }
        return 0;
    }
    }
}

static void esp32s3_lcd_cam_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned int size)
{
    ESP32S3LcdCamState *s = ESP32S3_LCD_CAM(opaque);

    switch (addr) {
    case LCD_CAM_LC_DMA_INT_CLR_REG:
        s->int_raw &= ~(uint32_t)value;
        esp32s3_lcd_cam_update_irq(s);
        return;
    case LCD_CAM_LC_DMA_INT_ENA_REG:
        s->int_ena = (uint32_t)value;
        esp32s3_lcd_cam_update_irq(s);
        return;
    case LCD_CAM_LCD_USER_REG: {
        uint32_t val = (uint32_t)value;
        /* LCD_START is self-clearing and triggers completion */
        if (val & BIT(0)) {
            /* MVP: immediate LCD transfer completion */
            s->int_raw |= LCD_CAM_INT_LCD_TRANS_DONE;
            esp32s3_lcd_cam_update_irq(s);
            val &= ~BIT(0);
        }
        /* LCD_RESET is self-clearing */
        if (val & BIT(1)) {
            val &= ~BIT(1);
        }
        s->regs[addr / 4] = val;
        return;
    }
    case LCD_CAM_CAM_CTRL1_REG: {
        uint32_t val = (uint32_t)value;
        /* CAM_START and CAM_RESET are self-clearing */
        if (val & BIT(0)) {
            val &= ~BIT(0);
        }
        if (val & BIT(1)) {
            val &= ~BIT(1);
        }
        s->regs[addr / 4] = val;
        return;
    }
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_LCD_CAM_REGS_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        return;
    }
    }
}

static const MemoryRegionOps esp32s3_lcd_cam_ops = {
    .read  = esp32s3_lcd_cam_read,
    .write = esp32s3_lcd_cam_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_lcd_cam_init(Object *obj)
{
    ESP32S3LcdCamState *s = ESP32S3_LCD_CAM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_lcd_cam_ops, s,
                          TYPE_ESP32S3_LCD_CAM, ESP32S3_LCD_CAM_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_lcd_cam_reset_hold(Object *obj, ResetType type)
{
    ESP32S3LcdCamState *s = ESP32S3_LCD_CAM(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->int_raw = 0;
    s->int_ena = 0;

    /* DATE register default */
    s->regs[LCD_CAM_LC_REG_DATE_REG / 4] = 0x02003020;

    qemu_set_irq(s->irq, 0);
}

static void esp32s3_lcd_cam_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_lcd_cam_reset_hold;
}

static const TypeInfo esp32s3_lcd_cam_info = {
    .name          = TYPE_ESP32S3_LCD_CAM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3LcdCamState),
    .instance_init = esp32s3_lcd_cam_init,
    .class_init    = esp32s3_lcd_cam_class_init,
};

static void esp32s3_lcd_cam_register_types(void)
{
    type_register_static(&esp32s3_lcd_cam_info);
}

type_init(esp32s3_lcd_cam_register_types);
