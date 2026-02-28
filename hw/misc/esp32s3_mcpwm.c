/*
 * ESP32-S3 MCPWM Controller Model
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Sprint S7: R/W register store with interrupt model.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32s3_mcpwm.h"

static void esp32s3_mcpwm_update_irq(ESP32S3McpwmState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) ? 1 : 0);
}

static uint64_t esp32s3_mcpwm_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3McpwmState *s = ESP32S3_MCPWM(opaque);

    switch (addr) {
    case MCPWM_INT_RAW_REG:
        return s->int_raw;
    case MCPWM_INT_ST_REG:
        return s->int_raw & s->int_ena;
    case MCPWM_INT_ENA_REG:
        return s->int_ena;
    case MCPWM_INT_CLR_REG:
        return 0;  /* write-only */
    case MCPWM_CLK_REG:
        return s->regs[addr / 4];
    case MCPWM_VERSION_REG:
        return 0x20190625;  /* MCPWM version per ESP32-S3 */
    /* Timer status registers are read-only: return counter = 0, direction = up */
    case MCPWM_TIMER0_STATUS_REG:
    case MCPWM_TIMER0_STATUS_REG + 0x10:
    case MCPWM_TIMER0_STATUS_REG + 0x20:
        return 0;
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_MCPWM_REGS_COUNT) {
            return s->regs[idx];
        }
        return 0;
    }
    }
}

static void esp32s3_mcpwm_write(void *opaque, hwaddr addr,
                                uint64_t value, unsigned int size)
{
    ESP32S3McpwmState *s = ESP32S3_MCPWM(opaque);

    /* Handle INT_CLR: write offset after INT_ST is at +4 = 0x128.
     * But struct ends at 0x128. The ESP-IDF mcpwm_struct defines
     * int_clr at the same offset block. We handle it inline. */

    switch (addr) {
    case MCPWM_INT_ENA_REG:
        s->int_ena = (uint32_t)value;
        esp32s3_mcpwm_update_irq(s);
        return;
    case MCPWM_INT_CLR_REG:
        /* W1C: write-1-to-clear interrupt raw bits */
        s->int_raw &= ~(uint32_t)value;
        esp32s3_mcpwm_update_irq(s);
        return;
    case MCPWM_INT_RAW_REG:
        /* INT_RAW is RO/WTC/SS on ESP32-S3 — some bits allow W1C */
        s->int_raw &= ~(uint32_t)value;
        esp32s3_mcpwm_update_irq(s);
        return;
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_MCPWM_REGS_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        return;
    }
    }
}

static const MemoryRegionOps esp32s3_mcpwm_ops = {
    .read  = esp32s3_mcpwm_read,
    .write = esp32s3_mcpwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_mcpwm_init(Object *obj)
{
    ESP32S3McpwmState *s = ESP32S3_MCPWM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_mcpwm_ops, s,
                          TYPE_ESP32S3_MCPWM, ESP32S3_MCPWM_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_mcpwm_reset_hold(Object *obj, ResetType type)
{
    ESP32S3McpwmState *s = ESP32S3_MCPWM(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->int_raw = 0;
    s->int_ena = 0;

    qemu_set_irq(s->irq, 0);
}

static void esp32s3_mcpwm_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_mcpwm_reset_hold;
}

static const TypeInfo esp32s3_mcpwm_info = {
    .name          = TYPE_ESP32S3_MCPWM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3McpwmState),
    .instance_init = esp32s3_mcpwm_init,
    .class_init    = esp32s3_mcpwm_class_init,
};

static void esp32s3_mcpwm_register_types(void)
{
    type_register_static(&esp32s3_mcpwm_info);
}

type_init(esp32s3_mcpwm_register_types);
