/*
 * ESP32-S3 APB SAR ADC Controller (stub)
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Register-faithful compatibility model for the APB SAR ADC block at
 * 0x60040000.  Provides:
 *  - Full R/W register store (0x400 bytes)
 *  - Standard interrupt semantics (INT_RAW / INT_ST / INT_ENA / INT_CLR)
 *  - Deterministic mid-scale ADC readings for SAR1 and SAR2 data status
 *  - Self-clearing SAR_START bit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/misc/esp32s3_apb_saradc.h"

static void esp32s3_apb_saradc_update_irq(ESP32S3ApbSaradcState *s)
{
    bool level = (s->int_raw & s->int_ena) != 0;
    qemu_set_irq(s->irq, level);
}

static uint64_t esp32s3_apb_saradc_read(void *opaque, hwaddr addr,
                                         unsigned int size)
{
    ESP32S3ApbSaradcState *s = ESP32S3_APB_SARADC(opaque);
    uint32_t idx = addr >> 2;

    switch (addr) {
    case APB_SARADC_REG_SAR1_DATA_STATUS:
        /* Return a deterministic mid-scale 12-bit value (channel 0).
         * Format: [16:0] = data, [19:17] = channel */
        return APB_SARADC_MIDSCALE_12BIT;

    case APB_SARADC_REG_SAR2_DATA_STATUS:
        /* Same for SAR2 */
        return APB_SARADC_MIDSCALE_12BIT;

    case APB_SARADC_REG_SAR1_STATUS:
    case APB_SARADC_REG_SAR2_STATUS:
        /* FSM idle */
        return 0;

    case APB_SARADC_REG_INT_RAW:
        return s->int_raw;

    case APB_SARADC_REG_INT_ST:
        return s->int_raw & s->int_ena;

    case APB_SARADC_REG_INT_ENA:
        return s->int_ena;

    case APB_SARADC_REG_DATE:
        return APB_SARADC_DATE_DEFAULT;

    default:
        if (idx < APB_SARADC_REG_COUNT) {
            return s->regs[idx];
        }
        return 0;
    }
}

static void esp32s3_apb_saradc_write(void *opaque, hwaddr addr,
                                      uint64_t value, unsigned int size)
{
    ESP32S3ApbSaradcState *s = ESP32S3_APB_SARADC(opaque);
    uint32_t idx = addr >> 2;

    switch (addr) {
    case APB_SARADC_REG_CTRL:
        /* SAR_START (bit 1) is self-clearing.  We immediately "complete"
         * the conversion by raising the DONE interrupt. */
        if (value & BIT(1)) {
            value &= ~BIT(1);
            s->int_raw |= APB_SARADC_INT_SAR1_DONE | APB_SARADC_INT_SAR2_DONE;
            esp32s3_apb_saradc_update_irq(s);
        }
        if (idx < APB_SARADC_REG_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        break;

    case APB_SARADC_REG_INT_ENA:
        s->int_ena = (uint32_t)value;
        esp32s3_apb_saradc_update_irq(s);
        break;

    case APB_SARADC_REG_INT_CLR:
        s->int_raw &= ~(uint32_t)value;
        esp32s3_apb_saradc_update_irq(s);
        break;

    case APB_SARADC_REG_DATE:
        /* Read-only date register; ignore writes */
        break;

    case APB_SARADC_REG_INT_RAW:
    case APB_SARADC_REG_INT_ST:
        /* Read-only — ignore writes */
        break;

    default:
        if (idx < APB_SARADC_REG_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        break;
    }
}

static const MemoryRegionOps esp32s3_apb_saradc_ops = {
    .read = esp32s3_apb_saradc_read,
    .write = esp32s3_apb_saradc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32s3_apb_saradc_reset_hold(Object *obj, ResetType type)
{
    ESP32S3ApbSaradcState *s = ESP32S3_APB_SARADC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->int_raw = 0;
    s->int_ena = 0;

    /* Defaults from apb_saradc_reg.h:
     *   CTRL: WAIT_ARB_CYCLE=1(bit30), SAR_CLK_GATED=1(bit6), SAR_CLK_DIV=4(bits14:7)
     *   CTRL2: TIMER_TARGET=10(bits23:12)
     *   FSM_WAIT: various small defaults
     *   ARB_CTRL: APB_FORCE=0, GRANT_FORCE=0, APB_PRIORITY=0
     */
    s->regs[APB_SARADC_REG_CTRL >> 2]  = (1 << 30) | (1 << 6) | (4 << 7);
    s->regs[APB_SARADC_REG_CTRL2 >> 2] = (10 << 12);
    s->regs[APB_SARADC_REG_FSM_WAIT >> 2] = 0x00FF0808;  /* reasonable defaults */
    s->regs[APB_SARADC_REG_CLKM_CONF >> 2] = 0;

    qemu_set_irq(s->irq, 0);
}

static void esp32s3_apb_saradc_init(Object *obj)
{
    ESP32S3ApbSaradcState *s = ESP32S3_APB_SARADC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_apb_saradc_ops, s,
                          TYPE_ESP32S3_APB_SARADC, APB_SARADC_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_apb_saradc_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_apb_saradc_reset_hold;
}

static const TypeInfo esp32s3_apb_saradc_info = {
    .name          = TYPE_ESP32S3_APB_SARADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3ApbSaradcState),
    .instance_init = esp32s3_apb_saradc_init,
    .class_init    = esp32s3_apb_saradc_class_init,
};

static void esp32s3_apb_saradc_register_types(void)
{
    type_register_static(&esp32s3_apb_saradc_info);
}

type_init(esp32s3_apb_saradc_register_types)
