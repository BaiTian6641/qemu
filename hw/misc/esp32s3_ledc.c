/*
 * ESP32-S3 LED PWM Controller (LEDC)
 *
 * Low-speed only: 8 channels bound to 4 timers.
 * MVP: register-level read/write with duty shadow update and
 * immediate timer overflow semantics for driver compatibility.
 *
 * Register layout follows ESP32-S3 TRM v1.2 Chapter 35.
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
#include "hw/irq.h"
#include "hw/misc/esp32s3_ledc.h"

#define LEDC_DEBUG 0

#if LEDC_DEBUG
#define LEDC_DPRINTF(fmt, ...) \
    qemu_log_mask(LOG_UNIMP, "esp32s3_ledc: " fmt, ## __VA_ARGS__)
#else
#define LEDC_DPRINTF(fmt, ...) do {} while (0)
#endif

/* ================================================================== */
/*  IRQ helper                                                         */
/* ================================================================== */

static void esp32s3_ledc_update_irq(ESP32S3LEDCState *s)
{
    int level = !!(s->int_raw & s->int_ena);
    qemu_set_irq(s->irq, level);
}

/* ================================================================== */
/*  Register decode helpers                                            */
/* ================================================================== */

/* Return channel index [0..7] if addr is in channel register range, else -1 */
static int ledc_addr_to_channel(hwaddr addr, hwaddr *reg_off)
{
    if (addr < ESP32S3_LEDC_CH_REG_END) {
        int ch = addr / 0x14;
        if (ch < ESP32S3_LEDC_CHANNEL_COUNT) {
            *reg_off = addr % 0x14;
            return ch;
        }
    }
    return -1;
}

/* Return timer index [0..3] if addr is in timer register range, else -1 */
static int ledc_addr_to_timer(hwaddr addr, hwaddr *reg_off)
{
    if (addr >= 0xA0 && addr < 0xC0) {
        int t = (addr - 0xA0) / 0x08;
        if (t < ESP32S3_LEDC_TIMER_COUNT) {
            *reg_off = (addr - 0xA0) % 0x08;
            return t;
        }
    }
    return -1;
}

/* ================================================================== */
/*  MMIO read/write                                                    */
/* ================================================================== */

static uint64_t esp32s3_ledc_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3LEDCState *s = ESP32S3_LEDC(opaque);
    hwaddr reg_off;
    int idx;

    /* Channel registers */
    idx = ledc_addr_to_channel(addr, &reg_off);
    if (idx >= 0) {
        switch (reg_off) {
        case 0x00: return s->ch_conf0[idx];
        case 0x04: return s->ch_hpoint[idx];
        case 0x08: return s->ch_duty[idx];
        case 0x0C: return s->ch_conf1[idx];
        case 0x10: return s->ch_duty_r[idx];
        default: return 0;
        }
    }

    /* Timer registers */
    idx = ledc_addr_to_timer(addr, &reg_off);
    if (idx >= 0) {
        switch (reg_off) {
        case 0x00: return s->timer_conf[idx];
        case 0x04: return s->timer_value[idx];
        default: return 0;
        }
    }

    /* Other registers */
    switch (addr) {
    case ESP32S3_LEDC_INT_RAW_OFF:
        return s->int_raw;
    case ESP32S3_LEDC_INT_ST_OFF:
        return s->int_raw & s->int_ena;
    case ESP32S3_LEDC_INT_ENA_OFF:
        return s->int_ena;
    case ESP32S3_LEDC_INT_CLR_OFF:
        return 0;
    case ESP32S3_LEDC_CONF_OFF:
        return s->conf;
    case ESP32S3_LEDC_DATE_OFF:
        return 0x19052600;  /* VERSION */
    default:
        LEDC_DPRINTF("read: unhandled 0x%03" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void esp32s3_ledc_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned int size)
{
    ESP32S3LEDCState *s = ESP32S3_LEDC(opaque);
    hwaddr reg_off;
    int idx;

    /* Channel registers */
    idx = ledc_addr_to_channel(addr, &reg_off);
    if (idx >= 0) {
        switch (reg_off) {
        case 0x00: /* CH_CONF0 */
            s->ch_conf0[idx] = (uint32_t)value;
            if (value & LEDC_S3_CONF0_PARA_UP) {
                /* Update channel parameters — copy duty to duty_r shadow */
                s->ch_duty_r[idx] = s->ch_duty[idx];
                s->ch_conf0[idx] &= ~LEDC_S3_CONF0_PARA_UP;
                /* Signal duty change done */
                s->int_raw |= LEDC_S3_INT_DUTY_CHNG(idx);
                esp32s3_ledc_update_irq(s);
            }
            break;
        case 0x04:
            s->ch_hpoint[idx] = (uint32_t)value;
            break;
        case 0x08:
            s->ch_duty[idx] = (uint32_t)value;
            break;
        case 0x0C:
            s->ch_conf1[idx] = (uint32_t)value;
            break;
        case 0x10:
            /* DUTY_R is read-only */
            break;
        }
        return;
    }

    /* Timer registers */
    idx = ledc_addr_to_timer(addr, &reg_off);
    if (idx >= 0) {
        switch (reg_off) {
        case 0x00: /* TIMER_CONF */
            s->timer_conf[idx] = (uint32_t)value;
            if (value & LEDC_S3_TIMER_PARA_UP) {
                /* Timer parameter update — clear flag, signal overflow */
                s->timer_conf[idx] &= ~LEDC_S3_TIMER_PARA_UP;
                s->timer_value[idx] = 0;
                s->int_raw |= LEDC_S3_INT_TIMER_OVF(idx);
                esp32s3_ledc_update_irq(s);
            }
            if (value & LEDC_S3_TIMER_RST) {
                s->timer_value[idx] = 0;
                s->timer_conf[idx] &= ~LEDC_S3_TIMER_RST;
            }
            break;
        case 0x04: /* TIMER_VALUE (read-only counter) */
            break;
        }
        return;
    }

    /* Other registers */
    switch (addr) {
    case ESP32S3_LEDC_INT_RAW_OFF:
        /* Raw can be set by SW for testing */
        s->int_raw |= (uint32_t)value;
        esp32s3_ledc_update_irq(s);
        break;
    case ESP32S3_LEDC_INT_CLR_OFF:
        s->int_raw &= ~(uint32_t)value;
        esp32s3_ledc_update_irq(s);
        break;
    case ESP32S3_LEDC_INT_ENA_OFF:
        s->int_ena = (uint32_t)value;
        esp32s3_ledc_update_irq(s);
        break;
    case ESP32S3_LEDC_CONF_OFF:
        s->conf = (uint32_t)value;
        break;
    case ESP32S3_LEDC_DATE_OFF:
        /* Version register, ignore writes */
        break;
    default:
        LEDC_DPRINTF("write: unhandled 0x%03" HWADDR_PRIx " = 0x%08" PRIx64 "\n",
                    addr, value);
        break;
    }
}

/* ================================================================== */
/*  Lifecycle                                                          */
/* ================================================================== */

static const MemoryRegionOps esp32s3_ledc_ops = {
    .read  = esp32s3_ledc_read,
    .write = esp32s3_ledc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_ledc_reset_hold(Object *obj, ResetType type)
{
    ESP32S3LEDCState *s = ESP32S3_LEDC(obj);

    memset(s->ch_conf0, 0, sizeof(s->ch_conf0));
    memset(s->ch_hpoint, 0, sizeof(s->ch_hpoint));
    memset(s->ch_duty, 0, sizeof(s->ch_duty));
    memset(s->ch_conf1, 0, sizeof(s->ch_conf1));
    memset(s->ch_duty_r, 0, sizeof(s->ch_duty_r));
    memset(s->timer_conf, 0, sizeof(s->timer_conf));
    memset(s->timer_value, 0, sizeof(s->timer_value));

    s->int_raw = 0;
    s->int_ena = 0;
    s->conf = 0;
}

static void esp32s3_ledc_realize(DeviceState *dev, Error **errp)
{
    esp32s3_ledc_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}

static void esp32s3_ledc_init(Object *obj)
{
    ESP32S3LEDCState *s = ESP32S3_LEDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_ledc_ops, s,
                          TYPE_ESP32S3_LEDC, ESP32S3_LEDC_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_ledc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_ledc_reset_hold;
    dc->realize = esp32s3_ledc_realize;
}

static const TypeInfo esp32s3_ledc_info = {
    .name          = TYPE_ESP32S3_LEDC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3LEDCState),
    .instance_init = esp32s3_ledc_init,
    .class_init    = esp32s3_ledc_class_init,
    .class_size    = sizeof(ESP32S3LEDCClass),
};

static void esp32s3_ledc_register_types(void)
{
    type_register_static(&esp32s3_ledc_info);
}

type_init(esp32s3_ledc_register_types)
