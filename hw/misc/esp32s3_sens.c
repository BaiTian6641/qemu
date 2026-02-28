/*
 * ESP32-S3 SENS (Analog Sensor) Controller (stub)
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Register-faithful compatibility model for the SENS block at
 * 0x60008800 (DR_REG_SENS_BASE).  Provides:
 *  - Full R/W register store (0x200 bytes)
 *  - Touch sensor threshold/status registers (14 pads)
 *  - SAR ADC measurement control with MEAS_DONE auto-set
 *  - Temperature sensor readout (returns ~25 °C deterministic value)
 *  - ULP co-processor interrupt model (COCPU INT_RAW/ENA/ST/CLR)
 *  - W1TS/W1TC for COCPU_INT_ENA
 *  - SENS_SARDATE read-only
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/esp32s3_sens.h"

static uint64_t esp32s3_sens_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3SensState *s = ESP32S3_SENS(opaque);
    uint32_t idx = addr >> 2;

    switch (addr) {
    /* --- SAR measurement status: always report "done" and idle --- */
    case SENS_REG_SAR_READER1_STATUS:
    case SENS_REG_SAR_READER2_STATUS:
        return 0;  /* FSM idle */

    case SENS_REG_SAR_MEAS1_CTRL2:
        /*
         * Bit 31 = SAR1_DONE (read-only). Always report done so that
         * polling-based ADC drivers don't spin forever.
         */
        return s->regs[idx] | BIT(31);

    case SENS_REG_SAR_MEAS2_CTRL2:
        /* Bit 31 = SAR2_DONE */
        return s->regs[idx] | BIT(31);

    /* --- Temperature sensor: return ~25 °C deterministic value --- */
    case SENS_REG_SAR_TSENS_CTRL:
        /*
         * Bits [7:0] = tsens_out (read-only temperature code).
         * For ESP32-S3, out = offset + temp * sensitivity.
         * A code of ~128 roughly corresponds to 25 °C with default offset.
         * Bit 22 = TSENS_READY (1 = measurement done).
         */
        return (s->regs[idx] & 0xFFFFFF00) | 128 | BIT(22);

    /* --- Touch sensor status: all pads report no-touch --- */
    case SENS_REG_SAR_TOUCH_CHN_ST:
        /*
         * Bits [22:15] = scan_curr (current scanning pad = 0 = idle)
         * Bits [14:0]  = touch_meas_done (all pads done)
         */
        return 0x7FFF;  /* all 15 pads done, scan idle */

    /* Touch status 0..14 — approach/sleep/benchmark data */
    case SENS_REG_SAR_TOUCH_SLP_STATUS:
    case SENS_REG_SAR_TOUCH_APPR_STATUS:
        return 0;

    /* --- ULP co-processor interrupts --- */
    case SENS_REG_SAR_COCPU_INT_RAW:
        return s->cocpu_int_raw;

    case SENS_REG_SAR_COCPU_INT_ST:
        return s->cocpu_int_raw & s->cocpu_int_ena;

    case SENS_REG_SAR_COCPU_INT_ENA:
        return s->cocpu_int_ena;

    case SENS_REG_SAR_COCPU_STATE:
        /* Report ULP as "idle" (bits [25:0] = 0 means IDLE) */
        return 0;

    case SENS_REG_SAR_COCPU_DEBUG:
        return 0;

    case SENS_REG_SARDATE:
        return SENS_DATE_DEFAULT;

    default:
        if (idx < SENS_REG_COUNT) {
            return s->regs[idx];
        }
        return 0;
    }
}

static void esp32s3_sens_write(void *opaque, hwaddr addr,
                                uint64_t value, unsigned int size)
{
    ESP32S3SensState *s = ESP32S3_SENS(opaque);
    uint32_t idx = addr >> 2;

    switch (addr) {
    /* Start SAR1 measurement: self-clearing START bit (bit 1) */
    case SENS_REG_SAR_MEAS1_CTRL2:
        if (value & BIT(1)) {
            value &= ~BIT(1);  /* auto-clear start */
            /* SAR1_DONE (bit 31) will be set on next read */
        }
        if (idx < SENS_REG_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        break;

    /* Start SAR2 measurement: self-clearing START bit (bit 1) */
    case SENS_REG_SAR_MEAS2_CTRL2:
        if (value & BIT(1)) {
            value &= ~BIT(1);
        }
        if (idx < SENS_REG_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        break;

    /* --- COCPU interrupt control --- */
    case SENS_REG_SAR_COCPU_INT_ENA:
        s->cocpu_int_ena = (uint32_t)value;
        break;

    case SENS_REG_SAR_COCPU_INT_CLR:
        s->cocpu_int_raw &= ~(uint32_t)value;
        break;

    case SENS_REG_COCPU_INT_ENA_W1TS:
        s->cocpu_int_ena |= (uint32_t)value;
        break;

    case SENS_REG_COCPU_INT_ENA_W1TC:
        s->cocpu_int_ena &= ~(uint32_t)value;
        break;

    /* Read-only registers — ignore writes */
    case SENS_REG_SAR_READER1_STATUS:
    case SENS_REG_SAR_READER2_STATUS:
    case SENS_REG_SAR_COCPU_INT_RAW:
    case SENS_REG_SAR_COCPU_INT_ST:
    case SENS_REG_SAR_COCPU_STATE:
    case SENS_REG_SAR_COCPU_DEBUG:
    case SENS_REG_SAR_TOUCH_CHN_ST:
    case SENS_REG_SAR_TOUCH_SLP_STATUS:
    case SENS_REG_SAR_TOUCH_APPR_STATUS:
    case SENS_REG_SARDATE:
        break;

    default:
        if (idx < SENS_REG_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        break;
    }
}

static const MemoryRegionOps esp32s3_sens_ops = {
    .read = esp32s3_sens_read,
    .write = esp32s3_sens_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32s3_sens_reset_hold(Object *obj, ResetType type)
{
    ESP32S3SensState *s = ESP32S3_SENS(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->cocpu_int_raw = 0;
    s->cocpu_int_ena = 0;

    /* Select plausible defaults for measurement control:
     *   SAR_MEAS1_CTRL1: default from TRM is mostly-zero
     *   SAR_TOUCH_CONF: no scan-start, power-down
     */
}

static void esp32s3_sens_init(Object *obj)
{
    ESP32S3SensState *s = ESP32S3_SENS(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_sens_ops, s,
                          TYPE_ESP32S3_SENS, SENS_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32s3_sens_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_sens_reset_hold;
}

static const TypeInfo esp32s3_sens_info = {
    .name          = TYPE_ESP32S3_SENS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3SensState),
    .instance_init = esp32s3_sens_init,
    .class_init    = esp32s3_sens_class_init,
};

static void esp32s3_sens_register_types(void)
{
    type_register_static(&esp32s3_sens_info);
}

type_init(esp32s3_sens_register_types)
