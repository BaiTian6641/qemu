/*
 * ESP32-S3 ULP Coprocessor Stub
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Minimal ULP coprocessor stub providing:
 *  - 8 KiB RTC slow memory region at 0x60021000 (R/W from main CPU)
 *    This memory is shared between the main CPU and the ULP coprocessor.
 *    ESP-IDF loads ULP programs here and reads shared variables back.
 *  - The ULP does not actually execute code in this model — all
 *    instructions are treated as data (NOP execution).
 *
 * Control registers for ULP startup/wakeup reside in RTC_CNTL
 * (already modeled in esp32s3_rtc_cntl.c).
 * ULP interrupt/state registers reside in SENS block
 * (modeled in esp32s3_sens.c: COCPU_STATE/INT_RAW/ENA/ST/CLR).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/esp32s3_ulp.h"

/* ------------------------------------------------------------------ */
/*  RTC Slow Memory I/O                                                */
/* ------------------------------------------------------------------ */

static uint64_t esp32s3_ulp_slowmem_read(void *opaque, hwaddr addr,
                                          unsigned int size)
{
    ESP32S3UlpState *s = ESP32S3_ULP(opaque);

    if (addr + size > ESP32S3_ULP_SLOW_MEM_SIZE) {
        return 0;
    }

    uint64_t val = 0;
    memcpy(&val, &s->slow_mem_data[addr], size);
    return val;
}

static void esp32s3_ulp_slowmem_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned int size)
{
    ESP32S3UlpState *s = ESP32S3_ULP(opaque);

    if (addr + size > ESP32S3_ULP_SLOW_MEM_SIZE) {
        return;
    }

    memcpy(&s->slow_mem_data[addr], &value, size);
}

static const MemoryRegionOps esp32s3_ulp_slowmem_ops = {
    .read = esp32s3_ulp_slowmem_read,
    .write = esp32s3_ulp_slowmem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/*  QOM lifecycle                                                       */
/* ------------------------------------------------------------------ */

static void esp32s3_ulp_reset_hold(Object *obj, ResetType type)
{
    ESP32S3UlpState *s = ESP32S3_ULP(obj);
    memset(s->slow_mem_data, 0, sizeof(s->slow_mem_data));
}

static void esp32s3_ulp_init(Object *obj)
{
    ESP32S3UlpState *s = ESP32S3_ULP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->slowmem, obj, &esp32s3_ulp_slowmem_ops, s,
                          "esp32s3.ulp.slowmem", ESP32S3_ULP_SLOW_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->slowmem);
}

static void esp32s3_ulp_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_ulp_reset_hold;
}

static const TypeInfo esp32s3_ulp_info = {
    .name          = TYPE_ESP32S3_ULP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3UlpState),
    .instance_init = esp32s3_ulp_init,
    .class_init    = esp32s3_ulp_class_init,
};

static void esp32s3_ulp_register_types(void)
{
    type_register_static(&esp32s3_ulp_info);
}

type_init(esp32s3_ulp_register_types)
