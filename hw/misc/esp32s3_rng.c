/*
 * ESP32S3 Random Number Generator peripheral
 *
 * Copyright (c) 2019-2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Supports two modes:
 *  1. Normal mode (default): uses host random via qemu_guest_getrandom
 *  2. Deterministic mode (S8-T2): uses a seeded xorshift64 PRNG for
 *     repeatable CI execution.  Enable with:
 *       -global misc.esp32s3.rng.deterministic=true
 *       -global misc.esp32s3.rng.seed=12345
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/misc/esp32s3_rng.h"

/*
 * xorshift64 — minimal fast PRNG for deterministic mode.
 * Period 2^64 - 1, passes basic randomness tests.
 */
static uint64_t xorshift64(uint64_t *state)
{
    uint64_t x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    return x;
}

static uint64_t esp32s3_rng_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32s3RngState *s = ESP32S3_RNG(opaque);

    if (s->deterministic) {
        return (uint32_t)xorshift64(&s->state);
    }

    uint32_t r = 0;
    qemu_guest_getrandom_nofail(&r, sizeof(r));
    return r;
}

static const MemoryRegionOps esp32s3_rng_ops = {
    .read =  esp32s3_rng_read,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_rng_reset_hold(Object *obj, ResetType type)
{
    Esp32s3RngState *s = ESP32S3_RNG(obj);

    /* Re-seed the PRNG state from the configured seed on every reset */
    s->state = s->seed ? s->seed : 0x5A5A5A5A5A5A5A5AULL;
}

static void esp32s3_rng_init(Object *obj)
{
    Esp32s3RngState *s = ESP32S3_RNG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_rng_ops, s,
                          TYPE_ESP32S3_RNG, sizeof(uint32_t));
    sysbus_init_mmio(sbd, &s->iomem);
}

static Property esp32s3_rng_properties[] = {
    DEFINE_PROP_BOOL("deterministic", Esp32s3RngState, deterministic, false),
    DEFINE_PROP_UINT64("seed", Esp32s3RngState, seed, 0x5A5A5A5A5A5A5A5AULL),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_rng_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_rng_reset_hold;
    device_class_set_props(dc, esp32s3_rng_properties);
}

static const TypeInfo esp32s3_rng_info = {
    .name = TYPE_ESP32S3_RNG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32s3RngState),
    .instance_init = esp32s3_rng_init,
    .class_init = esp32s3_rng_class_init,
};

static void esp32s3_rng_register_types(void)
{
    type_register_static(&esp32s3_rng_info);
}

type_init(esp32s3_rng_register_types)
