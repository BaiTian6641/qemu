/*
 * ESP32-S3 random number generator
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"

#define DR_REG_WDEV_BASE                        0x3ff75000

#define TYPE_ESP32S3_RNG "misc.esp32s3.rng"
#define ESP32S3_RNG(obj) OBJECT_CHECK(Esp32s3RngState, (obj), TYPE_ESP32S3_RNG)

typedef struct Esp32s3RngState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* Deterministic mode (S8-T2): when enabled, use a seeded PRNG
     * instead of host random for repeatable CI test execution.
     * Controlled by -global misc.esp32s3.rng.deterministic=true
     * and -global misc.esp32s3.rng.seed=<uint64>
     */
    bool     deterministic;
    uint64_t seed;
    uint64_t state;   /* xorshift64 state */
} Esp32s3RngState;

#define ESP32S3_RNG_BASE 0x6003507C//(DR_REG_WDEV_BASE + 0x07c)

