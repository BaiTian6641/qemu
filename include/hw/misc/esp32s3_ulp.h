/*
 * ESP32-S3 ULP Coprocessor Stub
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Provides a minimal compatibility model for the ESP32-S3 Ultra-Low-Power
 * coprocessor (ULP-FSM and ULP-RISC-V).  This stub models:
 *
 *  - RTC Slow Memory region at DR_REG_RTC_SLOWMEM_BASE (0x60021000),
 *    accessible from both main CPU and ULP; 8 KiB R/W store.
 *  - The ULP itself does NOT execute code — this is a register-level
 *    compatibility stub so that ESP-IDF apps that check ULP status,
 *    load ULP programs, or read shared variables do not fault.
 *
 * ULP co-processor interrupts and state are exposed via the SENS block
 * (COCPU registers at DR_REG_SENS_BASE + 0xE4..0xF8).
 * ULP startup/control is via RTC_CNTL (already modeled).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_ULP  "esp32s3.ulp"
#define ESP32S3_ULP(obj)  OBJECT_CHECK(ESP32S3UlpState, (obj), TYPE_ESP32S3_ULP)

/*
 * RTC slow memory size: 8 KiB on ESP32-S3
 * Mapped at 0x60021000 (DR_REG_RTC_SLOWMEM_BASE) for main CPU access
 * Also accessible at 0x50000000 from ULP address space (handled by cache model).
 */
#define ESP32S3_ULP_SLOW_MEM_SIZE    (8 * 1024)

typedef struct {
    SysBusDevice parent_obj;

    MemoryRegion slowmem;
    uint8_t slow_mem_data[ESP32S3_ULP_SLOW_MEM_SIZE];
} ESP32S3UlpState;
