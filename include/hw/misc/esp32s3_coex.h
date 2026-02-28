/*
 * ESP32-S3 Wi-Fi / BLE Coexistence Arbitration Model
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * The ESP32-S3 has a single combo PHY (SOC_PHY_COMBO_MODULE=1) shared
 * between Wi-Fi and BLE.  This module provides a coarse time-division
 * arbitration model so that concurrent Wi-Fi and BLE workloads run
 * without deadlocking each other's radio access.
 *
 * Modes:
 *   - BALANCED:           50/50 time share (default)
 *   - WIFI_THROUGHPUT:    75% Wi-Fi / 25% BLE
 *   - BLE_LATENCY:        25% Wi-Fi / 75% BLE
 *
 * The model uses a periodic QEMU timer to alternate radio access
 * between the two subsystems.  During a subsystem's time slot,
 * its DMA/interrupt path runs normally.  During the other's slot,
 * the "off" subsystem's radio operations are deferred until the
 * next slot boundary.
 *
 * For MVP, we simply allow both to operate concurrently (no actual
 * blocking) but track slot state and expose a register interface
 * for the ESP-IDF coexistence adapter.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"
#include "qemu/timer.h"

#define TYPE_ESP32S3_COEX  "esp32s3.coex"
#define ESP32S3_COEX(obj)  OBJECT_CHECK(ESP32S3CoexState, (obj), \
                                        TYPE_ESP32S3_COEX)

/* ------------------------------------------------------------------ */
/*  Coexistence modes                                                  */
/* ------------------------------------------------------------------ */
typedef enum {
    ESP32S3_COEX_MODE_BALANCED = 0,
    ESP32S3_COEX_MODE_WIFI_THROUGHPUT = 1,
    ESP32S3_COEX_MODE_BLE_LATENCY = 2,
} Esp32s3CoexMode;

/* ------------------------------------------------------------------ */
/*  Radio slot assignment                                              */
/* ------------------------------------------------------------------ */
typedef enum {
    COEX_SLOT_WIFI = 0,
    COEX_SLOT_BLE  = 1,
} CoexSlot;

/* ------------------------------------------------------------------ */
/*  QEMU coex register interface (not real hardware)                   */
/*  Mapped as a small register block accessible from machine init.     */
/*  ESP-IDF coex adapter can optionally detect & configure this.       */
/* ------------------------------------------------------------------ */
#define COEX_REG_STATUS      0x00  /* R:  bit0=wifi_active, bit1=ble_active,
                                    *     [3:2]=current_slot (0=wifi,1=ble),
                                    *     [7:4]=mode */
#define COEX_REG_MODE        0x04  /* R/W: coex mode (0=balanced,1=wifi,2=ble) */
#define COEX_REG_SLOT_US     0x08  /* R/W: slot duration in microseconds
                                    *      (default 10000 = 10 ms) */
#define COEX_REG_WIFI_GRANT  0x0C  /* R:   number of Wi-Fi slot grants */
#define COEX_REG_BLE_GRANT   0x10  /* R:   number of BLE slot grants */
#define COEX_REG_CTRL        0x14  /* W:   bit0=enable coex timer,
                                    *      bit1=reset counters */
#define COEX_REG_MAGIC       0x1C  /* R:   0x434F4558 = 'COEX' */

#define COEX_REG_SIZE        0x20

/* Virtual QEMU-only MMIO base for coex model */
#define COEX_MMIO_BASE       0x600D2000

#define COEX_REG_MAGIC_VALUE  0x434F4558  /* 'COEX' */

/* Default slot duration: 10 ms in microseconds */
#define COEX_DEFAULT_SLOT_US  10000

/* Stats */
typedef struct {
    uint64_t wifi_grants;
    uint64_t ble_grants;
    uint64_t total_switches;
} CoexStats;

typedef struct ESP32S3CoexState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* Configuration */
    Esp32s3CoexMode mode;
    uint32_t slot_duration_us;
    bool enabled;

    /* Current state */
    CoexSlot current_slot;
    bool wifi_active;    /* Wi-Fi subsystem has pending radio work */
    bool ble_active;     /* BLE subsystem has pending radio work */

    /* Timer for slot switching */
    QEMUTimer *slot_timer;

    /* Statistics */
    CoexStats stats;

} ESP32S3CoexState;
