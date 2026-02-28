/*
 * ESP32-S3 Wi-Fi / BLE Coexistence Arbitration Model
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Implements a coarse time-division arbitration model for the shared
 * combo PHY on ESP32-S3.  A periodic timer alternates between Wi-Fi
 * and BLE radio slots.
 *
 * In BALANCED mode (default), slots alternate evenly.
 * In WIFI_THROUGHPUT mode, Wi-Fi gets 3 of every 4 slots.
 * In BLE_LATENCY mode, BLE gets 3 of every 4 slots.
 *
 * MVP: Both subsystems operate concurrently without actual blocking.
 * The model tracks slot state and counters for deterministic testing
 * and exposes a small MMIO register interface.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/esp32s3_coex.h"

/* ------------------------------------------------------------------ */
/*  Slot timer callback                                                */
/* ------------------------------------------------------------------ */

static void coex_slot_timer_cb(void *opaque)
{
    ESP32S3CoexState *s = ESP32S3_COEX(opaque);

    if (!s->enabled) {
        return;
    }

    s->stats.total_switches++;

    /*
     * Determine next slot based on mode and current slot.
     *
     * BALANCED:         WIFI → BLE → WIFI → BLE ...
     * WIFI_THROUGHPUT:  WIFI → WIFI → WIFI → BLE → repeat
     * BLE_LATENCY:      BLE → BLE → BLE → WIFI → repeat
     */
    switch (s->mode) {
    case ESP32S3_COEX_MODE_BALANCED:
        s->current_slot = (s->current_slot == COEX_SLOT_WIFI)
                          ? COEX_SLOT_BLE : COEX_SLOT_WIFI;
        break;

    case ESP32S3_COEX_MODE_WIFI_THROUGHPUT:
        /*
         * 4-slot cycle: W W W B.
         * Use total_switches modulo 4 to decide.
         */
        s->current_slot = ((s->stats.total_switches % 4) == 3)
                          ? COEX_SLOT_BLE : COEX_SLOT_WIFI;
        break;

    case ESP32S3_COEX_MODE_BLE_LATENCY:
        /* 4-slot cycle: B B B W */
        s->current_slot = ((s->stats.total_switches % 4) == 3)
                          ? COEX_SLOT_WIFI : COEX_SLOT_BLE;
        break;
    }

    /* Update grant counters */
    if (s->current_slot == COEX_SLOT_WIFI) {
        s->stats.wifi_grants++;
    } else {
        s->stats.ble_grants++;
    }

    /* Re-arm timer */
    int64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    timer_mod(s->slot_timer,
              now + (int64_t)s->slot_duration_us);
}

/* ------------------------------------------------------------------ */
/*  Register I/O                                                       */
/* ------------------------------------------------------------------ */

static uint64_t esp32s3_coex_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3CoexState *s = ESP32S3_COEX(opaque);

    switch (addr) {
    case COEX_REG_STATUS: {
        uint32_t val = 0;
        val |= (s->wifi_active ? BIT(0) : 0);
        val |= (s->ble_active  ? BIT(1) : 0);
        val |= ((uint32_t)s->current_slot << 2);
        val |= ((uint32_t)s->mode << 4);
        return val;
    }

    case COEX_REG_MODE:
        return (uint32_t)s->mode;

    case COEX_REG_SLOT_US:
        return s->slot_duration_us;

    case COEX_REG_WIFI_GRANT:
        return (uint32_t)(s->stats.wifi_grants & 0xFFFFFFFF);

    case COEX_REG_BLE_GRANT:
        return (uint32_t)(s->stats.ble_grants & 0xFFFFFFFF);

    case COEX_REG_CTRL:
        return s->enabled ? 1 : 0;

    case COEX_REG_MAGIC:
        return COEX_REG_MAGIC_VALUE;

    default:
        return 0;
    }
}

static void esp32s3_coex_write(void *opaque, hwaddr addr,
                                uint64_t value, unsigned int size)
{
    ESP32S3CoexState *s = ESP32S3_COEX(opaque);

    switch (addr) {
    case COEX_REG_MODE:
        if (value <= ESP32S3_COEX_MODE_BLE_LATENCY) {
            s->mode = (Esp32s3CoexMode)value;
        }
        break;

    case COEX_REG_SLOT_US:
        if (value >= 100 && value <= 1000000) {
            s->slot_duration_us = (uint32_t)value;
        }
        break;

    case COEX_REG_CTRL:
        /* bit 0: enable/disable coex timer */
        if (value & BIT(0)) {
            if (!s->enabled) {
                s->enabled = true;
                int64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
                timer_mod(s->slot_timer,
                          now + (int64_t)s->slot_duration_us);
            }
        } else {
            s->enabled = false;
            timer_del(s->slot_timer);
        }
        /* bit 1: reset counters */
        if (value & BIT(1)) {
            s->stats.wifi_grants = 0;
            s->stats.ble_grants = 0;
            s->stats.total_switches = 0;
        }
        break;

    default:
        break;
    }
}

static const MemoryRegionOps esp32s3_coex_ops = {
    .read = esp32s3_coex_read,
    .write = esp32s3_coex_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/*  QOM lifecycle                                                       */
/* ------------------------------------------------------------------ */

static void esp32s3_coex_reset_hold(Object *obj, ResetType type)
{
    ESP32S3CoexState *s = ESP32S3_COEX(obj);

    s->mode = ESP32S3_COEX_MODE_BALANCED;
    s->slot_duration_us = COEX_DEFAULT_SLOT_US;
    s->enabled = true;
    s->current_slot = COEX_SLOT_WIFI;
    s->wifi_active = false;
    s->ble_active = false;

    memset(&s->stats, 0, sizeof(s->stats));

    if (s->slot_timer) {
        timer_del(s->slot_timer);
    }
}

static void esp32s3_coex_realize(DeviceState *dev, Error **errp)
{
    ESP32S3CoexState *s = ESP32S3_COEX(dev);

    s->slot_timer = timer_new_us(QEMU_CLOCK_VIRTUAL, coex_slot_timer_cb, s);
    if (s->enabled) {
        int64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
        timer_mod(s->slot_timer, now + (int64_t)s->slot_duration_us);
    }
}

static void esp32s3_coex_init(Object *obj)
{
    ESP32S3CoexState *s = ESP32S3_COEX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_coex_ops, s,
                          TYPE_ESP32S3_COEX, COEX_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32s3_coex_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_coex_reset_hold;
    dc->realize = esp32s3_coex_realize;
}

static const TypeInfo esp32s3_coex_info = {
    .name          = TYPE_ESP32S3_COEX,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3CoexState),
    .instance_init = esp32s3_coex_init,
    .class_init    = esp32s3_coex_class_init,
};

static void esp32s3_coex_register_types(void)
{
    type_register_static(&esp32s3_coex_info);
}

type_init(esp32s3_coex_register_types)
