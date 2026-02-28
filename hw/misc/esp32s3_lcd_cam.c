/*
 * ESP32-S3 LCD_CAM Controller Model
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Refined model: full register-level semantics matching ESP-IDF lcd_ll.h.
 *   - Correct bit positions for LCD_START/RESET/UPDATE and CAM counterparts
 *   - GDMA TX integration for I80 and RGB LCD paths
 *   - Self-clearing bits for all WO/auto-clear fields
 *   - Proper reset defaults (lcd_afifo_threshold_num, DATE)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32s3_lcd_cam.h"
#include "hw/dma/esp_gdma.h"

#define LCD_CAM_WARN  0
#define LCD_CAM_DEBUG 0

#if LCD_CAM_DEBUG
#define DPRINTF(fmt, ...) qemu_log_mask(LOG_GUEST_ERROR, \
    "%s: " fmt, __func__, ## __VA_ARGS__)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

/* Maximum size of a single GDMA transfer we'll consume (64 KiB) */
#define LCD_CAM_MAX_DMA_XFER  (64 * 1024)

/* ------------------------------------------------------------------ */
/*  Interrupt helpers                                                  */
/* ------------------------------------------------------------------ */

static void esp32s3_lcd_cam_update_irq(ESP32S3LcdCamState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena) ? 1 : 0);
}

/* ------------------------------------------------------------------ */
/*  GDMA helpers                                                       */
/* ------------------------------------------------------------------ */

/**
 * Consume TX data from the GDMA channel bound to LCDCAM and signal
 * LCD_TRANS_DONE.  This is the behaviour expected by the ESP-IDF
 * i80 driver: GDMA feeds data → LCD peripheral shifts it out → TRANS_DONE.
 *
 * We don't model actual pixel shifting — we just drain the DMA chain
 * so descriptor ownership flips and the GDMA OUT_EOF/OUT_TOTAL_EOF
 * interrupts fire, then raise LCD_TRANS_DONE on the LCD_CAM side.
 */
static void lcd_cam_do_lcd_dma_transfer(ESP32S3LcdCamState *s)
{
    if (!s->gdma) {
        /* No GDMA link — just fire TRANS_DONE immediately (legacy MVP) */
        s->int_raw |= LCD_CAM_INT_LCD_TRANS_DONE;
        esp32s3_lcd_cam_update_irq(s);
        return;
    }

    uint32_t chan = 0;
    if (!esp_gdma_get_channel_periph(s->gdma, GDMA_LCDCAM,
                                     ESP_GDMA_OUT_IDX, &chan)) {
        /* LCDCAM not yet assigned to any GDMA channel — immediate done */
#if LCD_CAM_WARN
        qemu_log_mask(LOG_UNIMP,
            "esp32s3_lcd_cam: LCD_START but no GDMA channel bound to LCDCAM\n");
#endif
        s->int_raw |= LCD_CAM_INT_LCD_TRANS_DONE;
        esp32s3_lcd_cam_update_irq(s);
        return;
    }

    /*
     * Read (drain) data from the GDMA TX (OUT) channel.
     * We don't need the actual pixel bytes — we just need GDMA to walk
     * its descriptor chain so that OUT_EOF triggers on the DMA side.
     * Use a modest buffer to avoid huge allocations.
     */
    uint8_t sink[4096];
    bool more = true;
    while (more) {
        more = esp_gdma_read_channel(s->gdma, chan, sink, sizeof(sink));
    }

    /* Now raise LCD_TRANS_DONE */
    s->int_raw |= LCD_CAM_INT_LCD_TRANS_DONE;
    esp32s3_lcd_cam_update_irq(s);

    DPRINTF("LCD DMA transfer done via GDMA channel %u\n", chan);
}

/* ------------------------------------------------------------------ */
/*  MMIO read                                                          */
/* ------------------------------------------------------------------ */

static uint64_t esp32s3_lcd_cam_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    ESP32S3LcdCamState *s = ESP32S3_LCD_CAM(opaque);

    switch (addr) {
    case LCD_CAM_LC_DMA_INT_RAW_REG:
        return s->int_raw;
    case LCD_CAM_LC_DMA_INT_ST_REG:
        return s->int_raw & s->int_ena;
    case LCD_CAM_LC_DMA_INT_ENA_REG:
        return s->int_ena;
    case LCD_CAM_LC_DMA_INT_CLR_REG:
        return 0;  /* write-only register */
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_LCD_CAM_REGS_COUNT) {
            return s->regs[idx];
        }
        qemu_log_mask(LOG_UNIMP,
            "esp32s3_lcd_cam: unhandled read at 0x%03" HWADDR_PRIx "\n", addr);
        return 0;
    }
    }
}

/* ------------------------------------------------------------------ */
/*  MMIO write                                                         */
/* ------------------------------------------------------------------ */

static void esp32s3_lcd_cam_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned int size)
{
    ESP32S3LcdCamState *s = ESP32S3_LCD_CAM(opaque);
    uint32_t val = (uint32_t)value;

    switch (addr) {

    /* ---- Interrupt registers ---- */
    case LCD_CAM_LC_DMA_INT_CLR_REG:
        s->int_raw &= ~val;
        esp32s3_lcd_cam_update_irq(s);
        return;

    case LCD_CAM_LC_DMA_INT_ENA_REG:
        s->int_ena = val;
        esp32s3_lcd_cam_update_irq(s);
        return;

    case LCD_CAM_LC_DMA_INT_RAW_REG:
        /* RAW register is RO in hardware; absorb writes silently */
        return;

    /* ---- LCD_USER_REG (0x14) ---- */
    case LCD_CAM_LCD_USER_REG: {
        /* LCD_UPDATE (bit 20): self-clearing, latches shadow regs.
         * In simulation we just clear it immediately. */
        if (val & LCD_CAM_LCD_UPDATE) {
            val &= ~LCD_CAM_LCD_UPDATE;
        }

        /* LCD_RESET (bit 28): self-clearing, resets LCD TX state machine */
        if (val & LCD_CAM_LCD_RESET) {
            val &= ~LCD_CAM_LCD_RESET;
            /* Clear any pending LCD interrupt bits */
            s->int_raw &= ~(LCD_CAM_INT_LCD_VSYNC | LCD_CAM_INT_LCD_TRANS_DONE);
            esp32s3_lcd_cam_update_irq(s);
        }

        /* LCD_START (bit 27): in I80 mode this triggers a transaction.
         * The driver sets LCD_START=1, GDMA pushes data, then ISR checks
         * LCD_TRANS_DONE.  We drain the GDMA chain and fire TRANS_DONE.
         *
         * In RGB continuous mode (lcd_always_out_en=1), LCD_START stays
         * high; we still fire a single TRANS_DONE per write to unblock
         * the driver's first-frame handshake. */
        if (val & LCD_CAM_LCD_START) {
            lcd_cam_do_lcd_dma_transfer(s);
            /* In non-continuous (I80) mode, HW clears LCD_START after
             * the transaction completes.  In continuous mode it stays set.
             * The ESP-IDF i80 driver does not poll LCD_START so clearing
             * it is safe for both paths. */
            if (!(val & LCD_CAM_LCD_ALWAYS_OUT_EN)) {
                val &= ~LCD_CAM_LCD_START;
            }
        }

        s->regs[addr / 4] = val;
        return;
    }

    /* ---- LCD_MISC_REG (0x18) ---- */
    case LCD_CAM_LCD_MISC_REG: {
        /* LCD_AFIFO_RESET (bit 27): self-clearing */
        if (val & LCD_CAM_LCD_AFIFO_RESET) {
            val &= ~LCD_CAM_LCD_AFIFO_RESET;
        }
        s->regs[addr / 4] = val;
        return;
    }

    /* ---- CAM_CTRL_REG (0x04) ---- */
    case LCD_CAM_CAM_CTRL_REG: {
        /* CAM_UPDATE (bit 4): self-clearing */
        if (val & LCD_CAM_CAM_UPDATE) {
            val &= ~LCD_CAM_CAM_UPDATE;
        }
        s->regs[addr / 4] = val;
        return;
    }

    /* ---- CAM_CTRL1_REG (0x08) ---- */
    case LCD_CAM_CAM_CTRL1_REG: {
        /* CAM_START (bit 29): stays set in HW while capturing */
        /* CAM_RESET (bit 30): self-clearing */
        if (val & LCD_CAM_CAM_RESET) {
            val &= ~LCD_CAM_CAM_RESET;
            s->int_raw &= ~(LCD_CAM_INT_CAM_VSYNC | LCD_CAM_INT_CAM_HS);
            esp32s3_lcd_cam_update_irq(s);
        }
        /* CAM_AFIFO_RESET (bit 31): self-clearing */
        if (val & LCD_CAM_CAM_AFIFO_RESET) {
            val &= ~LCD_CAM_CAM_AFIFO_RESET;
        }
        s->regs[addr / 4] = val;
        return;
    }

    /* ---- LCD_CLOCK_REG (0x00) ---- */
    case LCD_CAM_LCD_CLOCK_REG:
        /* All bits are plain R/W; just store */
        s->regs[addr / 4] = val;
        return;

    /* ---- Default: generic R/W register store ---- */
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_LCD_CAM_REGS_COUNT) {
            s->regs[idx] = val;
        } else {
            qemu_log_mask(LOG_UNIMP,
                "esp32s3_lcd_cam: unhandled write at 0x%03" HWADDR_PRIx
                " val=0x%08x\n", addr, val);
        }
        return;
    }
    }
}

/* ------------------------------------------------------------------ */
/*  Memory region ops                                                  */
/* ------------------------------------------------------------------ */

static const MemoryRegionOps esp32s3_lcd_cam_ops = {
    .read  = esp32s3_lcd_cam_read,
    .write = esp32s3_lcd_cam_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/*  QOM lifecycle                                                      */
/* ------------------------------------------------------------------ */

static void esp32s3_lcd_cam_init(Object *obj)
{
    ESP32S3LcdCamState *s = ESP32S3_LCD_CAM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_lcd_cam_ops, s,
                          TYPE_ESP32S3_LCD_CAM, ESP32S3_LCD_CAM_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_lcd_cam_reset_hold(Object *obj, ResetType type)
{
    ESP32S3LcdCamState *s = ESP32S3_LCD_CAM(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->int_raw = 0;
    s->int_ena = 0;

    /* Apply reset defaults from TRM */
    /* LCD_MISC_REG: lcd_afifo_threshold_num = 17  (bits [5:1]) */
    s->regs[LCD_CAM_LCD_MISC_REG / 4] = LCD_CAM_LCD_MISC_RESET_VAL;

    /* DATE register default */
    s->regs[LCD_CAM_LC_REG_DATE_REG / 4] = LCD_CAM_DATE_RESET_VAL;

    qemu_set_irq(s->irq, 0);
}

static void esp32s3_lcd_cam_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_lcd_cam_reset_hold;
}

static const TypeInfo esp32s3_lcd_cam_info = {
    .name          = TYPE_ESP32S3_LCD_CAM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3LcdCamState),
    .instance_init = esp32s3_lcd_cam_init,
    .class_init    = esp32s3_lcd_cam_class_init,
};

static void esp32s3_lcd_cam_register_types(void)
{
    type_register_static(&esp32s3_lcd_cam_info);
}

type_init(esp32s3_lcd_cam_register_types);
