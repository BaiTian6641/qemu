/*
 * ESP32-S3 USB OTG (DWC2) Register Stub
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * R/W register store at USB_DWC base (0x60080000), size 0xE08.
 * Returns correct GHWCFG values for ESP32-S3 DWC2 configuration.
 * MVP: absorbs driver register accesses, reports proper HW config.
 * IRQ: ETS_USB_INTR_SOURCE (38).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_USB_OTG  "esp32s3.usb_otg"
#define ESP32S3_USB_OTG(obj)  OBJECT_CHECK(ESP32S3UsbOtgState, (obj), TYPE_ESP32S3_USB_OTG)

/* Base address not in existing reg.h — add DR_REG_USB_DWC_BASE */
#define DR_REG_USB_DWC_BASE   0x60080000

/* Register space: 0xE08 confirmed by _Static_assert(sizeof(usb_dwc_dev_t) == 0xe08) */
#define ESP32S3_USB_OTG_REG_SIZE  0x1000  /* Map full 4K page for alignment */

/* DWC2 global register offsets */
#define USB_GOTGCTL_REG    0x000
#define USB_GOTGINT_REG    0x004
#define USB_GAHBCFG_REG    0x008
#define USB_GUSBCFG_REG    0x00C
#define USB_GRSTCTL_REG    0x010
#define USB_GINTSTS_REG    0x014
#define USB_GINTMSK_REG    0x018
#define USB_GRXSTSR_REG    0x01C
#define USB_GRXSTSP_REG    0x020
#define USB_GRXFSIZ_REG    0x024
#define USB_GNPTXFSIZ_REG  0x028
#define USB_GNPTXSTS_REG   0x02C
#define USB_GSNPSID_REG    0x040
#define USB_GHWCFG1_REG    0x044
#define USB_GHWCFG2_REG    0x048
#define USB_GHWCFG3_REG    0x04C
#define USB_GHWCFG4_REG    0x050
#define USB_GDFIFOCFG_REG  0x05C
#define USB_HPTXFSIZ_REG   0x100
#define USB_HCFG_REG       0x400
#define USB_HFIR_REG       0x404
#define USB_HFNUM_REG      0x408
#define USB_HPRT_REG       0x440
#define USB_DCFG_REG       0x800
#define USB_DCTL_REG       0x804
#define USB_DSTS_REG       0x808
#define USB_PCGCCTL_REG    0xE00

/*
 * ESP32-S3 DWC2 hardware configuration values (from usb_dwc_cfg.h):
 *  OTG_MODE=0 (HNP/SRP), ARCHITECTURE=2 (internal DMA), SINGLE_POINT=1,
 *  FSPHY=1, HSPHY=0, NUM_EPS=6, NUM_IN_EPS=5, NUM_HOST_CHAN=8,
 *  DFIFO_DEPTH=256, EN_DED_TX_FIFO=1, EN_DESC_DMA=1
 */
#define ESP32S3_USB_GSNPSID_VAL   0x4F54400A  /* Synopsys DWC2 ID */
#define ESP32S3_USB_GHWCFG1_VAL   0x00000000  /* IN/OUT endpoint directions */
#define ESP32S3_USB_GHWCFG2_VAL   0x228DDD50  /* FS PHY, internal DMA, 8 host ch, etc. */
#define ESP32S3_USB_GHWCFG3_VAL   0x0FF000E8  /* FIFO depth 256, transfer size width, packet size */
#define ESP32S3_USB_GHWCFG4_VAL   0x1FF00020  /* Dedicated TX FIFO, desc DMA, etc. */

/* GNPTXSTS: non-periodic TX FIFO available (full), queue space available */
#define ESP32S3_USB_GNPTXSTS_VAL  0x00080100

#define ESP32S3_USB_OTG_REGS_COUNT  (ESP32S3_USB_OTG_REG_SIZE / 4)

typedef struct ESP32S3UsbOtgState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[ESP32S3_USB_OTG_REGS_COUNT];
    uint32_t gintsts;
    uint32_t gintmsk;
    uint32_t gahbcfg;
} ESP32S3UsbOtgState;
