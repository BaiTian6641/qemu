/*
 * ESP32-S3 USB OTG (DWC2) Register Stub
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Sprint S7: R/W register store returning proper ESP32-S3 DWC2 HW config.
 * Returns correct GSNPSID, GHWCFG1-4 for ESP32-S3 USB peripheral.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32s3_usb_otg.h"

static void esp32s3_usb_otg_update_irq(ESP32S3UsbOtgState *s)
{
    bool level = (s->gintsts & s->gintmsk) && (s->gahbcfg & BIT(0));
    qemu_set_irq(s->irq, level ? 1 : 0);
}

static uint64_t esp32s3_usb_otg_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    ESP32S3UsbOtgState *s = ESP32S3_USB_OTG(opaque);

    switch (addr) {
    /* Read-only hardware configuration registers */
    case USB_GSNPSID_REG:
        return ESP32S3_USB_GSNPSID_VAL;
    case USB_GHWCFG1_REG:
        return ESP32S3_USB_GHWCFG1_VAL;
    case USB_GHWCFG2_REG:
        return ESP32S3_USB_GHWCFG2_VAL;
    case USB_GHWCFG3_REG:
        return ESP32S3_USB_GHWCFG3_VAL;
    case USB_GHWCFG4_REG:
        return ESP32S3_USB_GHWCFG4_VAL;

    /* Interrupt/control registers with dedicated state */
    case USB_GINTSTS_REG:
        return s->gintsts;
    case USB_GINTMSK_REG:
        return s->gintmsk;
    case USB_GAHBCFG_REG:
        return s->gahbcfg;

    /* GNPTXSTS: non-periodic TX FIFO status (always available) */
    case USB_GNPTXSTS_REG:
        return ESP32S3_USB_GNPTXSTS_VAL;

    /* DSTS: device status — report FS speed, frame number 0 */
    case USB_DSTS_REG:
        return (0x1 << 1);  /* ENUMSPD = 0x01 (FS with FS PHY) */

    /* GRSTCTL: report AHB idle + DMA idle */
    case USB_GRSTCTL_REG:
        return BIT(31) | BIT(30);  /* AHBIdle | DMAReq */

    /* HFNUM: frame number register (host mode) */
    case USB_HFNUM_REG:
        return 0;

    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_USB_OTG_REGS_COUNT) {
            return s->regs[idx];
        }
        return 0;
    }
    }
}

static void esp32s3_usb_otg_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned int size)
{
    ESP32S3UsbOtgState *s = ESP32S3_USB_OTG(opaque);

    switch (addr) {
    case USB_GINTSTS_REG:
        /* W1C for interrupt status bits */
        s->gintsts &= ~(uint32_t)value;
        esp32s3_usb_otg_update_irq(s);
        return;
    case USB_GINTMSK_REG:
        s->gintmsk = (uint32_t)value;
        esp32s3_usb_otg_update_irq(s);
        return;
    case USB_GAHBCFG_REG:
        s->gahbcfg = (uint32_t)value;
        esp32s3_usb_otg_update_irq(s);
        return;
    case USB_GRSTCTL_REG: {
        /* Core soft reset (bit 0) is self-clearing */
        uint32_t val = (uint32_t)value;
        if (val & BIT(0)) {
            /* Soft reset: clear pending interrupts */
            s->gintsts = 0;
            esp32s3_usb_otg_update_irq(s);
        }
        /* DMA req and AHB idle bits auto-set on read; don't store reset bits */
        return;
    }
    /* Read-only registers — silently ignore writes */
    case USB_GSNPSID_REG:
    case USB_GHWCFG1_REG:
    case USB_GHWCFG2_REG:
    case USB_GHWCFG3_REG:
    case USB_GHWCFG4_REG:
    case USB_GNPTXSTS_REG:
    case USB_DSTS_REG:
        return;
    default: {
        uint32_t idx = addr / 4;
        if (idx < ESP32S3_USB_OTG_REGS_COUNT) {
            s->regs[idx] = (uint32_t)value;
        }
        return;
    }
    }
}

static const MemoryRegionOps esp32s3_usb_otg_ops = {
    .read  = esp32s3_usb_otg_read,
    .write = esp32s3_usb_otg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void esp32s3_usb_otg_init(Object *obj)
{
    ESP32S3UsbOtgState *s = ESP32S3_USB_OTG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_usb_otg_ops, s,
                          TYPE_ESP32S3_USB_OTG, ESP32S3_USB_OTG_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_usb_otg_reset_hold(Object *obj, ResetType type)
{
    ESP32S3UsbOtgState *s = ESP32S3_USB_OTG(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->gintsts = 0;
    s->gintmsk = 0;
    s->gahbcfg = 0;

    /* Set GRXFSIZ default (256 words) */
    s->regs[USB_GRXFSIZ_REG / 4] = 0x100;
    /* Set GNPTXFSIZ default */
    s->regs[USB_GNPTXFSIZ_REG / 4] = 0x01000100;

    qemu_set_irq(s->irq, 0);
}

static void esp32s3_usb_otg_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_usb_otg_reset_hold;
}

static const TypeInfo esp32s3_usb_otg_info = {
    .name          = TYPE_ESP32S3_USB_OTG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3UsbOtgState),
    .instance_init = esp32s3_usb_otg_init,
    .class_init    = esp32s3_usb_otg_class_init,
};

static void esp32s3_usb_otg_register_types(void)
{
    type_register_static(&esp32s3_usb_otg_info);
}

type_init(esp32s3_usb_otg_register_types);
