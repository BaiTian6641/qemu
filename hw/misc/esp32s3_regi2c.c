/*
 * ESP32-S3 REGI2C analog bus stub
 *
 * Absorbs ROM/ESP-IDF BBPLL, SAR ADC, and analog calibration register
 * accesses that target the I2C_ANA_MST window at 0x6000E000.
 * The key behaviour: ANA_CONF0 bit 24 (BBPLL_CAL_DONE) is always set
 * on read, preventing infinite polling loops in phy_bbpll_en_usb().
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/misc/esp32s3_regi2c.h"

#define REGI2C_DEBUG    0
#define REGI2C_WARNING  0


static uint64_t esp32s3_regi2c_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3RegI2CState *s = ESP32S3_REGI2C(opaque);
    uint64_t r = 0;

    switch (addr) {
    case A_REGI2C_ANA_CONF0:
        /* Always report BBPLL_CAL_DONE so calibration polling finishes */
        r = s->ana_conf0 | REGI2C_BBPLL_CAL_DONE;
        break;
    case A_REGI2C_ANA_CONFIG:
        r = s->ana_config;
        break;
    case A_REGI2C_ANA_CONFIG2:
        r = s->ana_config2;
        break;
    default:
        if (addr < ESP32S3_REGI2C_REG_SIZE) {
            r = s->regs[addr / 4];
        }
#if REGI2C_WARNING
        warn_report("[REGI2C] Unsupported read from 0x%03lx\n", (unsigned long)addr);
#endif
        break;
    }

#if REGI2C_DEBUG
    info_report("[REGI2C] Read  0x%03lx => 0x%08lx", (unsigned long)addr, (unsigned long)r);
#endif
    return r;
}


static void esp32s3_regi2c_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size)
{
    ESP32S3RegI2CState *s = ESP32S3_REGI2C(opaque);

#if REGI2C_DEBUG
    info_report("[REGI2C] Write 0x%03lx <= 0x%08lx", (unsigned long)addr, (unsigned long)value);
#endif

    switch (addr) {
    case A_REGI2C_ANA_CONF0:
        s->ana_conf0 = (uint32_t)value;
        break;
    case A_REGI2C_ANA_CONFIG:
        s->ana_config = (uint32_t)value;
        break;
    case A_REGI2C_ANA_CONFIG2:
        s->ana_config2 = (uint32_t)value;
        break;
    default:
        if (addr < ESP32S3_REGI2C_REG_SIZE) {
            s->regs[addr / 4] = (uint32_t)value;
        }
#if REGI2C_WARNING
        warn_report("[REGI2C] Unsupported write to 0x%03lx (0x%08lx)\n",
                    (unsigned long)addr, (unsigned long)value);
#endif
        break;
    }
}


static const MemoryRegionOps esp32s3_regi2c_ops = {
    .read  = esp32s3_regi2c_read,
    .write = esp32s3_regi2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};


static void esp32s3_regi2c_reset_hold(Object *obj, ResetType type)
{
    ESP32S3RegI2CState *s = ESP32S3_REGI2C(obj);

    s->ana_conf0  = 0;
    s->ana_config = 0;
    s->ana_config2 = 0;
    memset(s->regs, 0, sizeof(s->regs));
}


static void esp32s3_regi2c_realize(DeviceState *dev, Error **errp)
{
    esp32s3_regi2c_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}


static void esp32s3_regi2c_init(Object *obj)
{
    ESP32S3RegI2CState *s = ESP32S3_REGI2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_regi2c_ops, s,
                          TYPE_ESP32S3_REGI2C, ESP32S3_REGI2C_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}


static void esp32s3_regi2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_regi2c_reset_hold;
    dc->realize = esp32s3_regi2c_realize;
}


static const TypeInfo esp32s3_regi2c_info = {
    .name = TYPE_ESP32S3_REGI2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3RegI2CState),
    .instance_init = esp32s3_regi2c_init,
    .class_init = esp32s3_regi2c_class_init,
    .class_size = sizeof(ESP32S3RegI2CClass),
};

static void esp32s3_regi2c_register_types(void)
{
    type_register_static(&esp32s3_regi2c_info);
}

type_init(esp32s3_regi2c_register_types)
