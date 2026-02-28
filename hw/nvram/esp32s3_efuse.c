/*
 * ESP32-S3 eFuse emulation
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/nvram/esp32s3_efuse.h"


#define MAC_1_MASK                 0x0000ffffU
#define WAFER_VERSION_MINOR_LO_S   18U
#define WAFER_VERSION_MINOR_LO_M   0x001c0000U
#define BLK_VERSION_MINOR_S        24U
#define BLK_VERSION_MINOR_M        0x07000000U
#define WAFER_VERSION_MINOR_HI_S   23U
#define WAFER_VERSION_MINOR_HI_M   0x00800000U
#define WAFER_VERSION_MAJOR_S      24U
#define WAFER_VERSION_MAJOR_M      0x03000000U

/*
 * eFuse bits in RD_REPEAT_DATA3 (offset 0x3c) that disable USB download modes.
 * The ESP32-S3 ROM download stub computes peripheral base addresses per channel:
 *   channels 0-2 = UART0-2, channel 3 = USB Serial/JTAG, channel 4 = USB OTG.
 * For channels 3/4 the ROM's read function checks a mode flag at (channel_base+8).
 * In QEMU the addresses 0x6003e000 and 0x6004e000 are unmapped, so the flag reads
 * as 0 and the ROM falls through to a UART address formula that produces addresses
 * outside mapped memory (0x5FFE0000 / 0x5FFD0000), causing a LoadStoreError.
 * Setting these eFuse bits tells the ROM to skip USB channels entirely.
 */
#define DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE_S  4U
#define DIS_USB_OTG_DOWNLOAD_MODE_S          31U


static void esp32s3_efuse_realize(DeviceState *dev, Error **errp)
{
    ESP32S3EfuseClass* esp32s3_class = ESP32S3_EFUSE_GET_CLASS(dev);
    ESPEfuseState *s = ESP_EFUSE(dev);
    ESP32S3EfuseState *s3 = ESP32S3_EFUSE(dev);

    esp32s3_class->parent_realize(dev, errp);

    if (s->blk != NULL) {
        return;
    }

    assert(s->mirror != NULL);

    if (s3->has_custom_mac) {
        const uint32_t mac0 = ((uint32_t)s3->custom_mac[5]) |
                              ((uint32_t)s3->custom_mac[4] << 8) |
                              ((uint32_t)s3->custom_mac[3] << 16) |
                              ((uint32_t)s3->custom_mac[2] << 24);
        const uint32_t mac1 = ((uint32_t)s3->custom_mac[1]) |
                              ((uint32_t)s3->custom_mac[0] << 8);

        s->efuses.blocks.rd_mac_spi_sys_0 = mac0;
        s->efuses.blocks.rd_mac_spi_sys_1 = (s->efuses.blocks.rd_mac_spi_sys_1 & ~MAC_1_MASK) | (mac1 & MAC_1_MASK);
    }

    if (s3->has_chip_revision) {
        const uint32_t major = (s3->chip_revision / 100U) & 0x3U;
        const uint32_t minor = s3->chip_revision % 100U;

        s->efuses.blocks.rd_mac_spi_sys_3 =
            (s->efuses.blocks.rd_mac_spi_sys_3 & ~WAFER_VERSION_MINOR_LO_M) |
            (((minor & 0x7U) << WAFER_VERSION_MINOR_LO_S) & WAFER_VERSION_MINOR_LO_M);

        s->efuses.blocks.rd_mac_spi_sys_3 =
            (s->efuses.blocks.rd_mac_spi_sys_3 & ~BLK_VERSION_MINOR_M) |
            (((minor & 0x7U) << BLK_VERSION_MINOR_S) & BLK_VERSION_MINOR_M);

        s->efuses.blocks.rd_mac_spi_sys_5 =
            (s->efuses.blocks.rd_mac_spi_sys_5 & ~WAFER_VERSION_MINOR_HI_M) |
            ((((minor >> 3) & 0x1U) << WAFER_VERSION_MINOR_HI_S) & WAFER_VERSION_MINOR_HI_M);

        s->efuses.blocks.rd_mac_spi_sys_5 =
            (s->efuses.blocks.rd_mac_spi_sys_5 & ~WAFER_VERSION_MAJOR_M) |
            ((major << WAFER_VERSION_MAJOR_S) & WAFER_VERSION_MAJOR_M);
    }

    /*
     * Disable USB download channels: the ROM's download-mode read function
     * does not correctly handle channels 3 (USB Serial/JTAG) and 4 (USB OTG)
     * when the underlying peripheral registers are not fully emulated.
     * Setting these eFuse bits prevents the ROM from entering the USB code
     * paths that would otherwise access unmapped addresses and crash.
     */
    s->efuses.blocks.rd_repeat_data3 |= (1U << DIS_USB_OTG_DOWNLOAD_MODE_S)
                                       | (1U << DIS_USB_SERIAL_JTAG_DOWNLOAD_MODE_S);

    memcpy(s->mirror, &s->efuses.blocks, sizeof(ESPEfuseBlocks));
}


static void esp32s3_efuse_init(Object *obj)
{
}

static void esp32s3_efuse_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ESP32S3EfuseClass* esp32s3_efuse = ESP32S3_EFUSE_CLASS(klass);

    device_class_set_parent_realize(dc, esp32s3_efuse_realize, &esp32s3_efuse->parent_realize);
}

static const TypeInfo esp32s3_efuse_info = {
    .name = TYPE_ESP32S3_EFUSE,
    .parent = TYPE_ESP_EFUSE,
    .instance_size = sizeof(ESP32S3EfuseState),
    .instance_init = esp32s3_efuse_init,
    .class_init = esp32s3_efuse_class_init,
    .class_size = sizeof(ESP32S3EfuseClass)
};

static void esp32s3_efuse_register_types(void)
{
    type_register_static(&esp32s3_efuse_info);
}

type_init(esp32s3_efuse_register_types)
