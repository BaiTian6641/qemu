/*
 * ESP32-S3 SYSCON (APB_CTRL) register model
 *
 * Replaces the previous RAM-hack at DR_REG_APB_CTRL_BASE (0x60026000).
 * Full read/write register model covering all SYSCON registers documented
 * in the ESP32-S3 TRM Chapter 8 and the ESP-IDF syscon_reg.h / syscon_struct.h.
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
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/esp32s3_syscon.h"

#define SYSCON_DEBUG      0
#define SYSCON_WARNING    0


static uint64_t esp32s3_syscon_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3SysconState *s = ESP32S3_SYSCON(opaque);
    uint64_t r = 0;

    switch (addr) {
    case A_SYSCON_SYSCLK_CONF:
        r = s->clk_conf;
        break;
    case A_SYSCON_TICK_CONF:
        r = s->tick_conf;
        break;
    case A_SYSCON_CLK_OUT_EN:
        r = s->clk_out_en;
        break;
    case A_SYSCON_WIFI_BB_CFG:
        r = s->wifi_bb_cfg;
        break;
    case A_SYSCON_WIFI_BB_CFG_2:
        r = s->wifi_bb_cfg_2;
        break;
    case A_SYSCON_WIFI_CLK_EN:
        r = s->wifi_clk_en;
        break;
    case A_SYSCON_WIFI_RST_EN:
        r = s->wifi_rst_en;
        break;
    case A_SYSCON_HOST_INF_SEL:
        r = s->host_inf_sel;
        break;
    case A_SYSCON_EXT_MEM_PMS_LOCK:
        r = s->ext_mem_pms_lock;
        break;
    case A_SYSCON_EXT_MEM_WRITEBACK_BYPASS:
        r = s->ext_mem_writeback_bypass;
        break;

    /* Flash ACE registers */
    case A_SYSCON_FLASH_ACE0_ATTR ... A_SYSCON_FLASH_ACE3_ATTR:
        r = s->flash_ace_attr[(addr - A_SYSCON_FLASH_ACE0_ATTR) / 4];
        break;
    case A_SYSCON_FLASH_ACE0_ADDR ... A_SYSCON_FLASH_ACE3_ADDR:
        r = s->flash_ace_addr[(addr - A_SYSCON_FLASH_ACE0_ADDR) / 4];
        break;
    case A_SYSCON_FLASH_ACE0_SIZE ... A_SYSCON_FLASH_ACE3_SIZE:
        r = s->flash_ace_size[(addr - A_SYSCON_FLASH_ACE0_SIZE) / 4];
        break;

    /* SRAM ACE registers */
    case A_SYSCON_SRAM_ACE0_ATTR ... A_SYSCON_SRAM_ACE3_ATTR:
        r = s->sram_ace_attr[(addr - A_SYSCON_SRAM_ACE0_ATTR) / 4];
        break;
    case A_SYSCON_SRAM_ACE0_ADDR ... A_SYSCON_SRAM_ACE3_ADDR:
        r = s->sram_ace_addr[(addr - A_SYSCON_SRAM_ACE0_ADDR) / 4];
        break;
    case A_SYSCON_SRAM_ACE0_SIZE ... A_SYSCON_SRAM_ACE3_SIZE:
        r = s->sram_ace_size[(addr - A_SYSCON_SRAM_ACE0_SIZE) / 4];
        break;

    case A_SYSCON_SPI_MEM_PMS_CTRL:
        r = s->spi_mem_pms_ctrl;
        break;
    case A_SYSCON_SPI_MEM_REJECT_ADDR:
        r = s->spi_mem_reject_addr;  /* read-only */
        break;
    case A_SYSCON_SDIO_CTRL:
        r = s->sdio_ctrl;
        break;
    case A_SYSCON_REDCY_SIG0:
        r = s->redcy_sig0;
        break;
    case A_SYSCON_REDCY_SIG1:
        r = s->redcy_sig1;
        break;
    case A_SYSCON_FRONT_END_MEM_PD:
        r = s->front_end_mem_pd;
        break;
    case A_SYSCON_SPI_MEM_ECC_CTRL:
        r = s->spi_mem_ecc_ctrl;
        break;
    case A_SYSCON_CLKGATE_FORCE_ON:
        r = s->clkgate_force_on;
        break;
    case A_SYSCON_MEM_POWER_DOWN:
        r = s->mem_power_down;
        break;
    case A_SYSCON_MEM_POWER_UP:
        r = s->mem_power_up;
        break;
    case A_SYSCON_RETENTION_CTRL:
        r = s->retention_ctrl;
        break;
    case A_SYSCON_RETENTION_CTRL1:
        r = s->retention_ctrl1;
        break;
    case A_SYSCON_RETENTION_CTRL2:
        r = s->retention_ctrl2;
        break;
    case A_SYSCON_RETENTION_CTRL3:
        r = s->retention_ctrl3;
        break;
    case A_SYSCON_RETENTION_CTRL4:
        r = s->retention_ctrl4;
        break;
    case A_SYSCON_RETENTION_CTRL5:
        r = s->retention_ctrl5;
        break;

    case RGB_QEMU_ORIGIN_REG:
        r = s->qemu_origin;
        break;

    case A_SYSCON_DATE:
        r = s->date;
        break;

    default:
#if SYSCON_WARNING
        warn_report("[SYSCON] Unsupported read from 0x%03lx\n", (unsigned long)addr);
#endif
        break;
    }

#if SYSCON_DEBUG
    info_report("[SYSCON] Read  0x%03lx => 0x%08lx", (unsigned long)addr, (unsigned long)r);
#endif
    return r;
}


static void esp32s3_syscon_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size)
{
    ESP32S3SysconState *s = ESP32S3_SYSCON(opaque);

#if SYSCON_DEBUG
    info_report("[SYSCON] Write 0x%03lx <= 0x%08lx", (unsigned long)addr, (unsigned long)value);
#endif

    switch (addr) {
    case A_SYSCON_SYSCLK_CONF:
        s->clk_conf = (uint32_t)value;
        break;
    case A_SYSCON_TICK_CONF:
        s->tick_conf = (uint32_t)value;
        break;
    case A_SYSCON_CLK_OUT_EN:
        s->clk_out_en = (uint32_t)value;
        break;
    case A_SYSCON_WIFI_BB_CFG:
        s->wifi_bb_cfg = (uint32_t)value;
        break;
    case A_SYSCON_WIFI_BB_CFG_2:
        s->wifi_bb_cfg_2 = (uint32_t)value;
        break;
    case A_SYSCON_WIFI_CLK_EN:
        s->wifi_clk_en = (uint32_t)value;
        break;
    case A_SYSCON_WIFI_RST_EN:
        s->wifi_rst_en = (uint32_t)value;
        break;
    case A_SYSCON_HOST_INF_SEL:
        s->host_inf_sel = (uint32_t)value;
        break;
    case A_SYSCON_EXT_MEM_PMS_LOCK:
        s->ext_mem_pms_lock = (uint32_t)value;
        break;
    case A_SYSCON_EXT_MEM_WRITEBACK_BYPASS:
        s->ext_mem_writeback_bypass = (uint32_t)value;
        break;

    /* Flash ACE registers */
    case A_SYSCON_FLASH_ACE0_ATTR ... A_SYSCON_FLASH_ACE3_ATTR:
        s->flash_ace_attr[(addr - A_SYSCON_FLASH_ACE0_ATTR) / 4] = (uint32_t)value;
        break;
    case A_SYSCON_FLASH_ACE0_ADDR ... A_SYSCON_FLASH_ACE3_ADDR:
        s->flash_ace_addr[(addr - A_SYSCON_FLASH_ACE0_ADDR) / 4] = (uint32_t)value;
        break;
    case A_SYSCON_FLASH_ACE0_SIZE ... A_SYSCON_FLASH_ACE3_SIZE:
        s->flash_ace_size[(addr - A_SYSCON_FLASH_ACE0_SIZE) / 4] = (uint32_t)value;
        break;

    /* SRAM ACE registers */
    case A_SYSCON_SRAM_ACE0_ATTR ... A_SYSCON_SRAM_ACE3_ATTR:
        s->sram_ace_attr[(addr - A_SYSCON_SRAM_ACE0_ATTR) / 4] = (uint32_t)value;
        break;
    case A_SYSCON_SRAM_ACE0_ADDR ... A_SYSCON_SRAM_ACE3_ADDR:
        s->sram_ace_addr[(addr - A_SYSCON_SRAM_ACE0_ADDR) / 4] = (uint32_t)value;
        break;
    case A_SYSCON_SRAM_ACE0_SIZE ... A_SYSCON_SRAM_ACE3_SIZE:
        s->sram_ace_size[(addr - A_SYSCON_SRAM_ACE0_SIZE) / 4] = (uint32_t)value;
        break;

    case A_SYSCON_SPI_MEM_PMS_CTRL:
        s->spi_mem_pms_ctrl = (uint32_t)value;
        break;
    /* A_SYSCON_SPI_MEM_REJECT_ADDR is read-only, silently ignore writes */
    case A_SYSCON_SPI_MEM_REJECT_ADDR:
        break;
    case A_SYSCON_SDIO_CTRL:
        s->sdio_ctrl = (uint32_t)value;
        break;
    case A_SYSCON_REDCY_SIG0:
        s->redcy_sig0 = (uint32_t)value;
        break;
    case A_SYSCON_REDCY_SIG1:
        s->redcy_sig1 = (uint32_t)value;
        break;
    case A_SYSCON_FRONT_END_MEM_PD:
        s->front_end_mem_pd = (uint32_t)value;
        break;
    case A_SYSCON_SPI_MEM_ECC_CTRL:
        s->spi_mem_ecc_ctrl = (uint32_t)value;
        break;
    case A_SYSCON_CLKGATE_FORCE_ON:
        s->clkgate_force_on = (uint32_t)value;
        break;
    case A_SYSCON_MEM_POWER_DOWN:
        s->mem_power_down = (uint32_t)value;
        break;
    case A_SYSCON_MEM_POWER_UP:
        s->mem_power_up = (uint32_t)value;
        break;
    case A_SYSCON_RETENTION_CTRL:
        s->retention_ctrl = (uint32_t)value;
        break;
    case A_SYSCON_RETENTION_CTRL1:
        s->retention_ctrl1 = (uint32_t)value;
        break;
    case A_SYSCON_RETENTION_CTRL2:
        s->retention_ctrl2 = (uint32_t)value;
        break;
    case A_SYSCON_RETENTION_CTRL3:
        s->retention_ctrl3 = (uint32_t)value;
        break;
    case A_SYSCON_RETENTION_CTRL4:
        s->retention_ctrl4 = (uint32_t)value;
        break;
    case A_SYSCON_RETENTION_CTRL5:
        s->retention_ctrl5 = (uint32_t)value;
        break;

    case RGB_QEMU_ORIGIN_REG:
        s->qemu_origin = (uint32_t)value;
        break;

    case A_SYSCON_DATE:
        s->date = (uint32_t)value;
        break;

    default:
#if SYSCON_WARNING
        warn_report("[SYSCON] Unsupported write to 0x%03lx (0x%08lx)\n",
                    (unsigned long)addr, (unsigned long)value);
#endif
        break;
    }
}


static const MemoryRegionOps esp32s3_syscon_ops = {
    .read  = esp32s3_syscon_read,
    .write = esp32s3_syscon_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};


static void esp32s3_syscon_reset_hold(Object *obj, ResetType type)
{
    ESP32S3SysconState *s = ESP32S3_SYSCON(obj);

    s->clk_conf     = SYSCON_CLK_CONF_DEFAULT;
    s->tick_conf    = SYSCON_TICK_CONF_DEFAULT;
    s->clk_out_en   = SYSCON_CLK_OUT_EN_DEFAULT;
    s->wifi_bb_cfg   = 0;
    s->wifi_bb_cfg_2 = 0;
    s->wifi_clk_en   = SYSCON_WIFI_CLK_EN_DEFAULT;
    s->wifi_rst_en   = SYSCON_WIFI_RST_EN_DEFAULT;
    s->host_inf_sel  = 0;

    s->ext_mem_pms_lock = 0;
    s->ext_mem_writeback_bypass = 0;

    for (int i = 0; i < 4; i++) {
        s->flash_ace_attr[i] = SYSCON_ACE_ATTR_DEFAULT;
        s->sram_ace_attr[i]  = SYSCON_ACE_ATTR_DEFAULT;
        s->flash_ace_size[i] = SYSCON_ACE_SIZE_DEFAULT;
        s->sram_ace_size[i]  = SYSCON_ACE_SIZE_DEFAULT;
    }
    s->flash_ace_addr[0] = 0x00000000;
    s->flash_ace_addr[1] = SYSCON_FLASH_ACE1_ADDR_DEFAULT;
    s->flash_ace_addr[2] = SYSCON_FLASH_ACE2_ADDR_DEFAULT;
    s->flash_ace_addr[3] = SYSCON_FLASH_ACE3_ADDR_DEFAULT;
    s->sram_ace_addr[0]  = 0x00000000;
    s->sram_ace_addr[1]  = SYSCON_SRAM_ACE1_ADDR_DEFAULT;
    s->sram_ace_addr[2]  = SYSCON_SRAM_ACE2_ADDR_DEFAULT;
    s->sram_ace_addr[3]  = SYSCON_SRAM_ACE3_ADDR_DEFAULT;

    s->spi_mem_pms_ctrl   = 0;
    s->spi_mem_reject_addr = 0;
    s->sdio_ctrl          = 0;
    s->redcy_sig0         = 0;
    s->redcy_sig1         = 0;
    s->front_end_mem_pd   = SYSCON_FRONT_END_MEM_PD_DEFAULT;
    s->spi_mem_ecc_ctrl   = 0;

    s->clkgate_force_on  = 0xFFFFFFFF;
    s->mem_power_down    = 0;
    s->mem_power_up      = 0xFFFFFFFF;
    s->retention_ctrl    = 0;
    s->retention_ctrl1   = 0;
    s->retention_ctrl2   = 0;
    s->retention_ctrl3   = 0;
    s->retention_ctrl4   = 0xFFFFFFFF;
    s->retention_ctrl5   = 0;

    /* DATE register: 0x02101150 with MSB set for ECO3 revision detection */
    s->date = SYSCON_DATE_DEFAULT | 0x80000000;

    /* QEMU signature (matches previous RAM-hack behavior) */
    s->qemu_origin = 0;
}


static void esp32s3_syscon_realize(DeviceState *dev, Error **errp)
{
    esp32s3_syscon_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}


static void esp32s3_syscon_init(Object *obj)
{
    ESP32S3SysconState *s = ESP32S3_SYSCON(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_syscon_ops, s,
                          TYPE_ESP32S3_SYSCON, ESP32S3_SYSCON_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}


static void esp32s3_syscon_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_syscon_reset_hold;
    dc->realize = esp32s3_syscon_realize;
}


static const TypeInfo esp32s3_syscon_info = {
    .name = TYPE_ESP32S3_SYSCON,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3SysconState),
    .instance_init = esp32s3_syscon_init,
    .class_init = esp32s3_syscon_class_init,
    .class_size = sizeof(ESP32S3SysconClass),
};

static void esp32s3_syscon_register_types(void)
{
    type_register_static(&esp32s3_syscon_info);
}

type_init(esp32s3_syscon_register_types)
