/*
 * ESP32-S3 Clocks definition
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
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/xtensa/esp32s3_clk.h"
#include "hw/xtensa/esp32s3_clk_defs.h"

#define CLOCK_DEBUG      0
#define CLOCK_WARNING    0

static uint32_t esp32s3_read_cpu_intr(ESP32S3ClockState *s, uint32_t index)
{
    return (s->levels >> index) & 1;
}


static void esp32s3_write_cpu_intr(ESP32S3ClockState *s, uint32_t index, uint32_t value)
{
    const uint32_t field = FIELD_EX32(value, SYSTEM_CPU_INTR_FROM_CPU_0, CPU_INTR_FROM_CPU_0);
    if (field) {
        s->levels |= BIT(index);
        qemu_set_irq(s->irqs[index], 1);
    } else {
        s->levels &= ~BIT(index);
        qemu_set_irq(s->irqs[index], 0);
    }
}

static uint32_t esp32s3_clock_get_ext_dev_enc_dec_ctrl(ESP32S3ClockState *s)
{
    return s->sys_ext_dev_enc_dec_ctrl;
}

static uint64_t esp32s3_clock_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3ClockState *s = ESP32S3_CLOCK(opaque);
    uint64_t r = 0;

    switch(addr) {
        case A_SYSTEM_CORE_1_CONTROL_0_REG:
            r = s->core1_ctrl0;
            break;
        case A_SYSTEM_CORE_1_CONTROL_1_REG:
            r = s->app_cpu_addr;
            break;
        case A_SYSTEM_CPU_PER_CONF:
            r = s->cpuperconf;
            break;
        case A_SYSTEM_PERIP_CLK_EN0:
            r = s->perip_clk_en0;
            break;
        case A_SYSTEM_PERIP_CLK_EN1:
            r = s->perip_clk_en1;
            break;
        case A_SYSTEM_PERIP_RST_EN0:
            r = s->perip_rst_en0;
            break;
        case A_SYSTEM_PERIP_RST_EN1:
            r = s->perip_rst_en1;
            break;
        case A_SYSTEM_BT_LPCK_DIV_INT:
            r = s->bt_lpck_div_int;
            break;
        case A_SYSTEM_BT_LPCK_DIV_FRAC:
            r = s->bt_lpck_div_frac;
            break;
        case A_SYSTEM_CPU_INTR_FROM_CPU_0:
        case A_SYSTEM_CPU_INTR_FROM_CPU_1:
        case A_SYSTEM_CPU_INTR_FROM_CPU_2:
        case A_SYSTEM_CPU_INTR_FROM_CPU_3:
            r = esp32s3_read_cpu_intr(s, (addr - A_SYSTEM_CPU_INTR_FROM_CPU_0) / sizeof(uint32_t));
            break;
        case A_SYSTEM_RSA_PD_CTRL:
            r = s->rsa_pd_ctrl;
            break;
        case A_SYSTEM_EDMA_CTRL:
            r = s->edma_ctrl;
            break;
        case A_SYSTEM_CACHE_CONTROL:
            r = s->cache_control;
            break;
        case A_SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL:
            r = s->sys_ext_dev_enc_dec_ctrl;
            break;
        case A_SYSTEM_RTC_FASTMEM_CONFIG:
            r = s->rtc_fastmem_config;
            break;
        case A_SYSTEM_RTC_FASTMEM_CRC:
            r = s->rtc_fastmem_crc;
            break;
        case A_SYSTEM_REDUNDANT_ECO_CTRL:
            r = s->redundant_eco_ctrl;
            break;
        case A_SYSTEM_CLOCK_GATE:
            r = s->clock_gate;
            break;
        case A_SYSTEM_SYSCLK_CONF:
            r = s->sysclk;
            break;
        case A_SYSTEM_SYSTEM_DATE:
            r = s->system_date;
            break;
        /* CPU_PERI_CLK_EN (0x008) and CPU_PERI_RST_EN (0x00C) are not defined
         * in the current defs header — use direct offsets */
        case 0x008:
            r = s->cpu_peri_clk_en;
            break;
        case 0x00C:
            r = s->cpu_peri_rst_en;
            break;
        case 0x014:
            r = s->mem_pd_mask;
            break;
        default:
#if CLOCK_WARNING
            warn_report("[CLOCK] Unsupported read from %08lx\n", addr);
#endif
            break;
    }
    return r;
}

static void esp32s3_clock_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    ESP32S3ClockState *s = ESP32S3_CLOCK(opaque);

    switch(addr) {
        case A_SYSTEM_CORE_1_CONTROL_0_REG:
            s->core1_ctrl0 = (uint32_t)value;
            break;
        case A_SYSTEM_CORE_1_CONTROL_1_REG:
            s->app_cpu_addr = (uint32_t)value;
            break;
        case A_SYSTEM_CPU_PER_CONF:
            s->cpuperconf = (uint32_t)value;
            break;
        case A_SYSTEM_PERIP_CLK_EN0:
            s->perip_clk_en0 = (uint32_t)value;
            break;
        case A_SYSTEM_PERIP_CLK_EN1:
            s->perip_clk_en1 = (uint32_t)value;
            break;
        case A_SYSTEM_PERIP_RST_EN0:
            s->perip_rst_en0 = (uint32_t)value;
            break;
        case A_SYSTEM_PERIP_RST_EN1:
            s->perip_rst_en1 = (uint32_t)value;
            break;
        case A_SYSTEM_BT_LPCK_DIV_INT:
            s->bt_lpck_div_int = (uint32_t)value;
            break;
        case A_SYSTEM_BT_LPCK_DIV_FRAC:
            s->bt_lpck_div_frac = (uint32_t)value;
            break;
        case A_SYSTEM_CPU_INTR_FROM_CPU_0:
        case A_SYSTEM_CPU_INTR_FROM_CPU_1:
        case A_SYSTEM_CPU_INTR_FROM_CPU_2:
        case A_SYSTEM_CPU_INTR_FROM_CPU_3:
            esp32s3_write_cpu_intr(s, (addr - A_SYSTEM_CPU_INTR_FROM_CPU_0) / sizeof(uint32_t), value);
            break;
        case A_SYSTEM_RSA_PD_CTRL:
            s->rsa_pd_ctrl = (uint32_t)value;
            break;
        case A_SYSTEM_EDMA_CTRL:
            s->edma_ctrl = (uint32_t)value;
            break;
        case A_SYSTEM_CACHE_CONTROL:
            s->cache_control = (uint32_t)value;
            break;
        case A_SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL:
            s->sys_ext_dev_enc_dec_ctrl = value;
            break;
        case A_SYSTEM_RTC_FASTMEM_CONFIG:
            s->rtc_fastmem_config = (uint32_t)value;
            break;
        case A_SYSTEM_RTC_FASTMEM_CRC:
            /* effectively read-only, but allow writes silently */
            break;
        case A_SYSTEM_REDUNDANT_ECO_CTRL:
            s->redundant_eco_ctrl = (uint32_t)value;
            break;
        case A_SYSTEM_CLOCK_GATE:
            s->clock_gate = (uint32_t)value;
            break;
        case A_SYSTEM_SYSCLK_CONF:
            s->sysclk = (uint32_t)value;
            break;
        case A_SYSTEM_SYSTEM_DATE:
            s->system_date = (uint32_t)value;
            break;
        /* CPU_PERI_CLK_EN / RST_EN / MEM_PD_MASK — direct offsets */
        case 0x008:
            s->cpu_peri_clk_en = (uint32_t)value;
            break;
        case 0x00C:
            s->cpu_peri_rst_en = (uint32_t)value;
            break;
        case 0x014:
            s->mem_pd_mask = (uint32_t)value;
            break;
        default:
#if CLOCK_WARNING
            warn_report("[CLOCK] Unsupported write to %08lx (%08lx)\n", addr, value);
#endif
            break;
    }
}

static const MemoryRegionOps esp32s3_clock_ops = {
    .read =  esp32s3_clock_read,
    .write = esp32s3_clock_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_clock_reset_hold(Object *obj, ResetType type)
{
    ESP32S3ClockState *s = ESP32S3_CLOCK(obj);

    /* Core 1 control */
    s->core1_ctrl0 = 0;
    s->app_cpu_addr = 0;

    /* CPU peripheral clocking */
    s->cpu_peri_clk_en = 0;
    s->cpu_peri_rst_en = 0;

    /* On board reset, set the proper clocks and dividers */
    s->sysclk = ( 1 << R_SYSTEM_SYSCLK_CONF_PRE_DIV_CNT_SHIFT) |
                (ESP32S3_CLK_SEL_PLL << R_SYSTEM_SYSCLK_CONF_SOC_CLK_SEL_SHIFT) |
                (40 << R_SYSTEM_SYSCLK_CONF_CLK_XTAL_FREQ_SHIFT) |
                ( 1 << R_SYSTEM_SYSCLK_CONF_CLK_DIV_EN_SHIFT);

    /* Divider for PLL clock and APB frequency */
    s->cpuperconf = (ESP32S3_PERIOD_SEL_80 << R_SYSTEM_CPU_PER_CONF_CPUPERIOD_SEL_SHIFT) |
                    (ESP32S3_FREQ_SEL_PLL_480 << R_SYSTEM_CPU_PER_CONF_PLL_FREQ_SEL_SHIFT);

    s->mem_pd_mask = 1;  /* MEM_PD_MASK default */

    /* Peripheral clock enable defaults from ESP-IDF esp_perip_clk_init() expectations */
    s->perip_clk_en0 = 0xF9C1A7C0;
    s->perip_clk_en1 = 0x00000184;
    s->perip_rst_en0 = 0x00000000;
    s->perip_rst_en1 = 0x00000000;

    /* BT low-power clock */
    s->bt_lpck_div_int = 0;
    s->bt_lpck_div_frac = 0;

    /* Initialize the IRQs */
    s->levels = 0;
    for (int i = 0 ; i < ESP32S3_SYSTEM_CPU_INTR_COUNT; i++) {
        qemu_irq_lower(s->irqs[i]);
    }

    /* Crypto / power domain */
    s->rsa_pd_ctrl = 0;
    s->edma_ctrl   = 0;
    s->cache_control = 0x0F;  /* both caches clk_on=1, reset=1 */
    s->sys_ext_dev_enc_dec_ctrl = 0;

    /* RTC fast memory */
    s->rtc_fastmem_config = 0;
    s->rtc_fastmem_crc = 0;

    /* ECO / clock gate */
    s->redundant_eco_ctrl = 0;
    s->clock_gate = 1;  /* CLK_EN default = 1 */

    /* System date register */
    s->system_date = 0x02101150;
}

static void esp32s3_clock_realize(DeviceState *dev, Error **errp)
{
    /* Initialize the registers */
    esp32s3_clock_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}

static void esp32s3_clock_init(Object *obj)
{
    ESP32S3ClockState *s = ESP32S3_CLOCK(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_clock_ops, s,
                          TYPE_ESP32S3_CLOCK, A_SYSTEM_SYSTEM_DATE + sizeof(uint32_t));
    sysbus_init_mmio(sbd, &s->iomem);

    /* Initialize the output IRQ lines used to manually trigger interrupts */
    for (uint64_t i = 0; i < ESP32S3_SYSTEM_CPU_INTR_COUNT; i++) {
        sysbus_init_irq(sbd, &s->irqs[i]);
    }
}

static void esp32s3_clock_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ESP32S3ClockClass* esp32s3_clock = ESP32S3_CLOCK_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_clock_reset_hold;
    dc->realize = esp32s3_clock_realize;

    esp32s3_clock->get_ext_dev_enc_dec_ctrl = esp32s3_clock_get_ext_dev_enc_dec_ctrl;
}

static const TypeInfo esp32s3_cache_info = {
    .name = TYPE_ESP32S3_CLOCK,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3ClockState),
    .instance_init = esp32s3_clock_init,
    .class_init = esp32s3_clock_class_init,
    .class_size = sizeof(ESP32S3ClockClass)
};

static void esp32s3_cache_register_types(void)
{
    type_register_static(&esp32s3_cache_info);
}

type_init(esp32s3_cache_register_types)
