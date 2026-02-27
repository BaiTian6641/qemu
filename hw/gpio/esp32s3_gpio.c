/*
 * ESP32-S3 GPIO emulation with GPIO matrix routing
 *
 * Implements the full GPIO register model for ESP32-S3 including:
 * - Output data (GPIO_OUT/OUT1) with W1TS/W1TC atomic set/clear
 * - Output enable (GPIO_ENABLE/ENABLE1)
 * - Input registers (GPIO_IN/IN1)
 * - Interrupt status (GPIO_STATUS/STATUS1)
 * - Per-pin configuration (GPIO_PINn_REG for 49 GPIOs)
 * - Input signal matrix (GPIO_FUNCn_IN_SEL_CFG for 256 signals)
 * - Output signal matrix (GPIO_FUNCn_OUT_SEL_CFG for 49 GPIOs)
 *
 * MVP limitations:
 * - No real cross-device signal routing (IO MUX interaction is register-only)
 * - GPIO_IN reflects GPIO_OUT for basic software loopback
 * - Interrupt generation is basic (level-triggered from status bits)
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32s3_gpio.h"
#include "trace.h"

#define GPIO_WARNING 0

/* ---------- Interrupt logic ---------- */

static void esp32s3_gpio_update_irq(ESP32S3GPIOState *s)
{
    Esp32GpioState *base = ESP32_GPIO(s);
    uint32_t pending = s->gpio_status | s->gpio_status1;
    trace_esp32s3_gpio_irq_update(s->gpio_status, s->gpio_status1, pending ? 1 : 0);
    qemu_set_irq(base->irq, pending ? 1 : 0);
}

/* ---------- Input computation ---------- */

/*
 * MVP: GPIO_IN reflects GPIO_OUT for enabled output pins.
 * This allows basic software loopback testing.
 */
static uint32_t esp32s3_gpio_compute_in(ESP32S3GPIOState *s)
{
    return s->gpio_out & s->gpio_enable;
}

static uint32_t esp32s3_gpio_compute_in1(ESP32S3GPIOState *s)
{
    return s->gpio_out1 & s->gpio_enable1 & ESP32S3_GPIO1_MASK;
}

/* ---------- Register read ---------- */

static uint64_t esp32s3_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);
    Esp32GpioState *base = ESP32_GPIO(s);
    uint64_t r = 0;

    /* Per-pin config registers: 0x0074 + n*4 for n=0..48 */
    if (addr >= GPIO_PINn_REG_OFFSET(0) &&
        addr <= GPIO_PINn_REG_OFFSET(ESP32S3_GPIO_COUNT - 1) &&
        ((addr - GPIO_PINn_REG_OFFSET(0)) % 4 == 0)) {
        int n = (addr - GPIO_PINn_REG_OFFSET(0)) / 4;
        r = s->gpio_pin[n];
        goto out;
    }

    /* Input signal matrix: 0x0154 + n*4 for n=0..255 */
    if (addr >= GPIO_FUNC_IN_SEL_CFG_OFFSET(0) &&
        addr <= GPIO_FUNC_IN_SEL_CFG_OFFSET(ESP32S3_GPIO_FUNC_IN_SEL_COUNT - 1) &&
        ((addr - GPIO_FUNC_IN_SEL_CFG_OFFSET(0)) % 4 == 0)) {
        int n = (addr - GPIO_FUNC_IN_SEL_CFG_OFFSET(0)) / 4;
        r = s->func_in_sel_cfg[n];
        goto out;
    }

    /* Output signal matrix: 0x0554 + n*4 for n=0..48 */
    if (addr >= GPIO_FUNC_OUT_SEL_CFG_OFFSET(0) &&
        addr <= GPIO_FUNC_OUT_SEL_CFG_OFFSET(ESP32S3_GPIO_COUNT - 1) &&
        ((addr - GPIO_FUNC_OUT_SEL_CFG_OFFSET(0)) % 4 == 0)) {
        int n = (addr - GPIO_FUNC_OUT_SEL_CFG_OFFSET(0)) / 4;
        r = s->func_out_sel_cfg[n];
        goto out;
    }

    switch (addr) {
    case A_GPIO_OUT:
        r = s->gpio_out;
        break;
    case A_GPIO_OUT_W1TS:
    case A_GPIO_OUT_W1TC:
        r = 0; /* write-only */
        break;
    case A_GPIO_OUT1:
        r = s->gpio_out1 & ESP32S3_GPIO1_MASK;
        break;
    case A_GPIO_OUT1_W1TS:
    case A_GPIO_OUT1_W1TC:
        r = 0;
        break;

    case A_GPIO_SDIO_SELECT:
        r = s->sdio_select;
        break;

    case A_GPIO_ENABLE:
        r = s->gpio_enable;
        break;
    case A_GPIO_ENABLE_W1TS:
    case A_GPIO_ENABLE_W1TC:
        r = 0;
        break;
    case A_GPIO_ENABLE1:
        r = s->gpio_enable1 & ESP32S3_GPIO1_MASK;
        break;
    case A_GPIO_ENABLE1_W1TS:
    case A_GPIO_ENABLE1_W1TC:
        r = 0;
        break;

    case A_GPIO_STRAP:
        r = base->strap_mode;
        break;

    case A_GPIO_IN:
        r = esp32s3_gpio_compute_in(s);
        break;
    case A_GPIO_IN1:
        r = esp32s3_gpio_compute_in1(s);
        break;

    case A_GPIO_STATUS:
        r = s->gpio_status;
        break;
    case A_GPIO_STATUS_W1TS:
    case A_GPIO_STATUS_W1TC:
        r = 0;
        break;
    case A_GPIO_STATUS1:
        r = s->gpio_status1 & ESP32S3_GPIO1_MASK;
        break;
    case A_GPIO_STATUS1_W1TS:
    case A_GPIO_STATUS1_W1TC:
        r = 0;
        break;

    /* Per-CPU interrupt status (read-only mirrors of STATUS masked by PINn.INT_ENA) */
    case A_GPIO_PCPU_INT:
        r = s->gpio_status;  /* simplified: same as STATUS */
        break;
    case A_GPIO_PCPU_NMI_INT:
        r = 0;
        break;
    case A_GPIO_PCPU_INT1:
        r = s->gpio_status1 & ESP32S3_GPIO1_MASK;
        break;
    case A_GPIO_PCPU_NMI_INT1:
        r = 0;
        break;
    case A_GPIO_CPUSDIO_INT:
    case A_GPIO_CPUSDIO_INT1:
        r = 0;
        break;

    case A_GPIO_CLOCK_GATE:
        r = s->clock_gate;
        break;
    case A_GPIO_DATE:
        r = s->date_reg;
        break;

    default:
#if GPIO_WARNING
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read at offset 0x%04" HWADDR_PRIx "\n",
                      __func__, addr);
#endif
        break;
    }

out:
    trace_esp32s3_gpio_read(addr, (uint32_t)r);
    return r;
}

/* ---------- Register write ---------- */

static void esp32s3_gpio_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned int size)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(opaque);

    trace_esp32s3_gpio_write(addr, (uint32_t)value);

    /* Per-pin config registers */
    if (addr >= GPIO_PINn_REG_OFFSET(0) &&
        addr <= GPIO_PINn_REG_OFFSET(ESP32S3_GPIO_COUNT - 1) &&
        ((addr - GPIO_PINn_REG_OFFSET(0)) % 4 == 0)) {
        int n = (addr - GPIO_PINn_REG_OFFSET(0)) / 4;
        s->gpio_pin[n] = (uint32_t)value & 0x0003FFFF;  /* bits [17:0] valid */
        return;
    }

    /* Input signal matrix */
    if (addr >= GPIO_FUNC_IN_SEL_CFG_OFFSET(0) &&
        addr <= GPIO_FUNC_IN_SEL_CFG_OFFSET(ESP32S3_GPIO_FUNC_IN_SEL_COUNT - 1) &&
        ((addr - GPIO_FUNC_IN_SEL_CFG_OFFSET(0)) % 4 == 0)) {
        int n = (addr - GPIO_FUNC_IN_SEL_CFG_OFFSET(0)) / 4;
        s->func_in_sel_cfg[n] = (uint32_t)value & 0xFF;  /* bits [7:0] valid */
        return;
    }

    /* Output signal matrix */
    if (addr >= GPIO_FUNC_OUT_SEL_CFG_OFFSET(0) &&
        addr <= GPIO_FUNC_OUT_SEL_CFG_OFFSET(ESP32S3_GPIO_COUNT - 1) &&
        ((addr - GPIO_FUNC_OUT_SEL_CFG_OFFSET(0)) % 4 == 0)) {
        int n = (addr - GPIO_FUNC_OUT_SEL_CFG_OFFSET(0)) / 4;
        s->func_out_sel_cfg[n] = (uint32_t)value & 0xFFF;  /* bits [11:0] valid */
        return;
    }

    switch (addr) {
    case A_GPIO_OUT:
        s->gpio_out = (uint32_t)value;
        break;
    case A_GPIO_OUT_W1TS:
        s->gpio_out |= (uint32_t)value;
        break;
    case A_GPIO_OUT_W1TC:
        s->gpio_out &= ~(uint32_t)value;
        break;
    case A_GPIO_OUT1:
        s->gpio_out1 = (uint32_t)value & ESP32S3_GPIO1_MASK;
        break;
    case A_GPIO_OUT1_W1TS:
        s->gpio_out1 |= ((uint32_t)value & ESP32S3_GPIO1_MASK);
        break;
    case A_GPIO_OUT1_W1TC:
        s->gpio_out1 &= ~((uint32_t)value & ESP32S3_GPIO1_MASK);
        break;

    case A_GPIO_SDIO_SELECT:
        s->sdio_select = (uint32_t)value;
        break;

    case A_GPIO_ENABLE:
        s->gpio_enable = (uint32_t)value;
        break;
    case A_GPIO_ENABLE_W1TS:
        s->gpio_enable |= (uint32_t)value;
        break;
    case A_GPIO_ENABLE_W1TC:
        s->gpio_enable &= ~(uint32_t)value;
        break;
    case A_GPIO_ENABLE1:
        s->gpio_enable1 = (uint32_t)value & ESP32S3_GPIO1_MASK;
        break;
    case A_GPIO_ENABLE1_W1TS:
        s->gpio_enable1 |= ((uint32_t)value & ESP32S3_GPIO1_MASK);
        break;
    case A_GPIO_ENABLE1_W1TC:
        s->gpio_enable1 &= ~((uint32_t)value & ESP32S3_GPIO1_MASK);
        break;

    case A_GPIO_STATUS:
        s->gpio_status = (uint32_t)value;
        esp32s3_gpio_update_irq(s);
        break;
    case A_GPIO_STATUS_W1TS:
        s->gpio_status |= (uint32_t)value;
        esp32s3_gpio_update_irq(s);
        break;
    case A_GPIO_STATUS_W1TC:
        s->gpio_status &= ~(uint32_t)value;
        esp32s3_gpio_update_irq(s);
        break;
    case A_GPIO_STATUS1:
        s->gpio_status1 = (uint32_t)value & ESP32S3_GPIO1_MASK;
        esp32s3_gpio_update_irq(s);
        break;
    case A_GPIO_STATUS1_W1TS:
        s->gpio_status1 |= ((uint32_t)value & ESP32S3_GPIO1_MASK);
        esp32s3_gpio_update_irq(s);
        break;
    case A_GPIO_STATUS1_W1TC:
        s->gpio_status1 &= ~((uint32_t)value & ESP32S3_GPIO1_MASK);
        esp32s3_gpio_update_irq(s);
        break;

    case A_GPIO_CLOCK_GATE:
        s->clock_gate = (uint32_t)value & 0x1;
        break;
    case A_GPIO_DATE:
        s->date_reg = (uint32_t)value;
        break;

    /* Read-only registers — ignore writes */
    case A_GPIO_STRAP:
    case A_GPIO_IN:
    case A_GPIO_IN1:
    case A_GPIO_PCPU_INT:
    case A_GPIO_PCPU_NMI_INT:
    case A_GPIO_PCPU_INT1:
    case A_GPIO_PCPU_NMI_INT1:
    case A_GPIO_CPUSDIO_INT:
    case A_GPIO_CPUSDIO_INT1:
        break;

    default:
#if GPIO_WARNING
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write at offset 0x%04" HWADDR_PRIx "\n",
                      __func__, addr);
#endif
        break;
    }
}

/* ---------- Reset ---------- */

static void esp32s3_gpio_reset_hold(Object *obj, ResetType type)
{
    ESP32S3GPIOState *s = ESP32S3_GPIO(obj);

    trace_esp32s3_gpio_reset();

    s->gpio_out = 0;
    s->gpio_out1 = 0;
    s->gpio_enable = 0;
    s->gpio_enable1 = 0;
    s->gpio_status = 0;
    s->gpio_status1 = 0;
    s->sdio_select = 0;
    s->clock_gate = 0;
    s->date_reg = ESP32S3_GPIO_DATE_VERSION;

    for (int i = 0; i < ESP32S3_GPIO_COUNT; i++) {
        s->gpio_pin[i] = 0;
    }

    /* Input matrix default: all signals routed to constant low, bypass mode */
    for (int i = 0; i < ESP32S3_GPIO_FUNC_IN_SEL_COUNT; i++) {
        s->func_in_sel_cfg[i] = GPIO_FUNC_IN_LOW;
    }

    /* Output matrix default: no output signal routed (signal 0x100) */
    for (int i = 0; i < ESP32S3_GPIO_COUNT; i++) {
        s->func_out_sel_cfg[i] = GPIO_FUNC_OUT_SEL_NONE;
    }
}

/* ---------- Init / class ---------- */

static void esp32s3_gpio_init(Object *obj)
{
    /* Set the default value for the property */
    object_property_set_int(obj, "strap_mode", ESP32S3_STRAP_MODE_FLASH_BOOT, &error_fatal);
}

static void esp32s3_gpio_class_init(ObjectClass *klass, void *data)
{
    Esp32GpioClass *gc = ESP32_GPIO_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    /* Override parent virtual read/write */
    gc->gpio_read = esp32s3_gpio_read;
    gc->gpio_write = esp32s3_gpio_write;

    /* Override reset */
    rc->phases.hold = esp32s3_gpio_reset_hold;
}

static const TypeInfo esp32s3_gpio_info = {
    .name = TYPE_ESP32S3_GPIO,
    .parent = TYPE_ESP32_GPIO,
    .instance_size = sizeof(ESP32S3GPIOState),
    .instance_init = esp32s3_gpio_init,
    .class_init = esp32s3_gpio_class_init,
    .class_size = sizeof(ESP32S3GPIOClass),
};

static void esp32s3_gpio_register_types(void)
{
    type_register_static(&esp32s3_gpio_info);
}

type_init(esp32s3_gpio_register_types)
