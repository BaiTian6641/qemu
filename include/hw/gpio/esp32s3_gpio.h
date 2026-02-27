#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"
#include "esp32_gpio.h"

#define TYPE_ESP32S3_GPIO "esp32s3.gpio"
#define ESP32S3_GPIO(obj)           OBJECT_CHECK(ESP32S3GPIOState, (obj), TYPE_ESP32S3_GPIO)
#define ESP32S3_GPIO_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3GPIOClass, obj, TYPE_ESP32S3_GPIO)
#define ESP32S3_GPIO_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32S3GPIOClass, klass, TYPE_ESP32S3_GPIO)

/* Bootstrap options for ESP32-S3 (4-bit) */
#define ESP32S3_STRAP_MODE_FLASH_BOOT 0x4   /* SPI Boot */

/* ESP32-S3 has 49 GPIO pads (GPIO0 - GPIO48) */
#define ESP32S3_GPIO_COUNT             49

/* Number of input peripheral signals routed through the GPIO matrix */
#define ESP32S3_GPIO_FUNC_IN_SEL_COUNT 256

/* ---------- Register offsets relative to DR_REG_GPIO_BASE ---------- */

/* Output data registers */
REG32(GPIO_OUT, 0x0004)
REG32(GPIO_OUT_W1TS, 0x0008)
REG32(GPIO_OUT_W1TC, 0x000C)
REG32(GPIO_OUT1, 0x0010)          /* bits [16:0] for GPIO32-48 */
REG32(GPIO_OUT1_W1TS, 0x0014)
REG32(GPIO_OUT1_W1TC, 0x0018)

/* SDIO select */
REG32(GPIO_SDIO_SELECT, 0x001C)

/* Output enable registers */
REG32(GPIO_ENABLE, 0x0020)
REG32(GPIO_ENABLE_W1TS, 0x0024)
REG32(GPIO_ENABLE_W1TC, 0x0028)
REG32(GPIO_ENABLE1, 0x002C)       /* bits [16:0] for GPIO32-48 */
REG32(GPIO_ENABLE1_W1TS, 0x0030)
REG32(GPIO_ENABLE1_W1TC, 0x0034)

/* GPIO_STRAP is at 0x0038 — defined in esp32_gpio.h */

/* Input registers */
REG32(GPIO_IN, 0x003C)
REG32(GPIO_IN1, 0x0040)           /* bits [16:0] for GPIO32-48 */

/* Interrupt status registers */
REG32(GPIO_STATUS, 0x0044)
REG32(GPIO_STATUS_W1TS, 0x0048)
REG32(GPIO_STATUS_W1TC, 0x004C)
REG32(GPIO_STATUS1, 0x0050)       /* bits [16:0] for GPIO32-48 */
REG32(GPIO_STATUS1_W1TS, 0x0054)
REG32(GPIO_STATUS1_W1TC, 0x0058)

/* Per-CPU interrupt status (read-only) */
REG32(GPIO_PCPU_INT, 0x005C)
REG32(GPIO_PCPU_NMI_INT, 0x0060)
REG32(GPIO_PCPU_INT1, 0x0064)
REG32(GPIO_PCPU_NMI_INT1, 0x0068)
REG32(GPIO_CPUSDIO_INT, 0x006C)
REG32(GPIO_CPUSDIO_INT1, 0x0070)

/* Per-pin configuration: GPIO_PINn_REG = 0x0074 + n*4, n=0..48 */
#define GPIO_PINn_REG_OFFSET(n) (0x0074 + (n) * 4)

/* GPIO_PINn field layout */
    FIELD(GPIO_PIN0, SYNC2_BYPASS, 0, 2)
    FIELD(GPIO_PIN0, PAD_DRIVER, 2, 1)     /* 0=push-pull, 1=open-drain */
    FIELD(GPIO_PIN0, SYNC1_BYPASS, 3, 2)
    /* bits 5-6: reserved */
    FIELD(GPIO_PIN0, INT_TYPE, 7, 3)       /* 0=disable,1=rise,2=fall,3=any,4=low,5=high */
    FIELD(GPIO_PIN0, WAKEUP_ENABLE, 10, 1)
    /* bits 11-12: reserved */
    FIELD(GPIO_PIN0, INT_ENA, 13, 5)       /* per-CPU interrupt enable mask */

/* Input signal matrix: GPIO_FUNCn_IN_SEL_CFG_REG = 0x0154 + n*4, n=0..255 */
#define GPIO_FUNC_IN_SEL_CFG_OFFSET(n) (0x0154 + (n) * 4)

    FIELD(GPIO_FUNC0_IN_SEL_CFG, FUNC_IN_SEL, 0, 6)    /* GPIO number 0-48 or 0x38=high/0x3C=low */
    FIELD(GPIO_FUNC0_IN_SEL_CFG, FUNC_IN_INV_SEL, 6, 1) /* invert input */
    FIELD(GPIO_FUNC0_IN_SEL_CFG, SIG_IN_SEL, 7, 1)       /* 1=route via GPIO matrix, 0=bypass */

/* Output signal matrix: GPIO_FUNCn_OUT_SEL_CFG_REG = 0x0554 + n*4, n=0..48 */
#define GPIO_FUNC_OUT_SEL_CFG_OFFSET(n) (0x0554 + (n) * 4)

    FIELD(GPIO_FUNC0_OUT_SEL_CFG, FUNC_OUT_SEL, 0, 9)      /* output signal index */
    FIELD(GPIO_FUNC0_OUT_SEL_CFG, FUNC_OUT_INV_SEL, 9, 1)   /* invert output */
    FIELD(GPIO_FUNC0_OUT_SEL_CFG, FUNC_OEN_SEL, 10, 1)      /* 1=use signal's OE, 0=force */
    FIELD(GPIO_FUNC0_OUT_SEL_CFG, FUNC_OEN_INV_SEL, 11, 1)  /* invert OE */

/* Clock gate */
REG32(GPIO_CLOCK_GATE, 0x062C)
    FIELD(GPIO_CLOCK_GATE, CLK_EN, 0, 1)

/* Date/version */
REG32(GPIO_DATE, 0x06FC)

#define ESP32S3_GPIO_DATE_VERSION   0x2101191

/* GPIO1 registers bit mask — only GPIO32-48 (17 bits) */
#define ESP32S3_GPIO1_MASK  0x0001FFFF

/* Special input signal values */
#define GPIO_FUNC_IN_HIGH   0x38   /* Constant high */
#define GPIO_FUNC_IN_LOW    0x3C   /* Constant low */

/* Default FUNC_OUT_SEL when output is not routed (signal 256 = no output) */
#define GPIO_FUNC_OUT_SEL_NONE  0x100


typedef struct ESP32S3GPIOState {
    Esp32GpioState parent;

    /* Output data (GPIO0-31 and GPIO32-48) */
    uint32_t gpio_out;
    uint32_t gpio_out1;

    /* Output enable */
    uint32_t gpio_enable;
    uint32_t gpio_enable1;

    /* Interrupt status */
    uint32_t gpio_status;
    uint32_t gpio_status1;

    /* SDIO select */
    uint32_t sdio_select;

    /* Per-pin configuration (49 pins) */
    uint32_t gpio_pin[ESP32S3_GPIO_COUNT];

    /* Input signal matrix: 256 entries */
    uint32_t func_in_sel_cfg[ESP32S3_GPIO_FUNC_IN_SEL_COUNT];

    /* Output signal matrix: 49 entries */
    uint32_t func_out_sel_cfg[ESP32S3_GPIO_COUNT];

    /* Clock gate */
    uint32_t clock_gate;

    /* Date/version register */
    uint32_t date_reg;

} ESP32S3GPIOState;

typedef struct ESP32S3GPIOClass {
    Esp32GpioClass parent;
} ESP32S3GPIOClass;
