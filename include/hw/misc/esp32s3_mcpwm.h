/*
 * ESP32-S3 MCPWM Controller Model
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * R/W register store for MCPWM0 (0x6001E000) and MCPWM1 (0x6002C000).
 * MVP: register-level model absorbs all driver accesses.
 * 3 timers, 3 operators, capture, fault detection.
 * Interrupt model: 30 bits (INT_RAW/ST/ENA/CLR).
 * IRQ: ETS_PWM0_INTR_SOURCE (31), ETS_PWM1_INTR_SOURCE (32).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_MCPWM  "esp32s3.mcpwm"
#define ESP32S3_MCPWM(obj)  OBJECT_CHECK(ESP32S3McpwmState, (obj), TYPE_ESP32S3_MCPWM)

/* Register space: 0x128 bytes confirmed by _Static_assert */
#define ESP32S3_MCPWM_REG_SIZE  0x128

/* Key register offsets */
#define MCPWM_CLK_CFG_REG          0x000
/* Timer 0..2: each has 4 regs at stride 0x10 starting at 0x004 */
#define MCPWM_TIMER0_CFG0_REG     0x004
#define MCPWM_TIMER0_CFG1_REG     0x008
#define MCPWM_TIMER0_SYNC_REG    0x00C
#define MCPWM_TIMER0_STATUS_REG  0x010
/* Timer 1 at +0x10, Timer 2 at +0x20 */
#define MCPWM_TIMER_SYNCI_CFG_REG 0x034
#define MCPWM_OPERATOR_TIMERSEL_REG 0x038

/* Operators: 3 operators with complex sub-registers, stride ~0x3C each
 * Operator 0 starts at 0x03C, Operator 1 at 0x078, Operator 2 at 0x0B4 */
#define MCPWM_FAULT_DETECT_REG   0x0F0
#define MCPWM_CAP_TIMER_CFG_REG  0x0F4
#define MCPWM_CAP_TIMER_PHASE_REG 0x0F8
#define MCPWM_CAP_CH0_CFG_REG   0x0FC
#define MCPWM_CAP_CH1_CFG_REG   0x100
#define MCPWM_CAP_CH2_CFG_REG   0x104
#define MCPWM_CAP_CH0_REG       0x108
#define MCPWM_CAP_CH1_REG       0x10C
#define MCPWM_CAP_CH2_REG       0x110
#define MCPWM_CAP_STATUS_REG    0x114
#define MCPWM_UPDATE_CFG_REG    0x10C
#define MCPWM_INT_ENA_REG       0x110
#define MCPWM_INT_RAW_REG       0x114
#define MCPWM_INT_ST_REG        0x118
#define MCPWM_INT_CLR_REG       0x11C
#define MCPWM_CLK_REG           0x120
#define MCPWM_VERSION_REG       0x124

#define ESP32S3_MCPWM_REGS_COUNT  (ESP32S3_MCPWM_REG_SIZE / 4)

typedef struct ESP32S3McpwmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[ESP32S3_MCPWM_REGS_COUNT];
    uint32_t int_raw;
    uint32_t int_ena;
} ESP32S3McpwmState;
