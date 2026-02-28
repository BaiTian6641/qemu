/*
 * ESP32-S3 ASSIST_DEBUG register model
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_ASSIST_DEBUG "esp32s3.assist_debug"
#define ESP32S3_ASSIST_DEBUG(obj)           OBJECT_CHECK(ESP32S3AssistDebugState, (obj), TYPE_ESP32S3_ASSIST_DEBUG)
#define ESP32S3_ASSIST_DEBUG_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3AssistDebugClass, obj, TYPE_ESP32S3_ASSIST_DEBUG)
#define ESP32S3_ASSIST_DEBUG_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32S3AssistDebugClass, klass, TYPE_ESP32S3_ASSIST_DEBUG)

#define ESP32S3_ASSIST_DEBUG_REG_SIZE   0x200

/* Number of interrupt line outputs (one per core) */
#define ESP32S3_ASSIST_DEBUG_NUM_IRQS   2

/* ---- Core 0 register offsets ---- */
#define A_ASSIST_CORE0_INTR_ENA         0x000
#define A_ASSIST_CORE0_INTR_RAW         0x004
#define A_ASSIST_CORE0_INTR_RLS         0x008
#define A_ASSIST_CORE0_INTR_CLR         0x00C
#define A_ASSIST_CORE0_DRAM0_0_MIN      0x010
#define A_ASSIST_CORE0_DRAM0_0_MAX      0x014
#define A_ASSIST_CORE0_DRAM0_1_MIN      0x018
#define A_ASSIST_CORE0_DRAM0_1_MAX      0x01C
#define A_ASSIST_CORE0_PIF_0_MIN        0x020
#define A_ASSIST_CORE0_PIF_0_MAX        0x024
#define A_ASSIST_CORE0_PIF_1_MIN        0x028
#define A_ASSIST_CORE0_PIF_1_MAX        0x02C
#define A_ASSIST_CORE0_AREA_SP          0x030
#define A_ASSIST_CORE0_AREA_PC          0x034
#define A_ASSIST_CORE0_SP_UNSTABLE      0x038
#define A_ASSIST_CORE0_SP_MIN           0x03C
#define A_ASSIST_CORE0_SP_MAX           0x040
#define A_ASSIST_CORE0_SP_PC            0x044
#define A_ASSIST_CORE0_RCD_EN           0x048
#define A_ASSIST_CORE0_RCD_REC          0x04C
#define A_ASSIST_CORE0_RCD_INST         0x050
#define A_ASSIST_CORE0_RCD_STATUS       0x054
#define A_ASSIST_CORE0_RCD_DATA         0x058
#define A_ASSIST_CORE0_RCD_PC           0x05C
#define A_ASSIST_CORE0_RCD_LS0STAT      0x060
#define A_ASSIST_CORE0_RCD_LS0ADDR      0x064
#define A_ASSIST_CORE0_RCD_LS0DATA      0x068
#define A_ASSIST_CORE0_RCD_SP           0x06C
#define A_ASSIST_CORE0_IRAM0_EXMON_0    0x070
#define A_ASSIST_CORE0_IRAM0_EXMON_1    0x074
#define A_ASSIST_CORE0_DRAM0_EXMON_0    0x078
#define A_ASSIST_CORE0_DRAM0_EXMON_1    0x07C
#define A_ASSIST_CORE0_DRAM0_EXMON_2    0x080
#define A_ASSIST_CORE0_DRAM0_EXMON_3    0x084
#define A_ASSIST_CORE0_DRAM0_EXMON_4    0x088
#define A_ASSIST_CORE0_DRAM0_EXMON_5    0x08C

/* Core 1 mirrors at +0x90 offset from Core 0 */
#define A_ASSIST_CORE1_BASE_OFFSET      0x090

/* Shared registers */
#define A_ASSIST_COREX_EXMON_0          0x120
#define A_ASSIST_COREX_EXMON_1          0x124
#define A_ASSIST_LOG_SETTING            0x128
#define A_ASSIST_LOG_DATA_0             0x12C
#define A_ASSIST_LOG_DATA_1             0x130
#define A_ASSIST_LOG_DATA_2             0x134
#define A_ASSIST_LOG_DATA_3             0x138
#define A_ASSIST_LOG_DATA_MASK          0x13C
#define A_ASSIST_LOG_MIN                0x140
#define A_ASSIST_LOG_MAX                0x144
#define A_ASSIST_LOG_MEM_START          0x148
#define A_ASSIST_LOG_MEM_END            0x14C
#define A_ASSIST_LOG_MEM_WRITING_ADDR   0x150
#define A_ASSIST_LOG_MEM_FULL_FLAG      0x154
#define A_ASSIST_REG_DATE               0x1FC


/* Per-core state block */
typedef struct ESP32S3AssistDebugCoreState {
    uint32_t intr_ena;
    uint32_t intr_raw;
    uint32_t intr_rls;
    /* intr_clr is write-only */

    uint32_t dram0_min[2];
    uint32_t dram0_max[2];
    uint32_t pif_min[2];
    uint32_t pif_max[2];

    uint32_t area_sp;   /* RO: SP at violation */
    uint32_t area_pc;   /* RO: PC at violation */

    uint32_t sp_unstable;
    uint32_t sp_min;
    uint32_t sp_max;
    uint32_t sp_pc;     /* RO */

    uint32_t rcd_en;
    uint32_t rcd_rec;
    uint32_t rcd_inst;    /* RO */
    uint32_t rcd_status;  /* RO */
    uint32_t rcd_data;    /* RO */
    uint32_t rcd_pc;      /* RO */
    uint32_t rcd_ls0stat; /* RO */
    uint32_t rcd_ls0addr; /* RO */
    uint32_t rcd_ls0data; /* RO */
    uint32_t rcd_sp;      /* RO */

    uint32_t iram0_exmon[2];  /* RO */
    uint32_t dram0_exmon[6];  /* RO */
} ESP32S3AssistDebugCoreState;


typedef struct ESP32S3AssistDebugState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    ESP32S3AssistDebugCoreState core[2];

    /* Shared registers */
    uint32_t corex_exmon[2];
    uint32_t log_setting;
    uint32_t log_data[4];
    uint32_t log_data_mask;
    uint32_t log_min;
    uint32_t log_max;
    uint32_t log_mem_start;
    uint32_t log_mem_end;
    uint32_t log_mem_writing_addr;
    uint32_t log_mem_full_flag;
    uint32_t reg_date;

    /* Output IRQs: one per core */
    qemu_irq irqs[ESP32S3_ASSIST_DEBUG_NUM_IRQS];
} ESP32S3AssistDebugState;

typedef struct ESP32S3AssistDebugClass {
    SysBusDeviceClass parent_class;
} ESP32S3AssistDebugClass;
