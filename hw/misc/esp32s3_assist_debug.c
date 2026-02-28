/*
 * ESP32-S3 ASSIST_DEBUG register model
 *
 * Provides stack pointer guard (SP_MIN/SP_MAX), memory access region
 * watchpoints, and debug recording registers for both cores.
 * The initial implementation absorbs all R/W traffic and provides
 * correct reset defaults so esp_cpu_init_ccount(), stack watchpoint
 * setup, and other early-boot ASSIST_DEBUG accesses succeed silently.
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
#include "hw/misc/esp32s3_assist_debug.h"

#define ASSIST_DEBUG_DEBUG    0
#define ASSIST_DEBUG_WARNING  0

/* Number of registers in one core block (0x000–0x08C = 36 regs) */
#define CORE_REG_COUNT  36
#define CORE_REG_SIZE   (CORE_REG_COUNT * 4)  /* 0x90 */

/*
 * Helper: given an address within a per-core block (relative to that core's
 * base), perform the read.
 */
static uint64_t assist_core_read(ESP32S3AssistDebugCoreState *c, hwaddr off)
{
    switch (off) {
    case A_ASSIST_CORE0_INTR_ENA:     return c->intr_ena;
    case A_ASSIST_CORE0_INTR_RAW:     return c->intr_raw;
    case A_ASSIST_CORE0_INTR_RLS:     return c->intr_rls;
    case A_ASSIST_CORE0_INTR_CLR:     return 0;  /* write-only */
    case A_ASSIST_CORE0_DRAM0_0_MIN:  return c->dram0_min[0];
    case A_ASSIST_CORE0_DRAM0_0_MAX:  return c->dram0_max[0];
    case A_ASSIST_CORE0_DRAM0_1_MIN:  return c->dram0_min[1];
    case A_ASSIST_CORE0_DRAM0_1_MAX:  return c->dram0_max[1];
    case A_ASSIST_CORE0_PIF_0_MIN:    return c->pif_min[0];
    case A_ASSIST_CORE0_PIF_0_MAX:    return c->pif_max[0];
    case A_ASSIST_CORE0_PIF_1_MIN:    return c->pif_min[1];
    case A_ASSIST_CORE0_PIF_1_MAX:    return c->pif_max[1];
    case A_ASSIST_CORE0_AREA_SP:      return c->area_sp;
    case A_ASSIST_CORE0_AREA_PC:      return c->area_pc;
    case A_ASSIST_CORE0_SP_UNSTABLE:  return c->sp_unstable;
    case A_ASSIST_CORE0_SP_MIN:       return c->sp_min;
    case A_ASSIST_CORE0_SP_MAX:       return c->sp_max;
    case A_ASSIST_CORE0_SP_PC:        return c->sp_pc;
    case A_ASSIST_CORE0_RCD_EN:       return c->rcd_en;
    case A_ASSIST_CORE0_RCD_REC:      return c->rcd_rec;
    case A_ASSIST_CORE0_RCD_INST:     return c->rcd_inst;
    case A_ASSIST_CORE0_RCD_STATUS:   return c->rcd_status;
    case A_ASSIST_CORE0_RCD_DATA:     return c->rcd_data;
    case A_ASSIST_CORE0_RCD_PC:       return c->rcd_pc;
    case A_ASSIST_CORE0_RCD_LS0STAT:  return c->rcd_ls0stat;
    case A_ASSIST_CORE0_RCD_LS0ADDR:  return c->rcd_ls0addr;
    case A_ASSIST_CORE0_RCD_LS0DATA:  return c->rcd_ls0data;
    case A_ASSIST_CORE0_RCD_SP:       return c->rcd_sp;
    case A_ASSIST_CORE0_IRAM0_EXMON_0: return c->iram0_exmon[0];
    case A_ASSIST_CORE0_IRAM0_EXMON_1: return c->iram0_exmon[1];
    case A_ASSIST_CORE0_DRAM0_EXMON_0: return c->dram0_exmon[0];
    case A_ASSIST_CORE0_DRAM0_EXMON_1: return c->dram0_exmon[1];
    case A_ASSIST_CORE0_DRAM0_EXMON_2: return c->dram0_exmon[2];
    case A_ASSIST_CORE0_DRAM0_EXMON_3: return c->dram0_exmon[3];
    case A_ASSIST_CORE0_DRAM0_EXMON_4: return c->dram0_exmon[4];
    case A_ASSIST_CORE0_DRAM0_EXMON_5: return c->dram0_exmon[5];
    default:
        return 0;
    }
}


static void assist_core_write(ESP32S3AssistDebugCoreState *c, hwaddr off,
                               uint32_t value)
{
    switch (off) {
    case A_ASSIST_CORE0_INTR_ENA:
        c->intr_ena = value & 0xFFF;
        break;
    case A_ASSIST_CORE0_INTR_RLS:
        c->intr_rls = value & 0xFFF;
        break;
    case A_ASSIST_CORE0_INTR_CLR:
        /* Writing 1 to a bit clears corresponding bit in intr_raw */
        c->intr_raw &= ~(value & 0xFFF);
        break;
    case A_ASSIST_CORE0_DRAM0_0_MIN:  c->dram0_min[0] = value; break;
    case A_ASSIST_CORE0_DRAM0_0_MAX:  c->dram0_max[0] = value; break;
    case A_ASSIST_CORE0_DRAM0_1_MIN:  c->dram0_min[1] = value; break;
    case A_ASSIST_CORE0_DRAM0_1_MAX:  c->dram0_max[1] = value; break;
    case A_ASSIST_CORE0_PIF_0_MIN:    c->pif_min[0]   = value; break;
    case A_ASSIST_CORE0_PIF_0_MAX:    c->pif_max[0]   = value; break;
    case A_ASSIST_CORE0_PIF_1_MIN:    c->pif_min[1]   = value; break;
    case A_ASSIST_CORE0_PIF_1_MAX:    c->pif_max[1]   = value; break;
    case A_ASSIST_CORE0_SP_UNSTABLE:  c->sp_unstable = value & 0xFF; break;
    case A_ASSIST_CORE0_SP_MIN:       c->sp_min = value; break;
    case A_ASSIST_CORE0_SP_MAX:       c->sp_max = value; break;
    case A_ASSIST_CORE0_RCD_EN:       c->rcd_en = value;  break;
    case A_ASSIST_CORE0_RCD_REC:      c->rcd_rec = value; break;
    /* Read-only registers: silently ignore writes */
    case A_ASSIST_CORE0_INTR_RAW:
    case A_ASSIST_CORE0_AREA_SP:
    case A_ASSIST_CORE0_AREA_PC:
    case A_ASSIST_CORE0_SP_PC:
    case A_ASSIST_CORE0_RCD_INST:
    case A_ASSIST_CORE0_RCD_STATUS:
    case A_ASSIST_CORE0_RCD_DATA:
    case A_ASSIST_CORE0_RCD_PC:
    case A_ASSIST_CORE0_RCD_LS0STAT:
    case A_ASSIST_CORE0_RCD_LS0ADDR:
    case A_ASSIST_CORE0_RCD_LS0DATA:
    case A_ASSIST_CORE0_RCD_SP:
    case A_ASSIST_CORE0_IRAM0_EXMON_0:
    case A_ASSIST_CORE0_IRAM0_EXMON_1:
    case A_ASSIST_CORE0_DRAM0_EXMON_0:
    case A_ASSIST_CORE0_DRAM0_EXMON_1:
    case A_ASSIST_CORE0_DRAM0_EXMON_2:
    case A_ASSIST_CORE0_DRAM0_EXMON_3:
    case A_ASSIST_CORE0_DRAM0_EXMON_4:
    case A_ASSIST_CORE0_DRAM0_EXMON_5:
        break;
    default:
        break;
    }
}


static uint64_t esp32s3_assist_debug_read(void *opaque, hwaddr addr,
                                          unsigned int size)
{
    ESP32S3AssistDebugState *s = ESP32S3_ASSIST_DEBUG(opaque);
    uint64_t r = 0;

    if (addr < CORE_REG_SIZE) {
        /* Core 0 */
        r = assist_core_read(&s->core[0], addr);
    } else if (addr >= A_ASSIST_CORE1_BASE_OFFSET &&
               addr < A_ASSIST_CORE1_BASE_OFFSET + CORE_REG_SIZE) {
        /* Core 1 — remap to core-0-relative offset */
        r = assist_core_read(&s->core[1], addr - A_ASSIST_CORE1_BASE_OFFSET);
    } else {
        /* Shared registers */
        switch (addr) {
        case A_ASSIST_COREX_EXMON_0:        r = s->corex_exmon[0]; break;
        case A_ASSIST_COREX_EXMON_1:        r = s->corex_exmon[1]; break;
        case A_ASSIST_LOG_SETTING:          r = s->log_setting; break;
        case A_ASSIST_LOG_DATA_0:           r = s->log_data[0]; break;
        case A_ASSIST_LOG_DATA_1:           r = s->log_data[1]; break;
        case A_ASSIST_LOG_DATA_2:           r = s->log_data[2]; break;
        case A_ASSIST_LOG_DATA_3:           r = s->log_data[3]; break;
        case A_ASSIST_LOG_DATA_MASK:        r = s->log_data_mask; break;
        case A_ASSIST_LOG_MIN:              r = s->log_min; break;
        case A_ASSIST_LOG_MAX:              r = s->log_max; break;
        case A_ASSIST_LOG_MEM_START:        r = s->log_mem_start; break;
        case A_ASSIST_LOG_MEM_END:          r = s->log_mem_end; break;
        case A_ASSIST_LOG_MEM_WRITING_ADDR: r = s->log_mem_writing_addr; break;
        case A_ASSIST_LOG_MEM_FULL_FLAG:    r = s->log_mem_full_flag; break;
        case A_ASSIST_REG_DATE:             r = s->reg_date; break;
        default:
#if ASSIST_DEBUG_WARNING
            warn_report("[ASSIST_DEBUG] Unsupported read from 0x%03lx\n",
                        (unsigned long)addr);
#endif
            break;
        }
    }

#if ASSIST_DEBUG_DEBUG
    info_report("[ASSIST_DEBUG] Read  0x%03lx => 0x%08lx",
                (unsigned long)addr, (unsigned long)r);
#endif
    return r;
}


static void esp32s3_assist_debug_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned int size)
{
    ESP32S3AssistDebugState *s = ESP32S3_ASSIST_DEBUG(opaque);

#if ASSIST_DEBUG_DEBUG
    info_report("[ASSIST_DEBUG] Write 0x%03lx <= 0x%08lx",
                (unsigned long)addr, (unsigned long)value);
#endif

    if (addr < CORE_REG_SIZE) {
        assist_core_write(&s->core[0], addr, (uint32_t)value);
    } else if (addr >= A_ASSIST_CORE1_BASE_OFFSET &&
               addr < A_ASSIST_CORE1_BASE_OFFSET + CORE_REG_SIZE) {
        assist_core_write(&s->core[1], addr - A_ASSIST_CORE1_BASE_OFFSET,
                          (uint32_t)value);
    } else {
        switch (addr) {
        case A_ASSIST_COREX_EXMON_0:        s->corex_exmon[0] = (uint32_t)value; break;
        case A_ASSIST_COREX_EXMON_1:        s->corex_exmon[1] = (uint32_t)value; break;
        case A_ASSIST_LOG_SETTING:          s->log_setting = (uint32_t)value; break;
        case A_ASSIST_LOG_DATA_0:           s->log_data[0] = (uint32_t)value; break;
        case A_ASSIST_LOG_DATA_1:           s->log_data[1] = (uint32_t)value; break;
        case A_ASSIST_LOG_DATA_2:           s->log_data[2] = (uint32_t)value; break;
        case A_ASSIST_LOG_DATA_3:           s->log_data[3] = (uint32_t)value; break;
        case A_ASSIST_LOG_DATA_MASK:        s->log_data_mask = (uint32_t)value; break;
        case A_ASSIST_LOG_MIN:              s->log_min = (uint32_t)value; break;
        case A_ASSIST_LOG_MAX:              s->log_max = (uint32_t)value; break;
        case A_ASSIST_LOG_MEM_START:        s->log_mem_start = (uint32_t)value; break;
        case A_ASSIST_LOG_MEM_END:          s->log_mem_end = (uint32_t)value; break;
        case A_ASSIST_LOG_MEM_WRITING_ADDR: s->log_mem_writing_addr = (uint32_t)value; break;
        case A_ASSIST_LOG_MEM_FULL_FLAG:    s->log_mem_full_flag = (uint32_t)value; break;
        case A_ASSIST_REG_DATE:             s->reg_date = (uint32_t)value; break;
        default:
#if ASSIST_DEBUG_WARNING
            warn_report("[ASSIST_DEBUG] Unsupported write to 0x%03lx (0x%08lx)\n",
                        (unsigned long)addr, (unsigned long)value);
#endif
            break;
        }
    }
}


static const MemoryRegionOps esp32s3_assist_debug_ops = {
    .read  = esp32s3_assist_debug_read,
    .write = esp32s3_assist_debug_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};


static void assist_core_reset(ESP32S3AssistDebugCoreState *c)
{
    c->intr_ena = 0;
    c->intr_raw = 0;
    c->intr_rls = 0;

    /* DRAM0 region bounds: min=~0, max=0 means "no active region" */
    c->dram0_min[0] = 0xFFFFFFFF;
    c->dram0_max[0] = 0;
    c->dram0_min[1] = 0xFFFFFFFF;
    c->dram0_max[1] = 0;
    c->pif_min[0]   = 0xFFFFFFFF;
    c->pif_max[0]   = 0;
    c->pif_min[1]   = 0xFFFFFFFF;
    c->pif_max[1]   = 0;

    c->area_sp = 0;
    c->area_pc = 0;
    c->sp_unstable = 0;
    c->sp_min = 0;
    c->sp_max = 0xFFFFFFFF;
    c->sp_pc  = 0;

    c->rcd_en = 0;
    c->rcd_rec = 0;
    c->rcd_inst = 0;
    c->rcd_status = 0;
    c->rcd_data = 0;
    c->rcd_pc = 0;
    c->rcd_ls0stat = 0;
    c->rcd_ls0addr = 0;
    c->rcd_ls0data = 0;
    c->rcd_sp = 0;

    memset(c->iram0_exmon, 0, sizeof(c->iram0_exmon));
    memset(c->dram0_exmon, 0, sizeof(c->dram0_exmon));
}


static void esp32s3_assist_debug_reset_hold(Object *obj, ResetType type)
{
    ESP32S3AssistDebugState *s = ESP32S3_ASSIST_DEBUG(obj);

    assist_core_reset(&s->core[0]);
    assist_core_reset(&s->core[1]);

    memset(s->corex_exmon, 0, sizeof(s->corex_exmon));
    s->log_setting = 0;
    memset(s->log_data, 0, sizeof(s->log_data));
    s->log_data_mask = 0;
    s->log_min = 0;
    s->log_max = 0;
    s->log_mem_start = 0;
    s->log_mem_end = 0;
    s->log_mem_writing_addr = 0;
    s->log_mem_full_flag = 0;
    s->reg_date = 0x02007170;  /* version tag from TRM */

    for (int i = 0; i < ESP32S3_ASSIST_DEBUG_NUM_IRQS; i++) {
        qemu_irq_lower(s->irqs[i]);
    }
}


static void esp32s3_assist_debug_realize(DeviceState *dev, Error **errp)
{
    esp32s3_assist_debug_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}


static void esp32s3_assist_debug_init(Object *obj)
{
    ESP32S3AssistDebugState *s = ESP32S3_ASSIST_DEBUG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_assist_debug_ops, s,
                          TYPE_ESP32S3_ASSIST_DEBUG,
                          ESP32S3_ASSIST_DEBUG_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    for (int i = 0; i < ESP32S3_ASSIST_DEBUG_NUM_IRQS; i++) {
        sysbus_init_irq(sbd, &s->irqs[i]);
    }
}


static void esp32s3_assist_debug_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_assist_debug_reset_hold;
    dc->realize = esp32s3_assist_debug_realize;
}


static const TypeInfo esp32s3_assist_debug_info = {
    .name = TYPE_ESP32S3_ASSIST_DEBUG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3AssistDebugState),
    .instance_init = esp32s3_assist_debug_init,
    .class_init = esp32s3_assist_debug_class_init,
    .class_size = sizeof(ESP32S3AssistDebugClass),
};

static void esp32s3_assist_debug_register_types(void)
{
    type_register_static(&esp32s3_assist_debug_info);
}

type_init(esp32s3_assist_debug_register_types)
