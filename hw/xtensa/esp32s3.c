/*
 * ESP32S3 SoC and Machine
 *
 * Copyright (c) 2023-2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/memalign.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/xtensa/xtensa_memory.h"
#include "hw/misc/unimp.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "target/xtensa/cpu.h"

#include "hw/misc/esp32s3_rtc_cntl.h"
#include "hw/xtensa/esp32s3_intc.h"

#include "hw/sd/dwc_sdmmc.h"
#include "hw/misc/ssi_psram.h"
#include "core-esp32s3/core-isa.h"
#include "qemu/datadir.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/cpus.h"
#include "sysemu/runstate.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "exec/exec-all.h"
#include "net/net.h"
#include "net/util.h"
#include "elf.h"

#include "hw/ssi/esp32s3_spi.h"
#include "hw/misc/esp32s3_cache.h"
#include "hw/char/esp32s3_uart.h"
#include "hw/misc/esp32s3_rng.h"

#include "hw/nvram/esp32s3_efuse.h"
#include "hw/xtensa/esp32s3_clk.h"
#include "hw/dma/esp32s3_gdma.h"
#include "hw/misc/esp32s3_sha.h"
#include "hw/misc/esp32s3_aes.h"
#include "hw/misc/esp32s3_rsa.h"
#include "hw/misc/esp32s3_hmac.h"
#include "hw/misc/esp32s3_ds.h"
#include "hw/timer/esp32s3_timg.h"
#include "hw/timer/esp32s3_systimer.h"
#include "hw/gpio/esp32s3_gpio.h"
#include "hw/misc/esp32s3_xts_aes.h"
#include "hw/misc/esp32s3_pms.h"
#include "hw/net/can/esp32s3_twai.h"
#include "hw/gpio/esp32s3_iomux.h"
#include "hw/timer/esp32s3_rmt.h"
#include "hw/net/esp32s3_wifi.h"
#include "hw/misc/esp32s3_syscon.h"
#include "hw/misc/esp32s3_assist_debug.h"
#include "hw/misc/esp32s3_regi2c.h"
#include "hw/i2c/esp32s3_i2c.h"
#include "hw/misc/esp32s3_ledc.h"
#include "hw/misc/esp32s3_pcnt.h"
#include "hw/misc/esp32s3_rtc_io.h"
#include "hw/misc/esp32s3_bt.h"
#include "hw/misc/esp32s3_i2s.h"
#include "hw/misc/esp32s3_mcpwm.h"
#include "hw/misc/esp32s3_lcd_cam.h"
#include "hw/misc/esp32s3_usb_otg.h"
#include "hw/misc/esp32s3_gpspi.h"
#include "hw/misc/esp32s3_apb_saradc.h"
#include "hw/misc/esp32s3_sens.h"
#include "hw/misc/esp32s3_ulp.h"
#include "hw/misc/esp32s3_coex.h"

#include "cpu_esp32s3.h"

#include "hw/misc/esp32c3_jtag.h"
#include "hw/display/esp_rgb.h"

#define TYPE_ESP32S3_SOC "xtensa.esp32s3"
#define ESP32S3_SOC(obj) OBJECT_CHECK(Esp32s3SocState, (obj), TYPE_ESP32S3_SOC)

#define TYPE_ESP32S3_CPU XTENSA_CPU_TYPE_NAME("esp32s3")


enum {
    ESP32S3_MEMREGION_IROM,
    ESP32S3_MEMREGION_DROM,
    ESP32S3_MEMREGION_DRAM,
    ESP32S3_MEMREGION_IRAM,
    ESP32S3_MEMREGION_ICACHE,
    ESP32S3_MEMREGION_DCACHE,
    ESP32S3_MEMREGION_RTCSLOW,
    ESP32S3_MEMREGION_RTCFAST,
    ESP32S3_MEMREGION_FRAMEBUF,
};

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} esp32s3_memmap[] = {
    [ESP32S3_MEMREGION_DROM] = { 0x3ff00000, 0x20000 },
    [ESP32S3_MEMREGION_IROM] = { 0x40000000, 0x60000 },
    [ESP32S3_MEMREGION_DRAM] = { 0x3FC80000, 0x170000 },
    [ESP32S3_MEMREGION_IRAM] = { 0x40370000, 0x80000 },
    [ESP32S3_MEMREGION_DCACHE] = { 0x3c000000, ESP32S3_EXTMEM_REGION_SIZE },
    [ESP32S3_MEMREGION_ICACHE] = { 0x42000000, ESP32S3_EXTMEM_REGION_SIZE },
    [ESP32S3_MEMREGION_RTCSLOW] = { 0x50000000, 0x2000 },
    [ESP32S3_MEMREGION_RTCFAST] = { 0x600fe000, 0x2000 },
    /* Virtual Framebuffer, used for the graphical interface */
    [ESP32S3_MEMREGION_FRAMEBUF] = { 0x20000000, ESP_RGB_MAX_VRAM_SIZE },
};


#define ESP32S3_SOC_RESET_PROCPU    0x1
#define ESP32S3_SOC_RESET_APPCPU    0x2
#define ESP32S3_SOC_RESET_PERIPH    0x4
#define ESP32S3_SOC_RESET_DIG       (ESP32S3_SOC_RESET_PROCPU | ESP32S3_SOC_RESET_APPCPU | ESP32S3_SOC_RESET_PERIPH)
#define ESP32S3_SOC_RESET_RTC       0x8
#define ESP32S3_SOC_RESET_ALL       (ESP32S3_SOC_RESET_RTC | ESP32S3_SOC_RESET_DIG)

#define ESP32S3_IO_WARNING  0

typedef struct Esp32s3SocState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    XtensaCPU cpu[ESP32S3_CPU_COUNT];
    Esp32s3IntMatrixState intmatrix;
    ESP32S3UARTState uart[ESP32S3_UART_COUNT];
    ESP32S3GPIOState gpio;
    Esp32s3RngState rng;
    Esp32S3TWAIState twai;

    Esp32s3RtcCntlState rtc_cntl;

    BusState rtc_bus;
    BusState periph_bus;

    MemoryRegion cpu_specific_mem[ESP32S3_CPU_COUNT];
    ESP32S3SpiState spi1;
    ESP32S3CacheState cache;
    ESP32S3EfuseState efuse;
    ESP32S3ClockState clock;
    ESP32S3GdmaState gdma;
    ESP32S3ShaState sha;
    ESP32S3AesState aes;
    ESP32S3RsaState rsa;
    ESP32S3HmacState hmac;
    ESP32S3DsState ds;
    ESP32S3PmsState pms;

    ESP32S3XtsAesState xts_aes;
    ESP32S3TimgState timg[2];
    ESP32S3SysTimerState systimer;

    ESP32C3UsbJtagState jtag;
    ESPRgbState rgb;

    ESP32S3IOMuxState iomux;
    ESP32S3RmtState rmt;
    ESP32S3WifiState wifi;
    ESP32S3SysconState syscon;
    ESP32S3AssistDebugState assist_debug;
    ESP32S3RegI2CState regi2c;

    ESP32S3I2CState i2c[ESP32S3_I2C_COUNT];
    ESP32S3LEDCState ledc;
    ESP32S3PCNTState pcnt;
    ESP32S3RtcIoState rtc_io;
    ESP32S3BtState bt;

    /* S7 peripherals */
    ESP32S3I2SState i2s[2];
    ESP32S3McpwmState mcpwm[2];
    ESP32S3LcdCamState lcd_cam;
    ESP32S3UsbOtgState usb_otg;
    ESP32S3GpSpiState gpspi[2];

    /* S8 peripherals */
    ESP32S3ApbSaradcState apb_saradc;
    ESP32S3SensState sens;
    ESP32S3UlpState ulp;
    ESP32S3CoexState coex;

    MemoryRegion iomem;
    DWCSDMMCState sdmmc;
    DeviceState *eth;
    SsiPsramState *psram;

    uint32_t requested_reset;
} Esp32s3SocState;


/* "QEMU" as a 32-bit value, can be used by the application to to check whether it is running in
 * QEMU or on real hardware */
#define RGB_QEMU_ORIGIN     0x51454d55
#define RGB_QEMU_ORIGIN_REG 0x3F8

static void remove_cpu_watchpoints(XtensaCPU* xcs)
{
    for (int i = 0; i < MAX_NDBREAK; ++i) {
        if (xcs->env.cpu_watchpoint[i]) {
            cpu_watchpoint_remove_by_ref(CPU(xcs), xcs->env.cpu_watchpoint[i]);
            xcs->env.cpu_watchpoint[i] = NULL;
        }
    }
}

static void esp32s3_dig_reset(void *opaque, int n, int level)
{
    Esp32s3SocState *s = ESP32S3_SOC(opaque);
    if (level) {
        s->requested_reset = ESP32S3_SOC_RESET_DIG;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32s3_cpu_reset(void* opaque, int n, int level)
{
    Esp32s3SocState *s = ESP32S3_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? ESP32S3_SOC_RESET_PROCPU : ESP32S3_SOC_RESET_APPCPU;
        /* Use different cause for APP CPU so that its reset doesn't cause QEMU to exit,
         * when -no-reboot option is given.
         */
        ShutdownCause cause = (n == 0) ? SHUTDOWN_CAUSE_GUEST_RESET : SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        qemu_system_reset_request(cause);
    }
}

static void esp32s3_soc_reset(DeviceState *dev)
{
    Esp32s3SocState *s = ESP32S3_SOC(dev);
    if (s->requested_reset == 0) {
        s->requested_reset = ESP32S3_SOC_RESET_ALL;
    }
    if (s->requested_reset & ESP32S3_SOC_RESET_PERIPH) {
        device_cold_reset(DEVICE(&s->intmatrix));
        for (int i = 0; i < ESP32S3_UART_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->uart[i]));
        }
    }
    if (s->requested_reset & ESP32S3_SOC_RESET_PROCPU) {
        xtensa_select_static_vectors(&s->cpu[0].env, s->rtc_cntl.stat_vector_sel[0]);
        remove_cpu_watchpoints(&s->cpu[0]);
        cpu_reset(CPU(&s->cpu[0]));
    }
    if (s->requested_reset & ESP32S3_SOC_RESET_APPCPU && (ESP32S3_CPU_COUNT > 1)) {
        xtensa_select_static_vectors(&s->cpu[1].env, s->rtc_cntl.stat_vector_sel[1]);
        remove_cpu_watchpoints(&s->cpu[1]);
        cpu_reset(CPU(&s->cpu[1]));
    }
    s->requested_reset = 0;
}

static void esp32s3_cpu_stall(void* opaque, int n, int level)
{
}

static void esp32s3_clk_update(void* opaque, int n, int level)
{
    if (!level) {
        return;
    }
}

static void esp32s3_soc_add_periph_device(MemoryRegion *dest, void* dev, hwaddr dport_base_addr)
{
    MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_add_subregion_overlap(dest, dport_base_addr, mr, 0);
    MemoryRegion *mr_apb = g_new(MemoryRegion, 1);
    char *name = g_strdup_printf("mr-apb-0x%08x", (uint32_t) dport_base_addr);
    memory_region_init_alias(mr_apb, OBJECT(dev), name, mr, 0, memory_region_size(mr));
    g_free(name);
}

#define MB (1024*1024)

static void esp32s3_init_spi_flash(Esp32s3SocState *ms, BlockBackend* blk)
{
    DeviceState *spi_master = DEVICE(&ms->spi1);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");
    const char* flash_model = NULL;
    int64_t image_size = blk_getlength(blk);

    switch (image_size) {
        case 2 * MB:
            flash_model = "w25x16";
            break;
        case 4 * MB:
            flash_model = "gd25q32";
            break;
        case 8 * MB:
            flash_model = "gd25q64";
            break;
        case 16 * MB:
            flash_model = "is25lp128";
            break;
        default:
            error_report("Drive size error: only 2, 4, 8, and 16MB images are supported");
            return;
    }

    /* Create the SPI flash model */
    DeviceState *flash_dev = qdev_new(flash_model);
    qdev_prop_set_drive(flash_dev, "drive", blk);

    /* Realize the SPI flash, its "drive" (blk) property must already be set! */
    qdev_realize(flash_dev, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
}

static void esp32s3_machine_init_psram(Esp32s3SocState *ms, uint32_t size_mbytes)
{
    /* PSRAM attached to SPI1, CS1 */
    DeviceState *spi_master = DEVICE(&ms->spi1);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");
    DeviceState *psram = qdev_new(TYPE_SSI_PSRAM);
    qdev_prop_set_uint32(psram, "size_mbytes", size_mbytes);
    qdev_prop_set_uint8(psram, "cs", 1);
    qdev_realize(psram, spi_bus, &error_fatal);
    ms->psram = SSI_PSRAM(psram);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 1,
                                qdev_get_gpio_in_named(psram, SSI_GPIO_CS, 0));
}

static void esp32s3_machine_init_sd(Esp32s3SocState* ss)
{
    DriveInfo *dinfo = drive_get(IF_SD, 0, 0);
    if (dinfo) {
        DeviceState *card;

        card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(dinfo),
                                &error_fatal);
        /* See the comment on not using sysbus-default in esp32_machine_init_i2c */
        DeviceState *sdmmc = DEVICE(&ss->sdmmc);
        SDBus* sd_bus = SD_BUS(qdev_get_child_bus(sdmmc, "sd-bus"));
        qdev_realize_and_unref(card, BUS(sd_bus), &error_fatal);
    }
}

struct Esp32s3MachineState {
    MachineState parent;

    Esp32s3SocState esp32s3;
    DeviceState *flash_dev;
    uint32_t boot_mode;
    uint8_t custom_mac[6];
    bool has_custom_mac;
    uint32_t chip_revision;
    bool has_chip_revision;
};
#define TYPE_ESP32S3_MACHINE MACHINE_TYPE_NAME("esp32s3")

static void __attribute__((unused)) esp32s3_init_openeth(Esp32s3SocState *ms)
{
    MemoryRegion* mr = NULL;
    SysBusDevice* sbd = NULL;

    MemoryRegion* sys_mem = get_system_memory();

    /* Create a new OpenCores Ethernet component */
    DeviceState* open_eth_dev = qemu_create_nic_device("open_eth", true, NULL);
    if (!open_eth_dev) {
        return;
    }
    ms->eth = open_eth_dev;
    sbd = SYS_BUS_DEVICE(open_eth_dev);
    sysbus_realize(sbd, &error_fatal);

    /* OpenCores Ethernet has two memory regions: one for registers and one for descriptors,
        * we need to provide one I/O range for each of them */
    mr = sysbus_mmio_get_region(sbd, 0);
    memory_region_add_subregion_overlap(sys_mem, DR_REG_EMAC_BASE, mr, 0);
    mr = sysbus_mmio_get_region(sbd, 1);
    memory_region_add_subregion_overlap(sys_mem, DR_REG_EMAC_BASE + 0x400, mr, 0);

    sysbus_connect_irq(sbd, 0,
                        qdev_get_gpio_in(DEVICE(&ms->intmatrix), ETS_ETH_MAC_INTR_SOURCE));
}


static void esp32s3_soc_realize(DeviceState *dev, Error **errp)
{
    Esp32s3SocState *s = ESP32S3_SOC(dev);
    MachineState *ms = MACHINE(qdev_get_machine());
    DeviceState* intmatrix_dev = DEVICE(&s->intmatrix);
    MemoryRegion *sys_mem = get_system_memory();

    const struct MemmapEntry *memmap = esp32s3_memmap;

    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *rtcslow = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast = g_new(MemoryRegion, 1);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        MemoryRegion *drom = g_new(MemoryRegion, 1);
        MemoryRegion *irom = g_new(MemoryRegion, 1);

        char name[20];
        snprintf(name, sizeof(name), "esp32s3.irom.cpu%d", i);
        memory_region_init_rom(irom, NULL, name, memmap[ESP32S3_MEMREGION_IROM].size, &error_fatal);
        memory_region_add_subregion(&s->cpu_specific_mem[i], memmap[ESP32S3_MEMREGION_IROM].base, irom);

        const hwaddr offset_in_orig = 0x40000;
        snprintf(name, sizeof(name), "esp32s3.drom.cpu%d", i);
        memory_region_init_alias(drom, NULL, name, irom, offset_in_orig, memmap[ESP32S3_MEMREGION_DROM].size);
        memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_DROM].base, drom);
    }


    memory_region_init_ram(iram, NULL, "esp32s3.iram",
                           memmap[ESP32S3_MEMREGION_IRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_IRAM].base, iram);

    memory_region_init_ram(rtcslow, NULL, "esp32s3.rtcslow",
                           memmap[ESP32S3_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_RTCSLOW].base, rtcslow);

    memory_region_init_ram(rtcfast, NULL, "esp32s3.rtcfast",
                           memmap[ESP32S3_MEMREGION_RTCFAST].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_RTCFAST].base, rtcfast);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_realize(DEVICE(&s->cpu[i]), NULL, &error_fatal);
    }


    for (int i = 0; i < ESP32S3_CPU_COUNT; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "cpu%d", i);
        object_property_set_link(OBJECT(&s->intmatrix), name, OBJECT(qemu_get_cpu(i)), &error_abort);
    }
    qdev_realize(DEVICE(&s->intmatrix), &s->periph_bus, &error_fatal);


    qdev_realize(DEVICE(&s->rtc_cntl), &s->rtc_bus, &error_fatal);
    esp32s3_soc_add_periph_device(sys_mem, &s->rtc_cntl, DR_REG_RTCCNTL_BASE);

    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_DIG_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32S3_RTC_DIG_RESET_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32S3_RTC_CLK_UPDATE_GPIO, 0));
    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_CPU_RESET_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32S3_RTC_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_CPU_STALL_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32S3_RTC_CPU_STALL_GPIO, i));
    }

    for (int i = 0; i < ESP32S3_UART_COUNT; ++i) {
        const hwaddr uart_base[] = {DR_REG_UART_BASE, DR_REG_UART1_BASE, DR_REG_UART2_BASE};
        qdev_realize(DEVICE(&s->uart[i]), &s->periph_bus, &error_fatal);
        esp32s3_soc_add_periph_device(sys_mem, &s->uart[i], uart_base[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_UART0_INTR_SOURCE + i));
    }

    qdev_realize(DEVICE(&s->sdmmc), &s->periph_bus, &error_fatal);
    esp32s3_soc_add_periph_device(sys_mem, &s->sdmmc, DR_REG_SDMMC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdmmc), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_SDIO_HOST_INTR_SOURCE));

    /* SYSCON (APB_CTRL) register model — replaces the previous RAM hack.
     * All SYSCON registers including WIFI_CLK_EN, WIFI_RST_EN, DATE, ACE
     * regions, and QEMU signature are now handled by the proper device model.
     */
    {
        sysbus_realize(SYS_BUS_DEVICE(&s->syscon), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->syscon), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_APB_CTRL_BASE, mr, 0);
        /* Set the QEMU origin signature so apps can detect QEMU */
        s->syscon.qemu_origin = RGB_QEMU_ORIGIN;
    }

    /* ASSIST_DEBUG realization — stack pointer guard & debug recording */
    {
        sysbus_realize(SYS_BUS_DEVICE(&s->assist_debug), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->assist_debug), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_ASSIST_DEBUG_BASE, mr, 0);
        /* Connect core-0 interrupt to the interrupt matrix */
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->assist_debug), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_ASSIST_DEBUG_INTR_SOURCE));
    }

    /* REGI2C / I2C_ANA_MST analog bus stub — absorbs PHY calibration accesses */
    {
        sysbus_realize(SYS_BUS_DEVICE(&s->regi2c), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->regi2c), 0);
        memory_region_add_subregion_overlap(sys_mem, ESP32S3_REGI2C_BASE, mr, 0);
    }

    qemu_register_reset((QEMUResetHandler*) esp32s3_soc_reset, dev);

    /* TWAI realization */
    {
        /* Initialize and realize the TWAI device */
        sysbus_realize(SYS_BUS_DEVICE(&s->twai), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->twai), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_TWAI_BASE, mr, 0);
        /* Connect TWAI interrupt to the interrupt matrix */
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->twai), 0,
                          qdev_get_gpio_in(intmatrix_dev, ETS_TWAI_INTR_SOURCE));
    }
}


static uint64_t esp32s3_io_read(void *opaque, hwaddr addr, unsigned int size)
{
#if ESP32S3_IO_WARNING
    warn_report("[ESP32-S3] Unsupported read to $%08lx, size = %i\n", ESP32S3_IO_START_ADDR + addr, size);
#endif
    return 0;
}


static void esp32s3_io_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
#if ESP32S3_IO_WARNING
        warn_report("[ESP32-S3] Unsupported write $%08lx = %08lx\n", ESP32S3_IO_START_ADDR + addr, value);
#endif
}


/* Define operations for I/OS */
static const MemoryRegionOps esp32s3_io_ops = {
    .read =  esp32s3_io_read,
    .write = esp32s3_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};



static void esp32s3_soc_init(Object *obj)
{
    Esp32s3SocState *s = ESP32S3_SOC(obj);
    MachineState *ms = MACHINE(qdev_get_machine());
    char name[16];
    MemoryRegion *system_memory = get_system_memory();


    qbus_init(&s->periph_bus, sizeof(s->periph_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-periph-bus");
    qbus_init(&s->rtc_bus, sizeof(s->rtc_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-rtc-bus");

    for (int i = 0; i < ms->smp.cpus; ++i) {
        snprintf(name, sizeof(name), "cpu%d", i);

        object_initialize_child(obj, name, &s->cpu[i], TYPE_ESP32S3_CPU);
        // Allocate memory for TIE registers
        s->cpu[i].env.ext = qemu_memalign(16, sizeof(CPUXtensaEsp32s3State));

        if (i == 0)
        {
            s->cpu[i].env.sregs[PRID] = 0xcdcd;
        }
        if (i == 1)
        {
            s->cpu[i].env.sregs[PRID] = 0xabab;
        }

        snprintf(name, sizeof(name), "cpu%d-mem", i);
        memory_region_init(&s->cpu_specific_mem[i], NULL, name, UINT32_MAX);

        CPUState* cs = CPU(&s->cpu[i]);
        cs->num_ases = 1;
        cpu_address_space_init(cs, 0, "cpu-memory", &s->cpu_specific_mem[i]);

        MemoryRegion *cpu_view_sysmem = g_new(MemoryRegion, 1);
        snprintf(name, sizeof(name), "cpu%d-sysmem", i);
        memory_region_init_alias(cpu_view_sysmem, NULL, name, system_memory, 0, UINT32_MAX);
        memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], 0, cpu_view_sysmem, 0);
        cs->memory = &s->cpu_specific_mem[i];
    }

    for (int i = 0; i < ESP32S3_UART_COUNT; ++i) {
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(obj, name, &s->uart[i], TYPE_ESP32S3_UART);
    }

    object_property_add_alias(obj, "serial0", OBJECT(&s->uart[0]), "chardev");
    object_property_add_alias(obj, "serial1", OBJECT(&s->uart[1]), "chardev");
    // object_property_add_alias(obj, "serial2", OBJECT(&s->uart[2]), "chardev");
    qdev_prop_set_chr(DEVICE(&s->uart[0]), "chardev", serial_hd(0));
    qdev_prop_set_chr(DEVICE(&s->uart[1]), "chardev", serial_hd(1));
    // qdev_prop_set_chr(DEVICE(&s->uart[2]), "chardev", serial_hd(2));

    object_initialize_child(obj, "intmatrix", &s->intmatrix, TYPE_ESP32S3_INTMATRIX);

    object_initialize_child(obj, "rtc_cntl", &s->rtc_cntl, TYPE_ESP32S3_RTC_CNTL);

    qdev_init_gpio_in_named(DEVICE(s), esp32s3_dig_reset,  ESP32S3_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32s3_cpu_reset,  ESP32S3_RTC_CPU_RESET_GPIO, ESP32S3_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32s3_cpu_stall,  ESP32S3_RTC_CPU_STALL_GPIO, ESP32S3_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32s3_clk_update, ESP32S3_RTC_CLK_UPDATE_GPIO, 1);

    object_initialize_child(obj, "twai", &s->twai, TYPE_ESP32S3_TWAI);

    object_initialize_child(obj, "sdmmc", &s->sdmmc, TYPE_DWC_SDMMC);
}

static Property esp32s3_soc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32s3_soc_realize;
    device_class_set_props(dc, esp32s3_soc_properties);
}

static const TypeInfo esp32s3_soc_info = {
    .name = TYPE_ESP32S3_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32s3SocState),
    .instance_init = esp32s3_soc_init,
    .class_init = esp32s3_soc_class_init
};

static void esp32s3_soc_register_types(void)
{
    type_register_static(&esp32s3_soc_info);
}

type_init(esp32s3_soc_register_types)


static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}

OBJECT_DECLARE_SIMPLE_TYPE(Esp32s3MachineState, ESP32S3_MACHINE)

static void esp32s3_set_boot_mode(Object *obj, const char *str, Error **errp)
{
    Esp32s3MachineState *m = ESP32S3_MACHINE(obj);

    if (!strncasecmp(str, "flash", 5) ||
        !strncasecmp(str, "spi", 3) ||
        !strncasecmp(str, "normal", 6)) {
        m->boot_mode = ESP32S3_STRAP_MODE_FLASH_BOOT;
        return;
    }

    if (!strncasecmp(str, "download", 8) ||
        !strncasecmp(str, "uart", 4) ||
        !strncasecmp(str, "rom", 3)) {
        m->boot_mode = ESP32S3_STRAP_MODE_UART_BOOT;
        return;
    }

    error_setg(errp, "%s boot mode not supported", str);
}

static void esp32s3_set_mac(Object *obj, const char *str, Error **errp)
{
    Esp32s3MachineState *m = ESP32S3_MACHINE(obj);

    if (str == NULL || *str == '\0') {
        m->has_custom_mac = false;
        memset(m->custom_mac, 0, sizeof(m->custom_mac));
        return;
    }

    if (net_parse_macaddr(m->custom_mac, str) != 0) {
        error_setg(errp, "invalid MAC address '%s' (expected format XX:XX:XX:XX:XX:XX)", str);
        return;
    }

    m->has_custom_mac = true;
}

static char *esp32s3_get_mac(Object *obj, Error **errp)
{
    Esp32s3MachineState *m = ESP32S3_MACHINE(obj);

    if (!m->has_custom_mac) {
        return g_strdup("");
    }

    return g_strdup_printf("%02x:%02x:%02x:%02x:%02x:%02x",
                           m->custom_mac[0], m->custom_mac[1], m->custom_mac[2],
                           m->custom_mac[3], m->custom_mac[4], m->custom_mac[5]);
}

static void esp32s3_set_chip_revision(Object *obj, Visitor *v, const char *name,
                                      void *opaque, Error **errp)
{
    Esp32s3MachineState *m = ESP32S3_MACHINE(obj);
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }

    if (value > 399) {
        error_setg(errp, "chip-revision must be in range 0..399 (major*100+minor)");
        return;
    }

    m->chip_revision = value;
    m->has_chip_revision = true;
}

static void esp32s3_get_chip_revision(Object *obj, Visitor *v, const char *name,
                                      void *opaque, Error **errp)
{
    Esp32s3MachineState *m = ESP32S3_MACHINE(obj);
    uint32_t value = m->has_chip_revision ? m->chip_revision : 0;

    visit_type_uint32(v, name, &value, errp);
}

// -----------------------------------------------

/* Helper for quickly registering unimplemented MMIO at both DPORT and APB mappings.
 * Kept for future sprint use when mapping stub peripherals. */
static void __attribute__((unused))
esp32s3_soc_add_unimp_device(MemoryRegion *dest, const char* name, hwaddr dport_base_addr, size_t size)
{
    create_unimplemented_device(name, dport_base_addr, size);
    char * name_apb = g_strdup_printf("%s-apb", name);
    create_unimplemented_device(name_apb, dport_base_addr + APB_REG_BASE, size);
    g_free(name_apb);
}

static void esp32s3_machine_init(MachineState *machine)
{
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    BlockBackend* blk = NULL;
    if (dinfo) {
        /* MTD was given! We need to initialize and emulate SPI flash */
        qemu_log("Adding SPI flash device\n");
        blk = blk_by_legacy_dinfo(dinfo);
    } else {
        qemu_log("Not initializing SPI Flash\n");
    }

    MemoryRegion *sys_mem = get_system_memory();
    Esp32s3MachineState *ms = ESP32S3_MACHINE(machine);
    object_initialize_child(OBJECT(ms), "soc", &ms->esp32s3, TYPE_ESP32S3_SOC);
    Esp32s3SocState *ss = ESP32S3_SOC(&ms->esp32s3);

    /* Must be initialized before qdev_realize(DEVICE(ss)) because
     * esp32s3_soc_realize() realizes these sub-devices. */
    object_initialize_child(OBJECT(ss), "syscon", &ss->syscon, TYPE_ESP32S3_SYSCON);
    object_initialize_child(OBJECT(ss), "assist_debug", &ss->assist_debug, TYPE_ESP32S3_ASSIST_DEBUG);
    object_initialize_child(OBJECT(ss), "regi2c", &ss->regi2c, TYPE_ESP32S3_REGI2C);

    MemoryRegion *dram = g_new(MemoryRegion, 1);
    const struct MemmapEntry *memmap = esp32s3_memmap;

    memory_region_init_ram(dram, NULL, "esp32s3.dram",
                           memmap[ESP32S3_MEMREGION_DRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_DRAM].base, dram);


    memory_region_init_io(&ss->iomem, OBJECT(&ss->cpu[0]), &esp32s3_io_ops,
                          NULL, "esp32s3.iomem", 0xd1000);
    memory_region_add_subregion(sys_mem, ESP32S3_IO_START_ADDR, &ss->iomem);

    // qdev_prop_set_chr(DEVICE(ss), "serial0", serial_hd(0));
    // qdev_prop_set_chr(DEVICE(ss), "serial1", serial_hd(1));
    // qdev_prop_set_chr(DEVICE(ss), "serial2", serial_hd(2));

    qdev_realize(DEVICE(ss), NULL, &error_fatal);

    object_initialize_child(OBJECT(ss), "extmem", &ss->cache, TYPE_ESP32S3_CACHE);
    object_initialize_child(OBJECT(ss), "spi1", &ss->spi1, TYPE_ESP32S3_SPI);
    object_initialize_child(OBJECT(ss), "efuse", &ss->efuse, TYPE_ESP32S3_EFUSE);
    object_initialize_child(OBJECT(ss), "jtag", &ss->jtag, TYPE_ESP32C3_JTAG);
    object_initialize_child(OBJECT(ss), "gpio", &ss->gpio, TYPE_ESP32S3_GPIO);
    qdev_prop_set_uint32(DEVICE(&ss->gpio), "strap_mode", ms->boot_mode);
    object_initialize_child(OBJECT(ss), "rng", &ss->rng, TYPE_ESP32S3_RNG);

    object_initialize_child(OBJECT(ss), "clock", &ss->clock, TYPE_ESP32S3_CLOCK);

    object_initialize_child(OBJECT(ss), "gdma", &ss->gdma, TYPE_ESP32S3_GDMA);
    object_initialize_child(OBJECT(ss), "sha", &ss->sha, TYPE_ESP32S3_SHA);
    object_initialize_child(OBJECT(ss), "aes", &ss->aes, TYPE_ESP32S3_AES);
    object_initialize_child(OBJECT(ss), "rsa", &ss->rsa, TYPE_ESP32S3_RSA);
    object_initialize_child(OBJECT(ss), "hmac", &ss->hmac, TYPE_ESP32S3_HMAC);
    object_initialize_child(OBJECT(ss), "ds", &ss->ds, TYPE_ESP32S3_DS);
    object_initialize_child(OBJECT(ss), "pms", &ss->pms, TYPE_ESP32S3_PMS);

    object_initialize_child(OBJECT(ss), "xts_aes", &ss->xts_aes, TYPE_ESP32S3_XTS_AES);
    object_initialize_child(OBJECT(ss), "timg0", &ss->timg[0], TYPE_ESP32S3_TIMG);
    object_initialize_child(OBJECT(ss), "timg1", &ss->timg[1], TYPE_ESP32S3_TIMG);
    object_initialize_child(OBJECT(ss), "systimer", &ss->systimer, TYPE_ESP32S3_SYSTIMER);
    object_initialize_child(OBJECT(ss), "rgb", &ss->rgb, TYPE_ESP_RGB);
    object_initialize_child(OBJECT(ss), "iomux", &ss->iomux, TYPE_ESP32S3_IOMUX);
    object_initialize_child(OBJECT(ss), "rmt", &ss->rmt, TYPE_ESP32S3_RMT);
    object_initialize_child(OBJECT(ss), "wifi", &ss->wifi, TYPE_ESP32S3_WIFI);
    for (int i = 0; i < ESP32S3_I2C_COUNT; i++) {
        char name[8];
        snprintf(name, sizeof(name), "i2c%d", i);
        object_initialize_child(OBJECT(ss), name, &ss->i2c[i], TYPE_ESP32S3_I2C);
    }
    object_initialize_child(OBJECT(ss), "ledc", &ss->ledc, TYPE_ESP32S3_LEDC);
    object_initialize_child(OBJECT(ss), "pcnt", &ss->pcnt, TYPE_ESP32S3_PCNT);
    object_initialize_child(OBJECT(ss), "rtc_io", &ss->rtc_io, TYPE_ESP32S3_RTC_IO);
    object_initialize_child(OBJECT(ss), "bt", &ss->bt, TYPE_ESP32S3_BT);

    /* S7 peripherals */
    for (int i = 0; i < 2; i++) {
        char name[8];
        snprintf(name, sizeof(name), "i2s%d", i);
        object_initialize_child(OBJECT(ss), name, &ss->i2s[i], TYPE_ESP32S3_I2S);
    }
    for (int i = 0; i < 2; i++) {
        char name[8];
        snprintf(name, sizeof(name), "mcpwm%d", i);
        object_initialize_child(OBJECT(ss), name, &ss->mcpwm[i], TYPE_ESP32S3_MCPWM);
    }
    object_initialize_child(OBJECT(ss), "lcd_cam", &ss->lcd_cam, TYPE_ESP32S3_LCD_CAM);
    object_initialize_child(OBJECT(ss), "usb_otg", &ss->usb_otg, TYPE_ESP32S3_USB_OTG);

    /* S8 peripherals */
    object_initialize_child(OBJECT(ss), "apb_saradc", &ss->apb_saradc, TYPE_ESP32S3_APB_SARADC);
    object_initialize_child(OBJECT(ss), "sens", &ss->sens, TYPE_ESP32S3_SENS);
    object_initialize_child(OBJECT(ss), "ulp", &ss->ulp, TYPE_ESP32S3_ULP);
    object_initialize_child(OBJECT(ss), "coex", &ss->coex, TYPE_ESP32S3_COEX);
    for (int i = 0; i < 2; i++) {
        char name[8];
        snprintf(name, sizeof(name), "gpspi%d", i + 2);
        object_initialize_child(OBJECT(ss), name, &ss->gpspi[i], TYPE_ESP32S3_GPSPI);
    }

    DeviceState* intmatrix_dev = DEVICE(&ss->intmatrix);
    {
        /* Store the current Machine CPU in the interrupt matrix */
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->intmatrix), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_INTERRUPT_BASE, mr, 0);
    }

    /* OpenCores Ethernet is no longer initialized by default.
     * Wi-Fi data-plane backend (S4) now provides host networking via the
     * QEMU NIC subsystem and uses ETS_WIFI_MAC_INTR_SOURCE (source 0).
     * To use legacy OpenCores Ethernet instead, uncomment the line below
     * and remove Wi-Fi MAC IRQ connection in the wifi block.
     * esp32s3_init_openeth(ss);
     */

    /* USB Serial JTAG realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->jtag), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->jtag), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_USB_SERIAL_JTAG_BASE, mr, 0);
    }

    /* SPI1 controller (SPI Flash) */
    {
        ss->spi1.xts_aes = &ss->xts_aes;
        sysbus_realize(SYS_BUS_DEVICE(&ss->spi1), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->spi1), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SPI1_BASE, mr, 0);
        if (blk) {
            esp32s3_init_spi_flash(ss, blk);
        }
        if (machine->ram_size > 0) {
            esp32s3_machine_init_psram(ss, (uint32_t) (machine->ram_size / MiB));
        }
    }

    /* (Extmem) Cache realization */
    {
        if (blk) {
            ss->cache.flash_blk = blk;
        }
        if (ss->psram) {
            ss->cache.psram = ss->psram;
        }
        ss->cache.xts_aes = &ss->xts_aes;
        sysbus_realize(SYS_BUS_DEVICE(&ss->cache), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->cache), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_EXTMEM_BASE, mr, 0);

        memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_DCACHE].base, &ss->cache.dcache);
        memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_ICACHE].base, &ss->cache.icache);
    }

    /* eFuses realization */
    {
        ss->efuse.has_custom_mac = ms->has_custom_mac;
        memcpy(ss->efuse.custom_mac, ms->custom_mac, sizeof(ms->custom_mac));
        ss->efuse.has_chip_revision = ms->has_chip_revision;
        ss->efuse.chip_revision = ms->chip_revision;

        sysbus_realize(SYS_BUS_DEVICE(&ss->efuse), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->efuse), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_EFUSE_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->efuse), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_EFUSE_INTR_SOURCE));
    }

    /* System clock realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->clock), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->clock), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SYSTEM_BASE, mr, 0);
        /* Connect the IRQ lines to the interrupt matrix */
        for (int i = 0; i < ESP32S3_SYSTEM_CPU_INTR_COUNT; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&ss->clock), i,
                           qdev_get_gpio_in(intmatrix_dev, ETS_FROM_CPU_INTR0_SOURCE + i));
        }
    }
    /* Timer Groups realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->timg[0]), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->timg[0]), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_TIMERGROUP0_BASE, mr, 0);
        /* Connect the T0 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[0]), ESP32S3_T0_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG0_T0_LEVEL_INTR_SOURCE));
        /* Connect the T1 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[0]), ESP32S3_T1_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG0_T1_LEVEL_INTR_SOURCE));
        /* Connect the Watchdog interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[0]), ESP32S3_WDT_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG0_WDT_LEVEL_INTR_SOURCE));
    }
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->timg[1]), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->timg[1]), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_TIMERGROUP1_BASE, mr, 0);
        /* Connect the T0 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[1]), ESP32S3_T0_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG1_T0_LEVEL_INTR_SOURCE));
        /* Connect the T1 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[1]), ESP32S3_T1_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG1_T1_LEVEL_INTR_SOURCE));
        /* Connect the Watchdog interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[1]), ESP32S3_WDT_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG1_WDT_LEVEL_INTR_SOURCE));
    }

    /* System timer */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->systimer), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->systimer), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SYSTIMER_BASE, mr, 0);
        for (int i = 0; i < ESP_SYSTIMER_IRQ_COUNT; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&ss->systimer), i,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SYSTIMER_TARGET0_EDGE_INTR_SOURCE + i));
        }
    }


    /* GPIO realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->gpio), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->gpio), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_GPIO_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->gpio), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_GPIO_INTR_SOURCE));
    }

    {
        qdev_realize(DEVICE(&ss->rng), &ss->periph_bus, &error_fatal);
        esp32s3_soc_add_periph_device(sys_mem, &ss->rng, ESP32S3_RNG_BASE);

    }


    /* GDMA Realization */
    {
        object_property_set_link(OBJECT(&ss->gdma), "soc_mr", OBJECT(dram), &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&ss->gdma), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->gdma), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_GDMA_BASE, mr, 0);
        /* Connect the IRQs to the Interrupt Matrix */
        for (int i = 0; i < ESP32S3_GDMA_CHANNEL_COUNT; i++) {
            qdev_connect_gpio_out_named(DEVICE(&ss->gdma), ESP_GDMA_IRQ_IN_NAME, i,
                                        qdev_get_gpio_in(intmatrix_dev, ETS_DMA_IN_CH0_INTR_SOURCE + i));
            qdev_connect_gpio_out_named(DEVICE(&ss->gdma), ESP_GDMA_IRQ_OUT_NAME, i,
                                        qdev_get_gpio_in(intmatrix_dev, ETS_DMA_OUT_CH0_INTR_SOURCE + i));
        }
   }

    /* SHA realization */
    {
        ss->sha.parent.gdma = ESP_GDMA(&ss->gdma);
        sysbus_realize(SYS_BUS_DEVICE(&ss->sha), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->sha), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SHA_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->sha), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SHA_INTR_SOURCE));
    }

    /* AES realization */
    {
        ss->aes.parent.gdma = ESP_GDMA(&ss->gdma);
        sysbus_realize(SYS_BUS_DEVICE(&ss->aes), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->aes), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_AES_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->aes), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_AES_INTR_SOURCE));
    }
    /* RSA realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->rsa), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->rsa), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_RSA_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->rsa), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_RSA_INTR_SOURCE));
    }
    /* PMS realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->pms), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->pms), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SENSITIVE_BASE, mr, 0);
    }

    /* HMAC realization */
    {
        ss->hmac.parent.efuse = ESP_EFUSE(&ss->efuse);
        qdev_realize(DEVICE(&ss->hmac), &ss->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->hmac), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_HMAC_BASE, mr, 0);
    }


    /* Digital Signature realization */
    {
        ss->ds.parent.hmac = ESP_HMAC(&ss->hmac);
        ss->ds.parent.aes = ESP_AES(&ss->aes);
        ss->ds.parent.rsa = ESP_RSA(&ss->rsa);
        ss->ds.parent.sha = ESP_SHA(&ss->sha);
        qdev_realize(DEVICE(&ss->ds), &ss->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->ds), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_DIGITAL_SIGNATURE_BASE, mr, 0);
    }
    /* XTS-AES realization */
    {
        ss->xts_aes.efuse = ESP_EFUSE(&ss->efuse);
        ss->xts_aes.clock = &ss->clock;
        qdev_realize(DEVICE(&ss->xts_aes), &ss->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->xts_aes), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_AES_XTS_BASE, mr, 0);
    }

    /* RGB display realization */
    {
        /* Give the internal RAM memory region to the display */
        ss->rgb.intram = dram;
        sysbus_realize(SYS_BUS_DEVICE(&ss->rgb), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->rgb), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_FRAMEBUF_BASE, mr, 0);
        memory_region_add_subregion_overlap(sys_mem, esp32s3_memmap[ESP32S3_MEMREGION_FRAMEBUF].base, &ss->rgb.vram, 0);
    }

    /* IO MUX realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->iomux), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->iomux), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_IO_MUX_BASE, mr, 0);
    }

    /* RMT realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->rmt), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->rmt), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_RMT_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->rmt), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_RMT_INTR_SOURCE));
    }

    /* Wi-Fi skeleton realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->wifi), &error_fatal);
        /* Map each Wi-Fi sub-block to its physical address */
        /* 0=BB, 1=NRX, 2=FE, 3=FE2, 4=SLC, 5=SLCHOST, 6=WDEV */
        memory_region_add_subregion_overlap(sys_mem, DR_REG_BB_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->wifi), 0), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_NRX_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->wifi), 1), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_FE_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->wifi), 2), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_FE2_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->wifi), 3), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SLC_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->wifi), 4), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SLCHOST_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->wifi), 5), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_WDEV_BASE,
            sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->wifi), 6), 0);
        /* Connect all Wi-Fi IRQs to the interrupt matrix.
         * Wi-Fi MAC IRQ now takes ETS_WIFI_MAC_INTR_SOURCE (source 0),
         * replacing the OpenCores Ethernet workaround (S4). */
        qdev_connect_gpio_out_named(DEVICE(&ss->wifi), ESP32S3_WIFI_IRQ_NAME, ESP32S3_WIFI_IRQ_MAC,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_WIFI_MAC_INTR_SOURCE));
        qdev_connect_gpio_out_named(DEVICE(&ss->wifi), ESP32S3_WIFI_IRQ_NAME, ESP32S3_WIFI_IRQ_MAC_NMI,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_WIFI_MAC_NMI_SOURCE));
        qdev_connect_gpio_out_named(DEVICE(&ss->wifi), ESP32S3_WIFI_IRQ_NAME, ESP32S3_WIFI_IRQ_PWR,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_WIFI_PWR_INTR_SOURCE));
        qdev_connect_gpio_out_named(DEVICE(&ss->wifi), ESP32S3_WIFI_IRQ_NAME, ESP32S3_WIFI_IRQ_BB,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_WIFI_BB_INTR_SOURCE));
    }

    /* I2C controllers (×2) */
    {
        static const hwaddr i2c_base[] = { DR_REG_I2C_EXT_BASE, DR_REG_I2C1_EXT_BASE };
        static const int i2c_irq[] = { ETS_I2C_EXT0_INTR_SOURCE, ETS_I2C_EXT1_INTR_SOURCE };
        for (int i = 0; i < ESP32S3_I2C_COUNT; i++) {
            sysbus_realize(SYS_BUS_DEVICE(&ss->i2c[i]), &error_fatal);
            MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->i2c[i]), 0);
            memory_region_add_subregion_overlap(sys_mem, i2c_base[i], mr, 0);
            sysbus_connect_irq(SYS_BUS_DEVICE(&ss->i2c[i]), 0,
                               qdev_get_gpio_in(intmatrix_dev, i2c_irq[i]));
        }
    }

    /* LEDC (LED PWM controller) */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->ledc), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->ledc), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_LEDC_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->ledc), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_LEDC_INTR_SOURCE));
    }

    /* PCNT (Pulse Counter) */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->pcnt), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->pcnt), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_PCNT_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->pcnt), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_PCNT_INTR_SOURCE));
    }

    /* RTC IO MUX stub (no dedicated IRQ) */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->rtc_io), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->rtc_io), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_RTCIO_BASE, mr, 0);
    }

    /* BLE Controller (register model + QEMU HCI transport + virtual peer) */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->bt), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->bt), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_BT_BASE, mr, 0);
        /* Wire 7 BT IRQs to interrupt matrix */
        static const int bt_irq_sources[] = {
            ETS_BT_MAC_INTR_SOURCE,   /* index 0 */
            ETS_BT_BB_INTR_SOURCE,    /* index 1 */
            ETS_BT_BB_NMI_SOURCE,     /* index 2 */
            ETS_RWBT_INTR_SOURCE,     /* index 3 */
            ETS_RWBLE_INTR_SOURCE,    /* index 4 */
            ETS_RWBT_NMI_SOURCE,      /* index 5 */
            ETS_RWBLE_NMI_SOURCE,     /* index 6 */
        };
        for (int i = 0; i < ESP32S3_BT_IRQ_COUNT; i++) {
            qdev_connect_gpio_out_named(DEVICE(&ss->bt),
                ESP32S3_BT_IRQ_NAME, i,
                qdev_get_gpio_in(intmatrix_dev, bt_irq_sources[i]));
        }
    }

    /* ================================================================ */
    /*  S7 Peripheral Batch 2                                           */
    /* ================================================================ */

    /* I2S ×2 */
    {
        static const hwaddr i2s_base[] = { DR_REG_I2S_BASE, DR_REG_I2S1_BASE };
        static const int i2s_irq[] = { ETS_I2S0_INTR_SOURCE, ETS_I2S1_INTR_SOURCE };
        static const GdmaPeripheral i2s_gdma_periph[] = { GDMA_I2S0, GDMA_I2S1 };
        for (int i = 0; i < 2; i++) {
            ss->i2s[i].gdma = ESP_GDMA(&ss->gdma);
            ss->i2s[i].gdma_periph = i2s_gdma_periph[i];
            sysbus_realize(SYS_BUS_DEVICE(&ss->i2s[i]), &error_fatal);
            MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->i2s[i]), 0);
            memory_region_add_subregion_overlap(sys_mem, i2s_base[i], mr, 0);
            sysbus_connect_irq(SYS_BUS_DEVICE(&ss->i2s[i]), 0,
                               qdev_get_gpio_in(intmatrix_dev, i2s_irq[i]));
        }
    }

    /* GP-SPI2 / GP-SPI3 */
    {
        static const hwaddr spi_base[] = { DR_REG_SPI2_BASE, DR_REG_SPI3_BASE };
        static const int spi_irq[] = { ETS_SPI2_INTR_SOURCE, ETS_SPI3_INTR_SOURCE };
        static const GdmaPeripheral spi_gdma_periph[] = { GDMA_SPI2, GDMA_SPI3 };
        for (int i = 0; i < 2; i++) {
            ss->gpspi[i].gdma = ESP_GDMA(&ss->gdma);
            ss->gpspi[i].gdma_periph = spi_gdma_periph[i];
            sysbus_realize(SYS_BUS_DEVICE(&ss->gpspi[i]), &error_fatal);
            MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->gpspi[i]), 0);
            memory_region_add_subregion_overlap(sys_mem, spi_base[i], mr, 0);
            sysbus_connect_irq(SYS_BUS_DEVICE(&ss->gpspi[i]), 0,
                               qdev_get_gpio_in(intmatrix_dev, spi_irq[i]));
        }
    }

    /* MCPWM ×2 */
    {
        static const hwaddr mcpwm_base[] = { DR_REG_PWM0_BASE, DR_REG_PWM1_BASE };
        static const int mcpwm_irq[] = { ETS_PWM0_INTR_SOURCE, ETS_PWM1_INTR_SOURCE };
        for (int i = 0; i < 2; i++) {
            sysbus_realize(SYS_BUS_DEVICE(&ss->mcpwm[i]), &error_fatal);
            MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->mcpwm[i]), 0);
            memory_region_add_subregion_overlap(sys_mem, mcpwm_base[i], mr, 0);
            sysbus_connect_irq(SYS_BUS_DEVICE(&ss->mcpwm[i]), 0,
                               qdev_get_gpio_in(intmatrix_dev, mcpwm_irq[i]));
        }
    }

    /* LCD_CAM */
    {
        ss->lcd_cam.gdma = ESP_GDMA(&ss->gdma);
        sysbus_realize(SYS_BUS_DEVICE(&ss->lcd_cam), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->lcd_cam), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_LCD_CAM_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->lcd_cam), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_LCD_CAM_INTR_SOURCE));
    }

    /* USB OTG (DWC2 register stub) */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->usb_otg), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->usb_otg), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_USB_DWC_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->usb_otg), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_USB_INTR_SOURCE));
    }

    /* APB SAR ADC (S8) */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->apb_saradc), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->apb_saradc), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_APB_SARADC_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->apb_saradc), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_APB_ADC_INTR_SOURCE));
    }

    /* SENS / Touch / Analog (S8) */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->sens), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->sens), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SENS_BASE, mr, 0);
    }

    /* ULP RTC Slow Memory (S8) */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->ulp), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->ulp), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_RTC_SLOWMEM_BASE, mr, 0);
    }

    /* Coexistence arbitration model (S8) — mapped after BT/Wi-Fi blocks */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->coex), &error_fatal);
        /* Virtual QEMU-only peripheral mapped in an unused window */
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->coex), 0);
        memory_region_add_subregion_overlap(sys_mem, COEX_MMIO_BASE, mr, 0);
    }

    esp32s3_machine_init_sd(ss);

    /* Need MMU initialized prior to ELF loading,
     * so that ELF gets loaded into virtual addresses
     */
    cpu_reset(CPU(&ss->cpu[0]));

    const char *load_elf_filename = NULL;
    if (machine->firmware) {
        load_elf_filename = machine->firmware;
    }
    if (machine->kernel_filename) {
        qemu_log("Warning: both -bios and -kernel arguments specified. Only loading the the -kernel file.\n");
        load_elf_filename = machine->kernel_filename;
    }

    if (load_elf_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int size = load_elf(load_elf_filename, NULL,
                               translate_phys_addr, &ss->cpu[0],
                               &elf_entry, &elf_lowaddr,
                               NULL, NULL, 0, EM_XTENSA, 0, 0);
        if (size < 0) {
            error_report("Error: could not load ELF file '%s'", load_elf_filename);
            exit(1);
        }

        if (elf_entry != XCHAL_RESET_VECTOR_PADDR) {
            // Since ROM is empty when loading elf file AND
            // PC value is 0x40000400 after reset
            // need to jump to elf entry point to run a programm
            uint8_t p[4];
            memcpy(p, &elf_entry, 4);
            uint8_t boot[] = {
                0x06, 0x01, 0x00,       /* j    1 */
                0x00,                   /* .literal_position */
                p[0], p[1], p[2], p[3], /* .literal elf_entry */
                                        /* 1: */
                0x01, 0xff, 0xff,       /* l32r a0, elf_entry */
                0xa0, 0x00, 0x00,       /* jx   a0 */
            };
            // Write boot function to reset-vector address (0x40000400) of the CPU 0
            rom_add_blob_fixed_as("boot", boot, sizeof(boot), XCHAL_RESET_VECTOR_PADDR, CPU(&ss->cpu[0])->as);
            ss->cpu[0].env.pc = XCHAL_RESET_VECTOR_PADDR;
        }
    } else {
        char *rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32s3_rev0_rom.bin");
        if (rom_binary == NULL) {
            error_report("Error: -bios argument not set, and ROM code binary not found (1)");
            exit(1);
        }

        int size = load_image_targphys_as(rom_binary, esp32s3_memmap[ESP32S3_MEMREGION_IROM].base, esp32s3_memmap[ESP32S3_MEMREGION_IROM].size, CPU(&ss->cpu[0])->as);
        if (size < 0) {
            error_report("Error: could not load ROM binary '%s'", rom_binary);
            exit(1);
        }
        g_free(rom_binary);

        if (ESP32S3_CPU_COUNT > 1)
        {
            rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32s3_rev0_rom.bin");
            if (rom_binary == NULL) {
                error_report("Error: -bios argument not set, and ROM code binary not found (2)");
                exit(1);
            }

            size = load_image_targphys_as(rom_binary, esp32s3_memmap[ESP32S3_MEMREGION_IROM].base, esp32s3_memmap[ESP32S3_MEMREGION_IROM].size, CPU(&ss->cpu[1])->as);
            if (size < 0) {
                error_report("Error: could not load ROM binary '%s'", rom_binary);
                exit(1);
            }
            g_free(rom_binary);
        }
    }
}


static ram_addr_t esp32s3_fixup_ram_size(ram_addr_t requested_size)
{
    ram_addr_t size;
    if (requested_size == 0) {
        size = 0;
    } else if (requested_size <= 2 * MiB) {
        size = 2 * MiB;
    } else if (requested_size <= 4 * MiB ) {
        size = 4 * MiB;
    } else if (requested_size <= 8 * MiB ) {
        size = 8 * MiB;
    } else if (requested_size <= 16 * MiB ) {
        size = 16 * MiB;
    } else if (requested_size <= 32 * MiB ) {
        size = 32 * MiB;
    } else {
        qemu_log("RAM size larger than 32 MB not supported\n");
        size = 32 * MiB;
    }
    return size;
}

/* Initialize machine type */
static void esp32s3_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    ObjectProperty *prop;
    mc->desc = "Espressif ESP32S3 machine";
    mc->init = esp32s3_machine_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;
    mc->default_ram_size = 0;
    mc->fixup_ram_size = esp32s3_fixup_ram_size;

    prop = object_class_property_add_str(oc, "boot-mode", NULL, esp32s3_set_boot_mode);
    object_class_property_set_description(oc, "boot-mode",
                                          "ESP32-S3 boot mode: flash|download");
    object_property_set_default_str(prop, "flash");

    prop = object_class_property_add_str(oc, "mac", esp32s3_get_mac, esp32s3_set_mac);
    object_class_property_set_description(oc, "mac",
                                          "ESP32-S3 base MAC address (XX:XX:XX:XX:XX:XX)");
    object_property_set_default_str(prop, "");

    object_class_property_add(oc, "chip-revision", "uint32",
                              esp32s3_get_chip_revision,
                              esp32s3_set_chip_revision,
                              NULL, NULL);
    object_class_property_set_description(oc, "chip-revision",
                                          "ESP32-S3 revision encoded as major*100+minor (0..399)");
}

static const TypeInfo esp32s3_info = {
    .name = TYPE_ESP32S3_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Esp32s3MachineState),
    .class_init = esp32s3_machine_class_init,
};

static void esp32s3_machine_type_init(void)
{
    type_register_static(&esp32s3_info);
}

type_init(esp32s3_machine_type_init);

