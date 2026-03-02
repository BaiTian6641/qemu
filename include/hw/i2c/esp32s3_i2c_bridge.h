/*
 * ESP32-S3 I2C-to-GUI Bridge Slave  (multi-address, dynamic)
 *
 * A single virtual I2C slave per bus that claims an arbitrary set of
 * 7-bit addresses via a custom `match_and_add` override.  Each
 * transaction is intercepted, write data buffered, and the result
 * emitted as a [PERIPH][I2C] JSON line on stderr for the GUI bridge
 * pipeline (QemuController → PeripheralManager → Python device sim).
 *
 * Addresses are managed at runtime through the QOM string property
 * "registered-addrs" (comma-separated hex, e.g. "3c,40").  The GUI
 * pushes this via QMP `qom-set` after capabilities negotiation.
 *
 * I2C scans (firmware calling i2c_scan_bus) will only find addresses
 * that are in the active set — exactly matching real-hardware
 * behaviour where only powered, connected devices ACK.
 *
 * Copyright (c) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#pragma once

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_ESP32S3_I2C_BRIDGE "esp32s3.i2c-bridge"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32S3I2CBridgeState, ESP32S3_I2C_BRIDGE)

/* Maximum bytes we buffer in a single I2C transaction */
#define I2C_BRIDGE_MAX_XFER  1024

/* Maximum bytes we buffer for read-back responses */
#define I2C_BRIDGE_RSP_MAX   256

struct Esp32S3I2CBridgeState {
    I2CSlave parent_obj;

    /* ---- properties (set before realize) ---- */
    char *controller_name;       /* e.g. "i2c0", "i2c1" */

    /* ---- dynamic address table ---- */
    bool     addr_active[128];   /* bitmap: addr_active[a] == true ⇒ ACK address a */
    char    *registered_addrs_str; /* cached "3c,40" for the QOM getter */

    /* ---- per-transaction state ---- */
    uint8_t  current_target_addr; /* address matched for the in-flight transaction */
    bool     in_recv;            /* true when bus is in read (slave→master) mode */
    uint8_t  xfer_buf[I2C_BRIDGE_MAX_XFER];
    uint32_t xfer_len;

    /* ---- read response buffer ---- */
    uint8_t  rsp_buf[I2C_BRIDGE_RSP_MAX];
    uint32_t rsp_len;
    uint32_t rsp_pos;
};
