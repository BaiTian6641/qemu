/*
 * ESP32-S3 I2C-to-GUI Bridge Slave  (multi-address, dynamic)
 *
 * Virtual I2C slave that intercepts bus transactions and emits
 * [PERIPH][I2C] tagged JSON on stderr for the GUI bridge.
 *
 * One instance per I2C bus.  Addresses are NOT fixed at startup;
 * they are pushed at runtime via the QOM property "registered-addrs"
 * (comma-separated hex, e.g. "3c,40").  I2C scans only find
 * addresses that are in the active set — matching real hardware.
 *
 * Data flow:
 *   Firmware I2C write → QEMU I2C bus → this device (ACK + buffer)
 *                                      → fprintf(stderr, "[PERIPH][I2C] {...}\n")
 *                                      → GUI QProcess::readyReadStandardError
 *                                      → QemuController::ingestBridgeEventLine()
 *                                      → PeripheralManager::dispatchI2cTransfer()
 *                                      → Python device sim (JSON-RPC)
 *
 * Copyright (c) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/i2c/esp32s3_i2c_bridge.h"
#include "hw/qdev-properties.h"

/* #define BRIDGE_DEBUG 1 */

#ifdef BRIDGE_DEBUG
#define BRIDGE_DPRINTF(fmt, ...) \
    fprintf(stderr, "i2c_bridge: " fmt, ## __VA_ARGS__)
#else
#define BRIDGE_DPRINTF(fmt, ...) do {} while (0)
#endif

/* ================================================================== */
/*  Address-table helpers                                              */
/* ================================================================== */

/*
 * Parse a comma-separated hex string like "3c,40,44" and populate
 * the addr_active[] bitmap.  Clears the bitmap first.
 */
static void bridge_parse_addrs(Esp32S3I2CBridgeState *s, const char *str)
{
    memset(s->addr_active, 0, sizeof(s->addr_active));

    if (!str || !*str) {
        return;
    }

    const char *p = str;
    while (*p) {
        /* skip leading whitespace / commas */
        while (*p == ',' || *p == ' ' || *p == '\t') {
            p++;
        }
        if (!*p) {
            break;
        }

        char *end = NULL;
        unsigned long val = strtoul(p, &end, 16);
        if (end == p) {
            break;  /* parse error */
        }
        if (val < 128) {
            s->addr_active[val] = true;
            BRIDGE_DPRINTF("registered addr 0x%02lx on %s\n",
                           val, s->controller_name ? s->controller_name : "?");
        }
        p = end;
    }
}

/*
 * Rebuild the cached string from the bitmap (for the QOM getter).
 */
static void bridge_rebuild_addrs_str(Esp32S3I2CBridgeState *s)
{
    g_free(s->registered_addrs_str);

    GString *gs = g_string_new(NULL);
    bool first = true;
    for (int a = 0; a < 128; a++) {
        if (s->addr_active[a]) {
            if (!first) {
                g_string_append_c(gs, ',');
            }
            g_string_append_printf(gs, "%02x", a);
            first = false;
        }
    }
    s->registered_addrs_str = g_string_free(gs, FALSE);
}

/* ================================================================== */
/*  QOM property "registered-addrs" (runtime-writable)                */
/* ================================================================== */

static char *bridge_get_addrs(Object *obj, Error **errp)
{
    Esp32S3I2CBridgeState *s = ESP32S3_I2C_BRIDGE(obj);
    return g_strdup(s->registered_addrs_str ? s->registered_addrs_str : "");
}

static void bridge_set_addrs(Object *obj, const char *value, Error **errp)
{
    Esp32S3I2CBridgeState *s = ESP32S3_I2C_BRIDGE(obj);

    bridge_parse_addrs(s, value);
    bridge_rebuild_addrs_str(s);

    BRIDGE_DPRINTF("registered-addrs set to \"%s\" on %s\n",
                   s->registered_addrs_str,
                   s->controller_name ? s->controller_name : "?");
}

/* ================================================================== */
/*  Custom match_and_add  (replaces default i2c_slave_match)          */
/* ================================================================== */

/*
 * Called by i2c_scan_bus() and i2c_start_transfer() for every slave
 * on the bus.  We claim the address if it's in our active set.
 */
static bool esp32s3_i2c_bridge_match(I2CSlave *candidate, uint8_t address,
                                     bool broadcast,
                                     I2CNodeList *current_devs)
{
    Esp32S3I2CBridgeState *s = ESP32S3_I2C_BRIDGE(candidate);

    if (broadcast) {
        /* General-call: participate like every other device */
        I2CNode *node = g_new(struct I2CNode, 1);
        node->elt = candidate;
        QLIST_INSERT_HEAD(current_devs, node, next);
        s->current_target_addr = 0x00;
        return true;
    }

    if (address < 128 && s->addr_active[address]) {
        I2CNode *node = g_new(struct I2CNode, 1);
        node->elt = candidate;
        QLIST_INSERT_HEAD(current_devs, node, next);
        s->current_target_addr = address;
        BRIDGE_DPRINTF("match addr=0x%02x on %s\n", address,
                       s->controller_name ? s->controller_name : "?");
        return true;
    }

    /* Address not registered → NACK (don't add to list) */
    return false;
}

/* ================================================================== */
/*  Emit bridge event on stderr                                        */
/* ================================================================== */

static void esp32s3_i2c_bridge_emit(Esp32S3I2CBridgeState *s, bool is_read)
{
    const char *ctrl = s->controller_name ? s->controller_name : "i2c0";
    uint8_t addr = s->current_target_addr;

    /* Build the JSON manually for zero-overhead (no json-c dependency).
     * We keep it on one line so the GUI line-parser works. */
    char *buf = g_malloc(64 + s->xfer_len * 4 + 32);
    int pos = 0;

    pos += sprintf(buf + pos,
                   "[PERIPH][I2C] {\"controller\":\"%s\",\"address\":\"0x%02x\",\"ops\":[{",
                   ctrl, addr);

    if (is_read) {
        pos += sprintf(buf + pos, "\"dir\":\"read\",\"len\":%u", s->xfer_len);
    } else {
        pos += sprintf(buf + pos, "\"dir\":\"write\",\"data\":[");
        for (uint32_t i = 0; i < s->xfer_len; i++) {
            if (i > 0) {
                buf[pos++] = ',';
            }
            pos += sprintf(buf + pos, "%u", s->xfer_buf[i]);
        }
        buf[pos++] = ']';
    }

    pos += sprintf(buf + pos, "}]}\n");

    fwrite(buf, 1, pos, stderr);
    fflush(stderr);

    g_free(buf);
}

/* ================================================================== */
/*  I2C slave callbacks                                                */
/* ================================================================== */

static int esp32s3_i2c_bridge_event(I2CSlave *i2c, enum i2c_event event)
{
    Esp32S3I2CBridgeState *s = ESP32S3_I2C_BRIDGE(i2c);

    switch (event) {
    case I2C_START_SEND:
        BRIDGE_DPRINTF("START_SEND addr=0x%02x\n", s->current_target_addr);
        s->in_recv = false;
        s->xfer_len = 0;
        break;

    case I2C_START_RECV:
        BRIDGE_DPRINTF("START_RECV addr=0x%02x\n", s->current_target_addr);
        s->in_recv = true;
        s->xfer_len = 0;
        break;

    case I2C_FINISH:
        BRIDGE_DPRINTF("FINISH addr=0x%02x len=%u recv=%d\n",
                       s->current_target_addr, s->xfer_len, s->in_recv);
        if (s->xfer_len > 0) {
            esp32s3_i2c_bridge_emit(s, s->in_recv);
        }
        s->xfer_len = 0;
        break;

    case I2C_NACK:
        BRIDGE_DPRINTF("NACK addr=0x%02x\n", s->current_target_addr);
        break;

    default:
        break;
    }

    return 0;  /* ACK — we only get here if match_and_add already accepted */
}

static int esp32s3_i2c_bridge_send(I2CSlave *i2c, uint8_t data)
{
    Esp32S3I2CBridgeState *s = ESP32S3_I2C_BRIDGE(i2c);

    BRIDGE_DPRINTF("TX byte=0x%02x (pos=%u)\n", data, s->xfer_len);

    if (s->xfer_len < I2C_BRIDGE_MAX_XFER) {
        s->xfer_buf[s->xfer_len++] = data;
    }

    return 0;  /* ACK */
}

static uint8_t esp32s3_i2c_bridge_recv(I2CSlave *i2c)
{
    Esp32S3I2CBridgeState *s = ESP32S3_I2C_BRIDGE(i2c);
    uint8_t val = 0xFF;

    if (s->rsp_pos < s->rsp_len) {
        val = s->rsp_buf[s->rsp_pos++];
    }

    s->xfer_len++;  /* Count read bytes for the emit */

    BRIDGE_DPRINTF("RX byte=0x%02x (pos=%u/%u)\n", val, s->rsp_pos, s->rsp_len);
    return val;
}

/* ================================================================== */
/*  Properties                                                         */
/* ================================================================== */

static Property esp32s3_i2c_bridge_properties[] = {
    DEFINE_PROP_STRING("controller", Esp32S3I2CBridgeState, controller_name),
    DEFINE_PROP_END_OF_LIST(),
};

/* ================================================================== */
/*  Lifecycle                                                          */
/* ================================================================== */

static void esp32s3_i2c_bridge_instance_init(Object *obj)
{
    Esp32S3I2CBridgeState *s = ESP32S3_I2C_BRIDGE(obj);

    memset(s->addr_active, 0, sizeof(s->addr_active));
    s->registered_addrs_str = NULL;
    s->current_target_addr = 0;
    s->xfer_len = 0;
    s->rsp_len = 0;
    s->rsp_pos = 0;
    s->in_recv = false;

    /* Register the runtime-writable QOM property */
    object_property_add_str(obj, "registered-addrs",
                            bridge_get_addrs, bridge_set_addrs);
}

static void esp32s3_i2c_bridge_realize(DeviceState *dev, Error **errp)
{
    Esp32S3I2CBridgeState *s = ESP32S3_I2C_BRIDGE(dev);

    if (!s->controller_name) {
        s->controller_name = g_strdup("i2c0");
    }
}

static void esp32s3_i2c_bridge_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = esp32s3_i2c_bridge_realize;
    device_class_set_props(dc, esp32s3_i2c_bridge_properties);

    sc->event       = esp32s3_i2c_bridge_event;
    sc->send        = esp32s3_i2c_bridge_send;
    sc->recv        = esp32s3_i2c_bridge_recv;
    sc->match_and_add = esp32s3_i2c_bridge_match;
}

static const TypeInfo esp32s3_i2c_bridge_info = {
    .name          = TYPE_ESP32S3_I2C_BRIDGE,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Esp32S3I2CBridgeState),
    .instance_init = esp32s3_i2c_bridge_instance_init,
    .class_init    = esp32s3_i2c_bridge_class_init,
};

static void esp32s3_i2c_bridge_register_types(void)
{
    type_register_static(&esp32s3_i2c_bridge_info);
}

type_init(esp32s3_i2c_bridge_register_types)
