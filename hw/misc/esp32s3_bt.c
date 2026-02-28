/*
 * ESP32-S3 BLE Controller Model
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Sprint S6: BLE Controller MVP
 *  - S6-T1: BT register model (R/W store at 0x60011000, 7 IRQs)
 *  - S6-T2: VHCI transport (QEMU HCI registers at 0xF00-0xFFF)
 *  - S6-T3: Virtual BLE peer (deterministic adv/scan/connect)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32s3_bt.h"

/* ================================================================== */
/*  Forward declarations                                               */
/* ================================================================== */
static void bt_hci_process_command(ESP32S3BtState *s);
static void bt_acl_process(ESP32S3BtState *s);
static void bt_hci_enqueue_rx(ESP32S3BtState *s,
                              const uint8_t *data, uint16_t len);
static void bt_update_irq(ESP32S3BtState *s);
static void bt_scan_timer_cb(void *opaque);

/* ================================================================== */
/*  IRQ helpers                                                        */
/* ================================================================== */
static void bt_update_irq(ESP32S3BtState *s)
{
    uint32_t pending = s->int_raw & s->int_ena;
    /* Route HCI-related interrupts through RWBLE (index 4) */
    qemu_set_irq(s->irq[ESP32S3_BT_IRQ_RWBLE], pending ? 1 : 0);
}

/* ================================================================== */
/*  HCI RX queue management                                            */
/* ================================================================== */
static void bt_hci_enqueue_rx(ESP32S3BtState *s,
                              const uint8_t *data, uint16_t len)
{
    if (s->rx_count >= BT_HCI_RX_QUEUE_DEPTH) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: RX queue overflow, dropping packet\n", __func__);
        return;
    }
    BtHciRxPacket *pkt = g_new0(BtHciRxPacket, 1);
    memcpy(pkt->data, data, MIN(len, BT_HCI_RX_BUF_SIZE));
    pkt->len = MIN(len, BT_HCI_RX_BUF_SIZE);
    QTAILQ_INSERT_TAIL(&s->rx_queue, pkt, entry);
    s->rx_count++;

    /* Signal RX ready interrupt */
    s->int_raw |= BT_HCI_INT_RX_READY;
    bt_update_irq(s);
}

/* ================================================================== */
/*  HCI event builders                                                 */
/* ================================================================== */

/* Build Command Complete event: H4(0x04) + Evt(0x0E) + Len + NumPkts + Opcode + Status + params */
static void bt_send_cmd_complete(ESP32S3BtState *s, uint16_t opcode,
                                 uint8_t status,
                                 const uint8_t *params, uint8_t param_len)
{
    uint8_t buf[BT_HCI_RX_BUF_SIZE];
    int pos = 0;

    buf[pos++] = HCI_H4_EVT;
    buf[pos++] = HCI_EVT_CMD_COMPLETE;
    buf[pos++] = 3 + 1 + param_len;  /* param total length */
    buf[pos++] = s->num_hci_cmd_pkts; /* Num_HCI_Command_Packets */
    buf[pos++] = opcode & 0xFF;
    buf[pos++] = (opcode >> 8) & 0xFF;
    buf[pos++] = status;
    if (param_len > 0 && params) {
        memcpy(&buf[pos], params, param_len);
        pos += param_len;
    }
    bt_hci_enqueue_rx(s, buf, pos);
}

/* Build Command Status event */
static void bt_send_cmd_status(ESP32S3BtState *s, uint16_t opcode,
                               uint8_t status)
{
    uint8_t buf[8];
    int pos = 0;
    buf[pos++] = HCI_H4_EVT;
    buf[pos++] = HCI_EVT_CMD_STATUS;
    buf[pos++] = 4;   /* parameter length */
    buf[pos++] = status;
    buf[pos++] = s->num_hci_cmd_pkts;
    buf[pos++] = opcode & 0xFF;
    buf[pos++] = (opcode >> 8) & 0xFF;
    bt_hci_enqueue_rx(s, buf, pos);
}

/* Build LE Meta Event */
static void bt_send_le_meta(ESP32S3BtState *s, uint8_t sub_event,
                            const uint8_t *params, uint8_t param_len)
{
    uint8_t buf[BT_HCI_RX_BUF_SIZE];
    int pos = 0;
    buf[pos++] = HCI_H4_EVT;
    buf[pos++] = HCI_EVT_LE_META;
    buf[pos++] = 1 + param_len;  /* sub_event + params */
    buf[pos++] = sub_event;
    if (param_len > 0 && params) {
        memcpy(&buf[pos], params, param_len);
        pos += param_len;
    }
    bt_hci_enqueue_rx(s, buf, pos);
}

/* ================================================================== */
/*  Virtual BLE Peer — Advertising report generation                   */
/* ================================================================== */
static void bt_generate_adv_report(ESP32S3BtState *s)
{
    /*
     * LE Advertising Report sub-event:
     *  Num_Reports(1) + Event_Type(1) + Address_Type(1) + Address(6) +
     *  Length_Data(1) + Data(N) + RSSI(1)
     *
     * We generate one report from the virtual peer with a complete local name.
     */
    const char *name = BT_VIRT_PEER_ADV_NAME;
    uint8_t name_len = strlen(name);
    uint8_t params[64];
    int pos = 0;

    params[pos++] = 1;            /* Num_Reports */
    params[pos++] = 0x00;         /* Event_Type: ADV_IND (connectable undirected) */
    params[pos++] = 0x01;         /* Address_Type: Random */
    /* Peer address (little-endian) */
    params[pos++] = 0x02; params[pos++] = 0x00; params[pos++] = 0x00;
    params[pos++] = 0x00; params[pos++] = 0x00; params[pos++] = 0x02;
    params[pos++] = 1 + name_len + 1;  /* Length_Data: type + name */
    params[pos++] = name_len + 1;      /* AD Length */
    params[pos++] = 0x09;              /* AD Type: Complete Local Name */
    memcpy(&params[pos], name, name_len);
    pos += name_len;
    params[pos++] = (uint8_t)(int8_t)BT_VIRT_PEER_RSSI;  /* RSSI */

    bt_send_le_meta(s, HCI_LE_EVT_ADV_REPORT, params, pos);
}

static void bt_scan_timer_cb(void *opaque)
{
    ESP32S3BtState *s = opaque;
    if (!s->scan_enabled || !s->controller_ready) {
        return;
    }
    bt_generate_adv_report(s);

    /* Re-arm timer: ~100 ms interval (virtual clock) */
    timer_mod(s->scan_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100 * SCALE_MS);
}

/* ================================================================== */
/*  Virtual GATT Database (S7-T1)                                      */
/*  Minimal service table for the virtual BLE peer.                    */
/*  Handle | UUID16 | Properties | Value                               */
/* ================================================================== */

/* GATT attribute entry */
typedef struct {
    uint16_t handle;
    uint16_t uuid16;
    uint8_t  properties;  /* for characteristic decl: R/W/N flags */
    const uint8_t *value;
    uint8_t  value_len;
    uint16_t end_group_handle;  /* for service decl only */
} GattAttr;

/* Characteristic property bits */
#define GATT_PROP_READ      0x02
#define GATT_PROP_WRITE     0x08
#define GATT_PROP_NOTIFY    0x10

static const uint8_t gap_device_name[] = "QEMU_BLE";
static const uint8_t gap_appearance[] = {0x00, 0x00};  /* Unknown */

/* The virtual GATT database:
 *  Handles 1-5:   GAP Service (0x1800) — Device Name + Appearance
 *  Handles 6-7:   GATT Service (0x1801)
 *  Handles 8-14:  Custom QEMU Service (0xFF00) — R/W char + Notify char
 */
#define GATT_DB_SIZE 14

static GattAttr gatt_db[GATT_DB_SIZE] = {
    /* GAP Service: handle 1-5 */
    { .handle = 0x0001, .uuid16 = UUID_PRIMARY_SERVICE, .value = (const uint8_t *)"\x00\x18", .value_len = 2, .end_group_handle = 0x0005 },
    { .handle = 0x0002, .uuid16 = UUID_CHARACTERISTIC, .properties = GATT_PROP_READ },
    { .handle = 0x0003, .uuid16 = UUID_DEVICE_NAME, .value = gap_device_name, .value_len = sizeof(gap_device_name) - 1 },
    { .handle = 0x0004, .uuid16 = UUID_CHARACTERISTIC, .properties = GATT_PROP_READ },
    { .handle = 0x0005, .uuid16 = UUID_APPEARANCE, .value = gap_appearance, .value_len = 2 },

    /* GATT Service: handle 6-7 */
    { .handle = 0x0006, .uuid16 = UUID_PRIMARY_SERVICE, .value = (const uint8_t *)"\x01\x18", .value_len = 2, .end_group_handle = 0x0007 },
    { .handle = 0x0007, .uuid16 = UUID_CHARACTERISTIC, .properties = 0 },  /* placeholder */

    /* Custom QEMU Service: handle 8-14 */
    { .handle = 0x0008, .uuid16 = UUID_PRIMARY_SERVICE, .value = (const uint8_t *)"\x00\xFF", .value_len = 2, .end_group_handle = 0x000E },
    { .handle = 0x0009, .uuid16 = UUID_CHARACTERISTIC, .properties = GATT_PROP_READ | GATT_PROP_WRITE },
    { .handle = 0x000A, .uuid16 = UUID_QEMU_RW_CHAR },   /* R/W value (dynamic) */
    { .handle = 0x000B, .uuid16 = UUID_CHARACTERISTIC, .properties = GATT_PROP_READ | GATT_PROP_NOTIFY },
    { .handle = 0x000C, .uuid16 = UUID_QEMU_NOTIFY_CHAR },  /* Notify value */
    { .handle = 0x000D, .uuid16 = UUID_CCC_DESCRIPTOR },     /* CCC descriptor */
    { .handle = 0x000E, .uuid16 = 0x0000 },  /* End marker */
};

static const GattAttr *gatt_find_by_handle(uint16_t handle)
{
    for (int i = 0; i < GATT_DB_SIZE; i++) {
        if (gatt_db[i].handle == handle) {
            return &gatt_db[i];
        }
    }
    return NULL;
}

/* ================================================================== */
/*  ACL / L2CAP / ATT processing (S7-T1)                              */
/* ================================================================== */

/* Send an ACL data packet (controller → host) */
static void bt_send_acl(ESP32S3BtState *s, uint16_t conn_handle,
                        uint16_t l2cap_cid,
                        const uint8_t *payload, uint16_t payload_len)
{
    uint8_t buf[BT_HCI_RX_BUF_SIZE];
    int pos = 0;
    uint16_t l2cap_len = payload_len;
    uint16_t acl_len = 4 + l2cap_len;  /* L2CAP header (4) + payload */

    buf[pos++] = HCI_H4_ACL;
    /* ACL header: handle (12-bit) + PB=0x02 (first-auto) + BC=0x00 */
    uint16_t hdr = (conn_handle & 0x0FFF) | (0x02 << 12);
    buf[pos++] = hdr & 0xFF;
    buf[pos++] = (hdr >> 8) & 0xFF;
    buf[pos++] = acl_len & 0xFF;
    buf[pos++] = (acl_len >> 8) & 0xFF;
    /* L2CAP header */
    buf[pos++] = l2cap_len & 0xFF;
    buf[pos++] = (l2cap_len >> 8) & 0xFF;
    buf[pos++] = l2cap_cid & 0xFF;
    buf[pos++] = (l2cap_cid >> 8) & 0xFF;
    /* Payload */
    if (payload_len > 0 && payload) {
        memcpy(&buf[pos], payload, MIN(payload_len,
               (uint16_t)(BT_HCI_RX_BUF_SIZE - pos)));
        pos += MIN(payload_len, (uint16_t)(BT_HCI_RX_BUF_SIZE - pos));
    }
    bt_hci_enqueue_rx(s, buf, pos);
}

/* Send ATT Error Response */
static void bt_att_error(ESP32S3BtState *s, uint8_t req_opcode,
                         uint16_t handle, uint8_t error_code)
{
    uint8_t rsp[5];
    rsp[0] = ATT_ERROR_RSP;
    rsp[1] = req_opcode;
    rsp[2] = handle & 0xFF;
    rsp[3] = (handle >> 8) & 0xFF;
    rsp[4] = error_code;
    bt_send_acl(s, s->conn_handle, L2CAP_CID_ATT, rsp, 5);
}

/* Handle ATT protocol PDU */
static void bt_att_handle(ESP32S3BtState *s, const uint8_t *att,
                          uint16_t att_len)
{
    if (att_len < 1) return;
    uint8_t opcode = att[0];

    switch (opcode) {

    case ATT_EXCHANGE_MTU_REQ: {
        if (att_len < 3) return;
        uint16_t client_mtu = att[1] | (att[2] << 8);
        s->att_mtu = MIN(client_mtu, 247);  /* Cap at 247 */
        if (s->att_mtu < 23) s->att_mtu = 23;
        uint8_t rsp[3];
        rsp[0] = ATT_EXCHANGE_MTU_RSP;
        rsp[1] = s->att_mtu & 0xFF;
        rsp[2] = (s->att_mtu >> 8) & 0xFF;
        bt_send_acl(s, s->conn_handle, L2CAP_CID_ATT, rsp, 3);
        break;
    }

    case ATT_READ_BY_GROUP_TYPE_REQ: {
        /* Primary service discovery: UUID = 0x2800 */
        if (att_len < 7) return;
        uint16_t start = att[1] | (att[2] << 8);
        uint16_t end   = att[3] | (att[4] << 8);
        uint16_t uuid  = att[5] | (att[6] << 8);

        if (uuid != UUID_PRIMARY_SERVICE) {
            bt_att_error(s, opcode, start, ATT_ERR_ATTR_NOT_FOUND);
            break;
        }

        /* Find services in range */
        uint8_t rsp[64];
        int pos = 0;
        rsp[pos++] = ATT_READ_BY_GROUP_TYPE_RSP;
        rsp[pos++] = 6;  /* length per entry: start(2) + end(2) + uuid16(2) */
        bool found = false;

        for (int i = 0; i < GATT_DB_SIZE; i++) {
            if (gatt_db[i].uuid16 == UUID_PRIMARY_SERVICE &&
                gatt_db[i].handle >= start &&
                gatt_db[i].handle <= end) {
                if (pos + 6 > (int)sizeof(rsp)) break;
                rsp[pos++] = gatt_db[i].handle & 0xFF;
                rsp[pos++] = (gatt_db[i].handle >> 8) & 0xFF;
                rsp[pos++] = gatt_db[i].end_group_handle & 0xFF;
                rsp[pos++] = (gatt_db[i].end_group_handle >> 8) & 0xFF;
                if (gatt_db[i].value && gatt_db[i].value_len >= 2) {
                    rsp[pos++] = gatt_db[i].value[0];
                    rsp[pos++] = gatt_db[i].value[1];
                } else {
                    rsp[pos++] = 0;
                    rsp[pos++] = 0;
                }
                found = true;
            }
        }
        if (!found) {
            bt_att_error(s, opcode, start, ATT_ERR_ATTR_NOT_FOUND);
        } else {
            bt_send_acl(s, s->conn_handle, L2CAP_CID_ATT, rsp, pos);
        }
        break;
    }

    case ATT_READ_BY_TYPE_REQ: {
        /* Characteristic discovery: UUID = 0x2803 */
        if (att_len < 7) return;
        uint16_t start = att[1] | (att[2] << 8);
        uint16_t end   = att[3] | (att[4] << 8);
        uint16_t uuid  = att[5] | (att[6] << 8);

        uint8_t rsp[64];
        int pos = 0;
        rsp[pos++] = ATT_READ_BY_TYPE_RSP;
        bool found = false;

        if (uuid == UUID_CHARACTERISTIC) {
            rsp[pos++] = 7;  /* handle(2) + props(1) + val_handle(2) + uuid16(2) */
            for (int i = 0; i < GATT_DB_SIZE; i++) {
                if (gatt_db[i].uuid16 == UUID_CHARACTERISTIC &&
                    gatt_db[i].handle >= start &&
                    gatt_db[i].handle <= end &&
                    gatt_db[i].properties != 0) {
                    if (pos + 7 > (int)sizeof(rsp)) break;
                    /* Handle of this declaration */
                    rsp[pos++] = gatt_db[i].handle & 0xFF;
                    rsp[pos++] = (gatt_db[i].handle >> 8) & 0xFF;
                    /* Properties */
                    rsp[pos++] = gatt_db[i].properties;
                    /* Value handle (next handle) */
                    uint16_t vh = gatt_db[i].handle + 1;
                    rsp[pos++] = vh & 0xFF;
                    rsp[pos++] = (vh >> 8) & 0xFF;
                    /* UUID of the value (from next entry) */
                    if (i + 1 < GATT_DB_SIZE) {
                        rsp[pos++] = gatt_db[i + 1].uuid16 & 0xFF;
                        rsp[pos++] = (gatt_db[i + 1].uuid16 >> 8) & 0xFF;
                    } else {
                        rsp[pos++] = 0;
                        rsp[pos++] = 0;
                    }
                    found = true;
                }
            }
        } else {
            /* Read by specific UUID — find value attributes with matching UUID */
            rsp[pos++] = 0;  /* placeholder for length, will be filled */
            for (int i = 0; i < GATT_DB_SIZE; i++) {
                if (gatt_db[i].uuid16 == uuid &&
                    gatt_db[i].handle >= start &&
                    gatt_db[i].handle <= end) {
                    const GattAttr *a = &gatt_db[i];
                    uint8_t vlen = a->value ? a->value_len : 0;
                    if (a->uuid16 == UUID_QEMU_RW_CHAR) {
                        vlen = s->rw_char_len;
                    }
                    if (pos + 2 + vlen > (int)sizeof(rsp)) break;
                    rsp[1] = 2 + vlen;  /* length per entry */
                    rsp[pos++] = a->handle & 0xFF;
                    rsp[pos++] = (a->handle >> 8) & 0xFF;
                    if (a->uuid16 == UUID_QEMU_RW_CHAR) {
                        memcpy(&rsp[pos], s->rw_char_value, vlen);
                    } else if (a->value && vlen > 0) {
                        memcpy(&rsp[pos], a->value, vlen);
                    }
                    pos += vlen;
                    found = true;
                }
            }
        }
        if (!found) {
            bt_att_error(s, opcode, start, ATT_ERR_ATTR_NOT_FOUND);
        } else {
            bt_send_acl(s, s->conn_handle, L2CAP_CID_ATT, rsp, pos);
        }
        break;
    }

    case ATT_FIND_INFORMATION_REQ: {
        if (att_len < 5) return;
        uint16_t start = att[1] | (att[2] << 8);
        uint16_t end   = att[3] | (att[4] << 8);

        uint8_t rsp[64];
        int pos = 0;
        rsp[pos++] = ATT_FIND_INFORMATION_RSP;
        rsp[pos++] = 0x01;  /* Format: UUID16 */
        bool found = false;

        for (int i = 0; i < GATT_DB_SIZE; i++) {
            if (gatt_db[i].handle >= start &&
                gatt_db[i].handle <= end) {
                if (pos + 4 > (int)sizeof(rsp)) break;
                rsp[pos++] = gatt_db[i].handle & 0xFF;
                rsp[pos++] = (gatt_db[i].handle >> 8) & 0xFF;
                rsp[pos++] = gatt_db[i].uuid16 & 0xFF;
                rsp[pos++] = (gatt_db[i].uuid16 >> 8) & 0xFF;
                found = true;
            }
        }
        if (!found) {
            bt_att_error(s, opcode, start, ATT_ERR_ATTR_NOT_FOUND);
        } else {
            bt_send_acl(s, s->conn_handle, L2CAP_CID_ATT, rsp, pos);
        }
        break;
    }

    case ATT_READ_REQ: {
        if (att_len < 3) return;
        uint16_t handle = att[1] | (att[2] << 8);
        const GattAttr *a = gatt_find_by_handle(handle);
        if (!a) {
            bt_att_error(s, opcode, handle, ATT_ERR_INVALID_HANDLE);
            break;
        }
        uint8_t rsp[64];
        rsp[0] = ATT_READ_RSP;
        int rlen = 1;
        if (a->uuid16 == UUID_QEMU_RW_CHAR) {
            memcpy(&rsp[1], s->rw_char_value, s->rw_char_len);
            rlen += s->rw_char_len;
        } else if (a->uuid16 == UUID_CCC_DESCRIPTOR) {
            rsp[1] = s->notify_ccc & 0xFF;
            rsp[2] = (s->notify_ccc >> 8) & 0xFF;
            rlen += 2;
        } else if (a->uuid16 == UUID_QEMU_NOTIFY_CHAR) {
            /* Return a counter-like value */
            rsp[1] = 0x42;
            rlen += 1;
        } else if (a->value && a->value_len > 0) {
            memcpy(&rsp[1], a->value, MIN(a->value_len, (uint8_t)62));
            rlen += MIN(a->value_len, (uint8_t)62);
        }
        bt_send_acl(s, s->conn_handle, L2CAP_CID_ATT, rsp, rlen);
        break;
    }

    case ATT_WRITE_REQ:
    case ATT_WRITE_CMD: {
        if (att_len < 3) return;
        uint16_t handle = att[1] | (att[2] << 8);
        const uint8_t *wdata = &att[3];
        uint16_t wlen = att_len - 3;

        if (handle == 0x000A) {
            /* R/W characteristic value */
            s->rw_char_len = MIN(wlen, BT_GATT_RW_CHAR_MAX_LEN);
            memcpy(s->rw_char_value, wdata, s->rw_char_len);
        } else if (handle == 0x000D) {
            /* CCC descriptor */
            if (wlen >= 2) {
                s->notify_ccc = wdata[0] | (wdata[1] << 8);
            }
        }
        /* Write Request needs response; Write Command does not */
        if (opcode == ATT_WRITE_REQ) {
            uint8_t rsp[1] = { ATT_WRITE_RSP };
            bt_send_acl(s, s->conn_handle, L2CAP_CID_ATT, rsp, 1);
        }
        break;
    }

    default:
        qemu_log_mask(LOG_UNIMP, "%s: unhandled ATT opcode 0x%02X\n",
                      __func__, opcode);
        bt_att_error(s, opcode, 0x0000, ATT_ERR_UNLIKELY);
        break;
    }
}

/* Process an ACL data packet from the host */
static void bt_acl_process(ESP32S3BtState *s)
{
    if (s->tx_pos < 5) return;  /* H4(1) + ACL_hdr(4) minimum */

    /* Parse ACL header */
    uint16_t acl_hdr = s->tx_buf[1] | (s->tx_buf[2] << 8);
    uint16_t conn_handle = acl_hdr & 0x0FFF;
    uint16_t acl_len = s->tx_buf[3] | (s->tx_buf[4] << 8);
    (void)acl_len;

    if (conn_handle != s->conn_handle || s->conn_handle == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: ACL for unknown handle 0x%04X\n",
                      __func__, conn_handle);
        return;
    }

    /* Parse L2CAP header (4 bytes after ACL header) */
    if (s->tx_pos < 9) return;
    uint16_t l2cap_len = s->tx_buf[5] | (s->tx_buf[6] << 8);
    uint16_t l2cap_cid = s->tx_buf[7] | (s->tx_buf[8] << 8);

    const uint8_t *payload = &s->tx_buf[9];
    uint16_t payload_avail = s->tx_pos - 9;
    uint16_t plen = MIN(l2cap_len, payload_avail);

    if (l2cap_cid == L2CAP_CID_ATT) {
        bt_att_handle(s, payload, plen);
    } else if (l2cap_cid == L2CAP_CID_SMP) {
        /* SMP pairing: stub — send Pairing Failed (not supported) */
        uint8_t rsp[2] = { 0x05, 0x05 };  /* Pairing Failed: Pairing Not Supported */
        bt_send_acl(s, conn_handle, L2CAP_CID_SMP, rsp, 2);
    } else {
        qemu_log_mask(LOG_UNIMP, "%s: unhandled L2CAP CID 0x%04X\n",
                      __func__, l2cap_cid);
    }

    /* Send Number of Completed Packets event */
    {
        uint8_t buf[BT_HCI_RX_BUF_SIZE];
        int pos = 0;
        buf[pos++] = HCI_H4_EVT;
        buf[pos++] = HCI_EVT_NUM_COMP_PKTS;
        buf[pos++] = 5;  /* parameter length */
        buf[pos++] = 1;  /* Num_Handles */
        buf[pos++] = conn_handle & 0xFF;
        buf[pos++] = (conn_handle >> 8) & 0xFF;
        buf[pos++] = 1;  /* Num_Completed = 1 */
        buf[pos++] = 0;
        bt_hci_enqueue_rx(s, buf, pos);
    }
}

/* ================================================================== */
/*  HCI Command Processor                                              */
/* ================================================================== */
static void bt_hci_process_command(ESP32S3BtState *s)
{
    if (s->tx_pos < 4) {
        /* Need at least H4 type(1) + opcode(2) + param_len(1) */
        return;
    }
    if (s->tx_buf[0] == HCI_H4_ACL) {
        /* Route ACL data packets to L2CAP/ATT processing */
        bt_acl_process(s);
        s->int_raw |= BT_HCI_INT_TX_DONE;
        bt_update_irq(s);
        return;
    }
    if (s->tx_buf[0] != HCI_H4_CMD) {
        /* Unknown H4 type; silently accept */
        s->int_raw |= BT_HCI_INT_TX_DONE;
        bt_update_irq(s);
        return;
    }

    uint16_t opcode = s->tx_buf[1] | (s->tx_buf[2] << 8);
    /* uint8_t plen = s->tx_buf[3]; */
    const uint8_t *cmd_params = &s->tx_buf[4];

    switch (opcode) {

    /* ---- Controller & Baseband (OGF=0x03) ---- */
    case HCI_CMD_RESET:
        s->adv_enabled  = false;
        s->scan_enabled = false;
        s->connecting   = false;
        s->conn_handle  = 0;
        s->le_event_mask = 0;
        s->event_mask    = 0x1FFFFFFFFFFF;  /* default */
        if (s->scan_timer) {
            timer_del(s->scan_timer);
        }
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_SET_EVENT_MASK:
        if (s->tx_pos >= 12) {
            memcpy(&s->event_mask, cmd_params, 8);
        }
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    /* ---- Informational (OGF=0x04) ---- */
    case HCI_CMD_READ_LOCAL_VERSION: {
        /* HCI_Version=0x09 (BLE 5.0), HCI_Revision=0x0001,
         * LMP_Version=0x09, Manufacturer=0x02E5 (Espressif),
         * LMP_Subversion=0x0001 */
        uint8_t p[8] = {0x09, 0x01, 0x00, 0x09, 0xE5, 0x02, 0x01, 0x00};
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 8);
        break;
    }

    case HCI_CMD_READ_BD_ADDR: {
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, s->bd_addr, 6);
        break;
    }

    case HCI_CMD_READ_LOCAL_COMMANDS: {
        /* Return a supported commands bitmask (simplified) */
        uint8_t p[64];
        memset(p, 0, 64);
        /* Byte 0: Inquiry not supported; Byte 14: LE commands */
        p[14] = 0xFF; /* LE Set Event Mask thru LE Set Scan Enable */
        p[25] = 0x0F; /* LE basic commands */
        p[26] = 0x0F;
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 64);
        break;
    }

    case HCI_CMD_READ_LOCAL_FEATURES: {
        /* LMP features page 0: indicate LE support */
        uint8_t p[8] = {0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00};
        /* Bit 37 = LE Supported (Host), Bit 38 = BR/EDR not supported */
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 8);
        break;
    }

    /* ---- LE Controller (OGF=0x08) ---- */
    case HCI_CMD_LE_SET_EVENT_MASK:
        if (s->tx_pos >= 12) {
            memcpy(&s->le_event_mask, cmd_params, 8);
        }
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_LE_READ_BUFFER_SIZE: {
        /* LE_ACL_Data_Packet_Length=251, Total_Num=8 */
        uint8_t p[3] = {0xFB, 0x00, 0x08};
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 3);
        break;
    }

    case HCI_CMD_LE_READ_LOCAL_FEAT: {
        /* LE Features: bit0=LE Encryption, bit5=LE Data Packet Len Ext */
        uint8_t p[8] = {0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 8);
        break;
    }

    case HCI_CMD_LE_SET_RANDOM_ADDR:
        if (s->tx_pos >= 10) {
            memcpy(s->random_addr, cmd_params, 6);
        }
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_LE_SET_ADV_PARAMS:
        /* Just accept — parameters stored (interval, type, etc.) */
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_LE_READ_ADV_TX_POWER: {
        uint8_t p[1] = {0};  /* 0 dBm */
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 1);
        break;
    }

    case HCI_CMD_LE_SET_ADV_DATA:
        if (s->tx_pos >= 5) {
            s->adv_data_len = MIN(cmd_params[0], 31);
            memcpy(s->adv_data, &cmd_params[1], s->adv_data_len);
        }
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_LE_SET_SCAN_RSP_DATA:
        if (s->tx_pos >= 5) {
            s->scan_rsp_len = MIN(cmd_params[0], 31);
            memcpy(s->scan_rsp_data, &cmd_params[1], s->scan_rsp_len);
        }
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_LE_SET_ADV_ENABLE:
        if (s->tx_pos >= 5) {
            s->adv_enabled = (cmd_params[0] != 0);
        }
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_LE_SET_SCAN_PARAMS:
        if (s->tx_pos >= 11) {
            s->scan_type     = cmd_params[0];
            s->scan_interval = cmd_params[1] | (cmd_params[2] << 8);
            s->scan_window   = cmd_params[3] | (cmd_params[4] << 8);
        }
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_LE_SET_SCAN_ENABLE:
        if (s->tx_pos >= 5) {
            bool enable = (cmd_params[0] != 0);
            s->scan_enabled = enable;
            if (enable) {
                /* Start generating periodic advertising reports from virtual peer */
                timer_mod(s->scan_timer,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 50 * SCALE_MS);
            } else {
                timer_del(s->scan_timer);
            }
        }
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_LE_CREATE_CONN: {
        /* Immediately respond with Command Status (pending), then
         * generate LE Connection Complete event from virtual peer */
        bt_send_cmd_status(s, opcode, HCI_SUCCESS);
        s->connecting = true;

        /* Generate LE Connection Complete after a short delay.
         * For immediate MVP, we do it synchronously. */
        s->conn_handle = 0x0001;  /* deterministic handle */
        uint8_t p[18];
        memset(p, 0, sizeof(p));
        p[0] = HCI_SUCCESS;          /* Status */
        p[1] = 0x01; p[2] = 0x00;    /* Connection_Handle = 0x0001 */
        p[3] = 0x01;                  /* Role: Peripheral = 1 (we initiated as Central) → 0x00 = Central */
        p[3] = 0x00;                  /* Role: Central */
        p[4] = 0x01;                  /* Peer_Address_Type: Random */
        /* Peer address */
        p[5] = 0x02; p[6] = 0x00; p[7] = 0x00;
        p[8] = 0x00; p[9] = 0x00; p[10] = 0x02;
        p[11] = 0x18; p[12] = 0x00;  /* Conn_Interval = 24 (30ms) */
        p[13] = 0x00; p[14] = 0x00;  /* Peripheral_Latency = 0 */
        p[15] = 0xC8; p[16] = 0x00;  /* Supervision_Timeout = 200 (2s) */
        p[17] = 0x00;                 /* Central_Clock_Accuracy = 0 */
        bt_send_le_meta(s, HCI_LE_EVT_CONN_COMPLETE, p, 18);
        s->connecting = false;
        break;
    }

    case HCI_CMD_LE_CREATE_CONN_CANCEL:
        s->connecting = false;
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, NULL, 0);
        break;

    case HCI_CMD_LE_READ_FILTER_LIST_SIZE: {
        uint8_t p[1] = {8};   /* Filter list size = 8 */
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 1);
        break;
    }

    case HCI_CMD_LE_READ_SUPP_STATES: {
        /* All LE states supported (bits 0-41) */
        uint8_t p[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00};
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 8);
        break;
    }

    case HCI_CMD_LE_SET_DATA_LENGTH: {
        /* Accept any data length parameters */
        uint8_t p[2] = {0x01, 0x00};  /* Connection_Handle */
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 2);
        break;
    }

    case HCI_CMD_LE_READ_MAX_DATA_LEN: {
        /* Supported_Max_TX/RX_Octets=251, Time=2120 */
        uint8_t p[8] = {0xFB, 0x00, 0x48, 0x08,
                         0xFB, 0x00, 0x48, 0x08};
        bt_send_cmd_complete(s, opcode, HCI_SUCCESS, p, 8);
        break;
    }

    /* ---- Link Control (OGF=0x01) ---- */
    case HCI_CMD_DISCONNECT: {
        if (s->conn_handle) {
            uint16_t ch = cmd_params[0] | (cmd_params[1] << 8);
            uint8_t reason = cmd_params[2];
            /* Send Command Status (pending) */
            bt_send_cmd_status(s, opcode, HCI_SUCCESS);
            /* Then Disconnection Complete event */
            uint8_t ev[BT_HCI_RX_BUF_SIZE];
            int epos = 0;
            ev[epos++] = HCI_H4_EVT;
            ev[epos++] = HCI_EVT_DISCONN_COMPLETE;
            ev[epos++] = 4;  /* param length */
            ev[epos++] = HCI_SUCCESS;
            ev[epos++] = ch & 0xFF;
            ev[epos++] = (ch >> 8) & 0xFF;
            ev[epos++] = reason;
            bt_hci_enqueue_rx(s, ev, epos);
            s->conn_handle = 0;
        } else {
            bt_send_cmd_status(s, opcode, 0x02); /* No Connection */
        }
        /* TX done already signaled by cmd_status path; skip normal path */
        goto tx_done;
    }

    case HCI_CMD_LE_CONN_UPDATE: {
        /* Accept any connection parameter update */
        bt_send_cmd_status(s, opcode, HCI_SUCCESS);
        /* Send LE Connection Update Complete event */
        if (s->conn_handle) {
            uint16_t ch = cmd_params[0] | (cmd_params[1] << 8);
            uint16_t interval = cmd_params[2] | (cmd_params[3] << 8);
            uint16_t latency  = cmd_params[4] | (cmd_params[5] << 8);
            uint16_t timeout  = cmd_params[6] | (cmd_params[7] << 8);
            uint8_t ev[BT_HCI_RX_BUF_SIZE];
            int epos = 0;
            ev[epos++] = HCI_H4_EVT;
            ev[epos++] = HCI_EVT_LE_META;
            ev[epos++] = 10;  /* param length */
            ev[epos++] = 0x03;  /* LE Connection Update Complete */
            ev[epos++] = HCI_SUCCESS;
            ev[epos++] = ch & 0xFF;
            ev[epos++] = (ch >> 8) & 0xFF;
            ev[epos++] = interval & 0xFF;
            ev[epos++] = (interval >> 8) & 0xFF;
            ev[epos++] = latency & 0xFF;
            ev[epos++] = (latency >> 8) & 0xFF;
            ev[epos++] = timeout & 0xFF;
            ev[epos++] = (timeout >> 8) & 0xFF;
            bt_hci_enqueue_rx(s, ev, epos);
        }
        goto tx_done;
    }

    default:
        /* Unknown command — respond with Command Complete + error status */
        qemu_log_mask(LOG_UNIMP,
                      "%s: unhandled HCI command 0x%04X\n", __func__, opcode);
        bt_send_cmd_complete(s, opcode, HCI_ERR_UNKNOWN_CMD, NULL, 0);
        break;
    }

    /* Signal TX done */
tx_done:
    s->int_raw |= BT_HCI_INT_TX_DONE;
    bt_update_irq(s);
}

/* ================================================================== */
/*  QEMU HCI Transport register read/write                            */
/* ================================================================== */
static uint64_t bt_hci_reg_read(ESP32S3BtState *s, hwaddr offset)
{
    switch (offset) {
    case BT_QEMU_HCI_STATUS: {
        uint32_t val = 0;
        val |= 1;                          /* bit0: TX always available */
        if (s->rx_count > 0)  val |= (1 << 1);     /* bit1: RX pending */
        if (s->controller_ready) val |= (1 << 2);    /* bit2: ctrl ready */
        val |= (s->rx_count & 0xFF) << 8;             /* [15:8]: RX count */
        return val;
    }

    case BT_QEMU_HCI_RX_LEN: {
        BtHciRxPacket *head = QTAILQ_FIRST(&s->rx_queue);
        return head ? head->len : 0;
    }

    case BT_QEMU_HCI_RX_DATA: {
        BtHciRxPacket *head = QTAILQ_FIRST(&s->rx_queue);
        if (!head || s->rx_read_pos >= head->len) {
            return 0;
        }
        /* Read up to 4 bytes (word aligned) */
        uint32_t val = 0;
        for (int i = 0; i < 4 && (s->rx_read_pos + i) < head->len; i++) {
            val |= (uint32_t)head->data[s->rx_read_pos + i] << (i * 8);
        }
        s->rx_read_pos += 4;
        return val;
    }

    case BT_QEMU_HCI_INT_RAW:
        return s->int_raw;

    case BT_QEMU_HCI_INT_ST:
        return s->int_raw & s->int_ena;

    case BT_QEMU_HCI_INT_ENA:
        return s->int_ena;

    case BT_QEMU_HCI_VERSION:
        return BT_QEMU_HCI_VERSION_VAL;

    case BT_QEMU_HCI_MAGIC:
        return BT_QEMU_HCI_MAGIC_VALUE;

    default:
        return 0;
    }
}

static void bt_hci_reg_write(ESP32S3BtState *s, hwaddr offset, uint64_t value)
{
    switch (offset) {
    case BT_QEMU_HCI_TX_LEN:
        s->tx_len = (uint16_t)value;
        s->tx_pos = 0;
        break;

    case BT_QEMU_HCI_TX_DATA: {
        /* Write up to 4 bytes into TX buffer */
        for (int i = 0; i < 4 && s->tx_pos < BT_HCI_TX_BUF_SIZE; i++) {
            s->tx_buf[s->tx_pos++] = (value >> (i * 8)) & 0xFF;
        }
        break;
    }

    case BT_QEMU_HCI_TX_PUSH:
        if (value & 1) {
            bt_hci_process_command(s);
            /* Reset TX buffer for next packet */
            s->tx_pos = 0;
            s->tx_len = 0;
        }
        break;

    case BT_QEMU_HCI_RX_POP:
        if (value & 1) {
            BtHciRxPacket *head = QTAILQ_FIRST(&s->rx_queue);
            if (head) {
                QTAILQ_REMOVE(&s->rx_queue, head, entry);
                g_free(head);
                s->rx_count--;
                s->rx_read_pos = 0;
            }
            /* Clear RX_READY if queue is now empty */
            if (s->rx_count == 0) {
                s->int_raw &= ~BT_HCI_INT_RX_READY;
            }
            bt_update_irq(s);
        }
        break;

    case BT_QEMU_HCI_CTRL: {
        bool enable = (value & BIT(0)) != 0;
        bool reset  = (value & BIT(1)) != 0;
        s->ble_mode = (value >> 2) & 0x3;

        if (reset) {
            s->adv_enabled  = false;
            s->scan_enabled = false;
            s->connecting   = false;
            s->conn_handle  = 0;
            if (s->scan_timer) {
                timer_del(s->scan_timer);
            }
        }
        if (enable && !s->controller_ready) {
            s->controller_ready = true;
            s->int_raw |= BT_HCI_INT_CTRL_READY;
            bt_update_irq(s);
        }
        if (!enable) {
            s->controller_ready = false;
        }
        break;
    }

    case BT_QEMU_HCI_INT_ENA:
        s->int_ena = (uint32_t)value;
        bt_update_irq(s);
        break;

    case BT_QEMU_HCI_INT_CLR:
        s->int_raw &= ~(uint32_t)value;
        bt_update_irq(s);
        break;

    default:
        break;
    }
}

/* ================================================================== */
/*  Top-level MMIO read/write                                          */
/* ================================================================== */
static uint64_t esp32s3_bt_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3BtState *s = ESP32S3_BT(opaque);

    if (addr >= 0xF00 && addr <= 0xFFF) {
        /* QEMU HCI transport registers */
        return bt_hci_reg_read(s, addr);
    }

    /* Generic blob register store (0x000 – 0xEFF) */
    uint32_t idx = addr / 4;
    if (idx < ESP32S3_BT_BLOB_REG_COUNT) {
        return s->blob_regs[idx];
    }
    return 0;
}

static void esp32s3_bt_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned int size)
{
    ESP32S3BtState *s = ESP32S3_BT(opaque);

    if (addr >= 0xF00 && addr <= 0xFFF) {
        /* QEMU HCI transport registers */
        bt_hci_reg_write(s, addr, value);
        return;
    }

    /* Generic blob register store */
    uint32_t idx = addr / 4;
    if (idx < ESP32S3_BT_BLOB_REG_COUNT) {
        s->blob_regs[idx] = (uint32_t)value;
    }
}

static const MemoryRegionOps esp32s3_bt_ops = {
    .read  = esp32s3_bt_read,
    .write = esp32s3_bt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ================================================================== */
/*  Device lifecycle                                                   */
/* ================================================================== */
static void esp32s3_bt_init(Object *obj)
{
    ESP32S3BtState *s = ESP32S3_BT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_bt_ops, s,
                          TYPE_ESP32S3_BT, ESP32S3_BT_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /* 7 IRQ lines */
    qdev_init_gpio_out_named(DEVICE(obj), s->irq, ESP32S3_BT_IRQ_NAME,
                             ESP32S3_BT_IRQ_COUNT);

    QTAILQ_INIT(&s->rx_queue);
}

static void esp32s3_bt_realize(DeviceState *dev, Error **errp)
{
    ESP32S3BtState *s = ESP32S3_BT(dev);

    /* Create scan timer for virtual peer advertising reports */
    s->scan_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bt_scan_timer_cb, s);
}

static void esp32s3_bt_reset_hold(Object *obj, ResetType type)
{
    ESP32S3BtState *s = ESP32S3_BT(obj);

    /* Clear blob register store */
    memset(s->blob_regs, 0, sizeof(s->blob_regs));

    /* Reset HCI transport state */
    s->controller_ready = false;
    s->ble_mode = 0;
    s->tx_len = 0;
    s->tx_pos = 0;
    s->rx_read_pos = 0;
    s->int_raw = 0;
    s->int_ena = 0;

    /* Drain RX queue */
    BtHciRxPacket *pkt, *tmp;
    QTAILQ_FOREACH_SAFE(pkt, &s->rx_queue, entry, tmp) {
        QTAILQ_REMOVE(&s->rx_queue, pkt, entry);
        g_free(pkt);
    }
    s->rx_count = 0;

    /* Reset BLE state */
    s->adv_enabled  = false;
    s->scan_enabled = false;
    s->connecting   = false;
    s->conn_handle  = 0;
    s->adv_data_len = 0;
    s->scan_rsp_len = 0;
    s->le_event_mask = 0;
    s->event_mask    = 0x1FFFFFFFFFFF;
    s->num_hci_cmd_pkts = 1;

    /* Deterministic BD_ADDR for QEMU */
    s->bd_addr[0] = 0x60;
    s->bd_addr[1] = 0x55;
    s->bd_addr[2] = 0xF9;
    s->bd_addr[3] = 0xF6;
    s->bd_addr[4] = 0x03;
    s->bd_addr[5] = 0x00;
    memset(s->random_addr, 0, 6);

    /* Reset GATT state */
    s->att_mtu = 23;
    s->rw_char_len = 0;
    memset(s->rw_char_value, 0, sizeof(s->rw_char_value));
    s->notify_ccc = 0;

    if (s->scan_timer) {
        timer_del(s->scan_timer);
    }

    /* Lower all IRQs */
    for (int i = 0; i < ESP32S3_BT_IRQ_COUNT; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
}

static void esp32s3_bt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32s3_bt_realize;
    rc->phases.hold = esp32s3_bt_reset_hold;
}

static const TypeInfo esp32s3_bt_info = {
    .name          = TYPE_ESP32S3_BT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3BtState),
    .instance_init = esp32s3_bt_init,
    .class_init    = esp32s3_bt_class_init,
};

static void esp32s3_bt_register_types(void)
{
    type_register_static(&esp32s3_bt_info);
}

type_init(esp32s3_bt_register_types);
