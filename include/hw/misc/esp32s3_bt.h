/*
 * ESP32-S3 BLE Controller Model
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Provides:
 *  - R/W register store at DR_REG_BT_BASE (0x60011000) absorbing blob accesses
 *  - QEMU-specific HCI transport registers (offsets 0xF00-0xFFF) for a VHCI bypass
 *  - Embedded HCI command processor handling standard BLE 5.0 commands
 *  - Virtual BLE peer for deterministic scan/connect testing
 *  - 7 IRQ outputs to the interrupt matrix (BT_MAC, BT_BB, BT_BB_NMI, RWBT,
 *    RWBLE, RWBT_NMI, RWBLE_NMI)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"
#include "qemu/fifo8.h"

/* ------------------------------------------------------------------ */
/*  Type / cast helpers                                                */
/* ------------------------------------------------------------------ */
#define TYPE_ESP32S3_BT  "esp32s3.bt"
#define ESP32S3_BT(obj)  OBJECT_CHECK(ESP32S3BtState, (obj), TYPE_ESP32S3_BT)

/* ------------------------------------------------------------------ */
/*  Register space                                                      */
/* ------------------------------------------------------------------ */
#define ESP32S3_BT_REG_SIZE       0x1000   /* 4 KiB mapped at DR_REG_BT_BASE */

/* Generic blob register store: 0x000 – 0xEFF (absorbs unknown accesses) */
#define ESP32S3_BT_BLOB_REG_COUNT (0xF00 / 4)

/* ------------------------------------------------------------------ */
/*  QEMU HCI Transport registers (offsets 0xF00 – 0xFFF)              */
/*  These are NOT real hardware — they exist only in QEMU to provide   */
/*  a clean HCI command/event pipe bypassing the proprietary blob.     */
/* ------------------------------------------------------------------ */
#define BT_QEMU_HCI_STATUS     0xF00  /* R: bit0=TX avail, bit1=RX pending,
                                        *    bit2=controller ready, [15:8]=RX pkt count */
#define BT_QEMU_HCI_TX_LEN     0xF04  /* W: length of next TX packet (host→controller) */
#define BT_QEMU_HCI_TX_DATA    0xF08  /* W: write TX packet data (word-at-a-time) */
#define BT_QEMU_HCI_TX_PUSH    0xF0C  /* W: write 1 → submit TX packet to HCI processor */
#define BT_QEMU_HCI_RX_LEN     0xF10  /* R: length of head RX packet (controller→host) */
#define BT_QEMU_HCI_RX_DATA    0xF14  /* R: read RX packet data (word-at-a-time) */
#define BT_QEMU_HCI_RX_POP     0xF18  /* W: write 1 → pop head RX packet */
#define BT_QEMU_HCI_CTRL       0xF1C  /* W: bit0=enable controller, bit1=reset,
                                        *    [3:2]=BLE mode (00=off,01=BLE), bit4=scan_en,
                                        *    bit5=adv_en, bit6=conn request */
#define BT_QEMU_HCI_INT_RAW    0xF20  /* R: raw interrupt status */
#define BT_QEMU_HCI_INT_ST     0xF24  /* R: masked interrupt status (RAW & ENA) */
#define BT_QEMU_HCI_INT_ENA    0xF28  /* R/W: interrupt enable mask */
#define BT_QEMU_HCI_INT_CLR    0xF2C  /* W: write-1-to-clear for INT_RAW */
#define BT_QEMU_HCI_VERSION    0xFF0  /* R: QEMU BLE HCI version (0x00050000 = BLE 5.0) */
#define BT_QEMU_HCI_MAGIC      0xFFC  /* R: 0x424C4551 ('BLEQ') — QEMU BLE detection */

#define BT_QEMU_HCI_MAGIC_VALUE  0x424C4551   /* 'BLEQ' */
#define BT_QEMU_HCI_VERSION_VAL  0x00050000   /* BLE 5.0 */

/* HCI interrupt bits (for INT_RAW / INT_ST / INT_ENA / INT_CLR) */
#define BT_HCI_INT_RX_READY    BIT(0)   /* RX packet available for host */
#define BT_HCI_INT_TX_DONE     BIT(1)   /* TX packet processed */
#define BT_HCI_INT_CTRL_READY  BIT(2)   /* Controller became ready */

/* ------------------------------------------------------------------ */
/*  HCI packet types (H4 transport)                                    */
/* ------------------------------------------------------------------ */
#define HCI_H4_CMD   0x01   /* Host → Controller: HCI Command */
#define HCI_H4_ACL   0x02   /* Bidirectional: ACL Data */
#define HCI_H4_EVT   0x04   /* Controller → Host: HCI Event */

/* ------------------------------------------------------------------ */
/*  HCI command opcodes (OGF<<10 | OCF)                                */
/* ------------------------------------------------------------------ */
/* Controller & Baseband (OGF 0x03) */
#define HCI_CMD_RESET                 0x0C03
#define HCI_CMD_SET_EVENT_MASK        0x0C01
/* Informational (OGF 0x04) */
#define HCI_CMD_READ_LOCAL_VERSION    0x1001
#define HCI_CMD_READ_BD_ADDR          0x1009
#define HCI_CMD_READ_LOCAL_COMMANDS   0x1002
#define HCI_CMD_READ_LOCAL_FEATURES   0x1003
/* LE Controller (OGF 0x08) */
#define HCI_CMD_LE_SET_EVENT_MASK     0x2001
#define HCI_CMD_LE_READ_BUFFER_SIZE   0x2002
#define HCI_CMD_LE_READ_LOCAL_FEAT    0x2003
#define HCI_CMD_LE_SET_RANDOM_ADDR    0x2005
#define HCI_CMD_LE_SET_ADV_PARAMS     0x2006
#define HCI_CMD_LE_READ_ADV_TX_POWER  0x2007
#define HCI_CMD_LE_SET_ADV_DATA       0x2008
#define HCI_CMD_LE_SET_SCAN_RSP_DATA  0x2009
#define HCI_CMD_LE_SET_ADV_ENABLE     0x200A
#define HCI_CMD_LE_SET_SCAN_PARAMS    0x200B
#define HCI_CMD_LE_SET_SCAN_ENABLE    0x200C
#define HCI_CMD_LE_CREATE_CONN        0x200D
#define HCI_CMD_LE_CREATE_CONN_CANCEL 0x200E
#define HCI_CMD_LE_READ_FILTER_LIST_SIZE 0x200F
#define HCI_CMD_LE_READ_SUPP_STATES   0x201C
#define HCI_CMD_LE_SET_DATA_LENGTH    0x2022
#define HCI_CMD_LE_READ_MAX_DATA_LEN  0x202F

/* HCI event codes */
#define HCI_EVT_CMD_COMPLETE          0x0E
#define HCI_EVT_CMD_STATUS            0x0F
#define HCI_EVT_LE_META               0x3E

/* LE sub-event codes */
#define HCI_LE_EVT_CONN_COMPLETE      0x01
#define HCI_LE_EVT_ADV_REPORT         0x02
#define HCI_LE_EVT_DATA_LENGTH_CHANGE 0x07

/* Additional HCI command opcodes (S7-T1: GATT support) */
#define HCI_CMD_DISCONNECT            0x0406
#define HCI_CMD_LE_CONN_UPDATE        0x2013

/* HCI statuses */
#define HCI_SUCCESS                   0x00
#define HCI_ERR_UNKNOWN_CMD           0x01
#define HCI_ERR_UNKNOWN_CONN          0x02
#define HCI_ERR_CONN_TERM_BY_HOST     0x16

/* HCI event codes (S7-T1) */
#define HCI_EVT_DISCONN_COMPLETE      0x05
#define HCI_EVT_NUM_COMP_PKTS         0x13

/* L2CAP CID definitions */
#define L2CAP_CID_ATT                 0x0004
#define L2CAP_CID_LE_SIGNALING        0x0005
#define L2CAP_CID_SMP                 0x0006

/* ATT opcodes */
#define ATT_ERROR_RSP                 0x01
#define ATT_EXCHANGE_MTU_REQ          0x02
#define ATT_EXCHANGE_MTU_RSP          0x03
#define ATT_FIND_INFORMATION_REQ      0x04
#define ATT_FIND_INFORMATION_RSP      0x05
#define ATT_READ_BY_TYPE_REQ          0x08
#define ATT_READ_BY_TYPE_RSP          0x09
#define ATT_READ_REQ                  0x0A
#define ATT_READ_RSP                  0x0B
#define ATT_READ_BY_GROUP_TYPE_REQ    0x10
#define ATT_READ_BY_GROUP_TYPE_RSP    0x11
#define ATT_WRITE_REQ                 0x12
#define ATT_WRITE_RSP                 0x13
#define ATT_HANDLE_VALUE_NTF          0x1B
#define ATT_WRITE_CMD                 0x52

/* ATT error codes */
#define ATT_ERR_INVALID_HANDLE        0x01
#define ATT_ERR_READ_NOT_PERMITTED    0x02
#define ATT_ERR_WRITE_NOT_PERMITTED   0x03
#define ATT_ERR_ATTR_NOT_FOUND        0x0A
#define ATT_ERR_UNLIKELY              0x0E

/* BLE UUID16 constants */
#define UUID_PRIMARY_SERVICE          0x2800
#define UUID_CHARACTERISTIC           0x2803
#define UUID_CCC_DESCRIPTOR           0x2902
#define UUID_GAP_SERVICE              0x1800
#define UUID_GATT_SERVICE             0x1801
#define UUID_DEVICE_NAME              0x2A00
#define UUID_APPEARANCE               0x2A01
#define UUID_QEMU_SERVICE             0xFF00  /* Custom QEMU test service */
#define UUID_QEMU_RW_CHAR             0xFF01  /* R/W characteristic */
#define UUID_QEMU_NOTIFY_CHAR         0xFF02  /* Notify characteristic */

/* Virtual GATT database limits */
#define BT_GATT_MAX_ATTRS             20
#define BT_GATT_RW_CHAR_MAX_LEN      32

/* ------------------------------------------------------------------ */
/*  IRQ indices                                                        */
/* ------------------------------------------------------------------ */
#define ESP32S3_BT_IRQ_MAC          0   /* ETS_BT_MAC_INTR_SOURCE (4) */
#define ESP32S3_BT_IRQ_BB           1   /* ETS_BT_BB_INTR_SOURCE (5) */
#define ESP32S3_BT_IRQ_BB_NMI       2   /* ETS_BT_BB_NMI_SOURCE (6) */
#define ESP32S3_BT_IRQ_RWBT         3   /* ETS_RWBT_INTR_SOURCE (7) */
#define ESP32S3_BT_IRQ_RWBLE        4   /* ETS_RWBLE_INTR_SOURCE (8) */
#define ESP32S3_BT_IRQ_RWBT_NMI     5   /* ETS_RWBT_NMI_SOURCE (9) */
#define ESP32S3_BT_IRQ_RWBLE_NMI    6   /* ETS_RWBLE_NMI_SOURCE (10) */
#define ESP32S3_BT_IRQ_COUNT        7

#define ESP32S3_BT_IRQ_NAME "esp32s3-bt-irq"

/* ------------------------------------------------------------------ */
/*  HCI TX/RX buffer sizes                                             */
/* ------------------------------------------------------------------ */
#define BT_HCI_TX_BUF_SIZE     260   /* max HCI command = 3+255 ≈ 258 */
#define BT_HCI_RX_BUF_SIZE     260   /* max HCI event = 2+255 ≈ 257 */
#define BT_HCI_RX_QUEUE_DEPTH  16    /* max queued RX packets */

/* ------------------------------------------------------------------ */
/*  Virtual BLE peer config                                            */
/* ------------------------------------------------------------------ */
#define BT_VIRT_PEER_ADDR       "\x02\x00\x00\x00\x00\x02"
#define BT_VIRT_PEER_RSSI       (-40)
#define BT_VIRT_PEER_ADV_NAME   "QEMU_BLE"

/* ------------------------------------------------------------------ */
/*  Device state                                                       */
/* ------------------------------------------------------------------ */

/* One queued RX packet */
typedef struct BtHciRxPacket {
    uint8_t  data[BT_HCI_RX_BUF_SIZE];
    uint16_t len;
    QTAILQ_ENTRY(BtHciRxPacket) entry;
} BtHciRxPacket;

typedef struct ESP32S3BtState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* Generic blob register store (0x000 – 0xEFF) */
    uint32_t blob_regs[ESP32S3_BT_BLOB_REG_COUNT];

    /* IRQ lines (7 outputs to interrupt matrix) */
    qemu_irq irq[ESP32S3_BT_IRQ_COUNT];

    /* --- QEMU HCI transport state --- */
    bool     controller_ready;
    uint8_t  ble_mode;        /* 0=off, 1=BLE */

    /* TX path (host → controller) */
    uint8_t  tx_buf[BT_HCI_TX_BUF_SIZE];
    uint16_t tx_len;          /* expected length */
    uint16_t tx_pos;          /* current write position */

    /* RX path (controller → host) — queue of packets */
    QTAILQ_HEAD(, BtHciRxPacket) rx_queue;
    int      rx_count;
    uint16_t rx_read_pos;     /* read cursor within head packet */

    /* HCI interrupt registers */
    uint32_t int_raw;
    uint32_t int_ena;

    /* --- BLE state (for virtual peer & HCI processing) --- */
    bool     adv_enabled;
    bool     scan_enabled;
    bool     connecting;
    uint16_t conn_handle;     /* 0 = no active connection */

    /* Stored advertising parameters */
    uint8_t  adv_data[31];
    uint8_t  adv_data_len;
    uint8_t  scan_rsp_data[31];
    uint8_t  scan_rsp_len;

    /* Stored scan parameters */
    uint8_t  scan_type;
    uint16_t scan_interval;
    uint16_t scan_window;

    /* Local device address */
    uint8_t  bd_addr[6];     /* deterministic: 60:55:F9:F6:03:00 */
    uint8_t  random_addr[6];

    /* LE event mask */
    uint64_t le_event_mask;
    uint64_t event_mask;

    /* Scan timer for periodic advertising reports */
    QEMUTimer *scan_timer;

    /* Number of completed HCI command packets the host may send */
    uint8_t  num_hci_cmd_pkts;

    /* --- GATT state (S7-T1) --- */
    uint16_t att_mtu;           /* negotiated ATT MTU (default 23) */
    uint8_t  rw_char_value[BT_GATT_RW_CHAR_MAX_LEN];
    uint8_t  rw_char_len;
    uint16_t notify_ccc;        /* Client Characteristic Config for notify char */

} ESP32S3BtState;
