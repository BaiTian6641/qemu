/*
 * ESP32-S3 Wi-Fi Backend — Host network bridge with AP/STA/AP_STA support
 *
 * This module bridges the ESP32-S3 Wi-Fi SLC DMA engine to the QEMU host
 * networking stack.  It provides:
 *
 *   • QEMU NIC integration (user/slirp/TAP backends via -nic / -netdev)
 *   • 802.11 ↔ Ethernet frame conversion
 *   • Virtual AP for STA mode  (beacon, probe-resp, auth, assoc)
 *   • AP mode frame bridging   (guest AP ↔ host network)
 *   • AP_STA concurrent mode
 *   • Deterministic virtual backend (fixed BSSID, seeded timing)
 *
 * Frame path:
 *
 *   Guest (blob) ──TX──▶ SLC DMA ──▶ wifi_backend_tx() ──▶ QEMU NIC ──▶ host
 *   Guest (blob) ◀──RX── SLC DMA ◀── wifi_backend_inject_rx() ◀── QEMU NIC
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "net/net.h"
#include "hw/irq.h"

/* Forward-declare; full definition in esp32s3_wifi.h */
typedef struct ESP32S3WifiState ESP32S3WifiState;

/* ------------------------------------------------------------------ */
/*  Wi-Fi operating modes                                              */
/* ------------------------------------------------------------------ */

typedef enum {
    ESP32S3_WIFI_MODE_NULL   = 0,   /* Radio off                     */
    ESP32S3_WIFI_MODE_STA    = 1,   /* Station (client)              */
    ESP32S3_WIFI_MODE_AP     = 2,   /* Soft-AP (access point)        */
    ESP32S3_WIFI_MODE_APSTA  = 3,   /* Concurrent AP + STA           */
} Esp32s3WifiMode;

/* ------------------------------------------------------------------ */
/*  802.11 frame definitions                                           */
/* ------------------------------------------------------------------ */

/* Frame Control field: type */
#define IEEE80211_FTYPE_MGMT    0x0000
#define IEEE80211_FTYPE_CTRL    0x0004
#define IEEE80211_FTYPE_DATA    0x0008

/* Management subtypes (within frame control bits [7:4]) */
#define IEEE80211_STYPE_ASSOC_REQ    0x0000
#define IEEE80211_STYPE_ASSOC_RESP   0x0010
#define IEEE80211_STYPE_REASSOC_REQ  0x0020
#define IEEE80211_STYPE_REASSOC_RESP 0x0030
#define IEEE80211_STYPE_PROBE_REQ    0x0040
#define IEEE80211_STYPE_PROBE_RESP   0x0050
#define IEEE80211_STYPE_BEACON       0x0080
#define IEEE80211_STYPE_DISASSOC     0x00A0
#define IEEE80211_STYPE_AUTH         0x00B0
#define IEEE80211_STYPE_DEAUTH       0x00C0
#define IEEE80211_STYPE_ACTION       0x00D0

/* Data subtypes */
#define IEEE80211_STYPE_DATA         0x0000
#define IEEE80211_STYPE_QOS_DATA     0x0080

/* Frame Control flags */
#define IEEE80211_FCTL_TODS     0x0100
#define IEEE80211_FCTL_FROMDS   0x0200
#define IEEE80211_FCTL_PROTECTED 0x4000

/* Header sizes */
#define IEEE80211_MGMT_HDR_LEN  24  /* FC+Dur+Addr1+Addr2+Addr3+SeqCtl */
#define IEEE80211_DATA_HDR_LEN  24  /* same for non-QoS               */
#define IEEE80211_QOS_HDR_LEN   26  /* +2 for QoS Control             */
#define IEEE80211_LLC_SNAP_LEN   8  /* AA AA 03 00 00 00 + EtherType  */
#define IEEE80211_ADDR_LEN       6

/* LLC/SNAP header (RFC 1042) */
#define LLC_SNAP_DSAP   0xAA
#define LLC_SNAP_SSAP   0xAA
#define LLC_SNAP_CTRL   0x03

/* Ethernet header size */
#define ETH_HLEN    14
#define ETH_ALEN     6

/* Auth algorithm numbers */
#define WLAN_AUTH_OPEN          0
#define WLAN_AUTH_SHARED_KEY    1

/* Status codes */
#define WLAN_STATUS_SUCCESS     0

/* Reason codes */
#define WLAN_REASON_UNSPECIFIED 1

/* Capability info bits */
#define WLAN_CAPABILITY_ESS     0x0001
#define WLAN_CAPABILITY_IBSS    0x0002
#define WLAN_CAPABILITY_PRIVACY 0x0010

/* Information Element IDs */
#define WLAN_EID_SSID           0
#define WLAN_EID_SUPP_RATES     1
#define WLAN_EID_DS_PARAMS      3
#define WLAN_EID_TIM            5
#define WLAN_EID_RSN            48
#define WLAN_EID_EXT_SUPP_RATES 50

/* ------------------------------------------------------------------ */
/*  ESP32-S3 RX metadata (wifi_pkt_rx_ctrl_t equivalent, 28 bytes)    */
/* ------------------------------------------------------------------ */

/*
 * When injecting frames into SLC RX DMA we prepend this structure
 * to match what the hardware delivers.  The blob parses these fields
 * to extract RSSI, rate, channel, etc.
 */
typedef struct __attribute__((packed)) {
    int8_t   rssi;          /* byte  0    */
    uint8_t  rate;          /* byte  1    */
    uint8_t  _pad0;         /* byte  2    */
    uint8_t  sig_mode;      /* byte  3    */
    uint8_t  _pad1[4];      /* bytes 4-7  */
    uint8_t  mcs;           /* byte  8    */
    uint8_t  cwb;           /* byte  9    */
    uint8_t  _pad2[6];      /* bytes 10-15 */
    uint8_t  aggregation;   /* byte  16   */
    uint8_t  stbc;          /* byte  17   */
    uint8_t  _pad3[2];      /* bytes 18-19 */
    uint8_t  channel;       /* byte  20   */
    uint8_t  _pad4[3];      /* bytes 21-23 */
    uint32_t timestamp;     /* bytes 24-27 */
} Esp32s3WifiRxCtrl;

#define WIFI_RX_CTRL_SIZE   28

/* ------------------------------------------------------------------ */
/*  Virtual AP / STA association state                                 */
/* ------------------------------------------------------------------ */

typedef enum {
    WIFI_ASSOC_IDLE = 0,    /* Not associated           */
    WIFI_ASSOC_AUTH,        /* Authenticated            */
    WIFI_ASSOC_ASSOC,       /* Fully associated         */
} Esp32s3WifiAssocState;

/* Maximum number of virtual stations tracked in AP mode */
#define WIFI_AP_MAX_STATIONS  8

typedef struct {
    uint8_t             mac[ETH_ALEN];
    Esp32s3WifiAssocState state;
    uint16_t            aid;        /* Association ID */
} Esp32s3WifiApStation;

/* ------------------------------------------------------------------ */
/*  Backend state                                                      */
/* ------------------------------------------------------------------ */

/* Maximum frame size in SLC DMA buffers */
#define WIFI_MAX_FRAME_SIZE     2400
/* Default beacon interval in ms */
#define WIFI_BEACON_INTERVAL_MS 100
/* Default SSID for the virtual AP (STA mode) */
#define WIFI_DEFAULT_SSID       "QEMU_WIFI"
#define WIFI_MAX_SSID_LEN       32
/* Default channel */
#define WIFI_DEFAULT_CHANNEL    6

typedef struct Esp32s3WifiBackend {
    /* Parent reference */
    ESP32S3WifiState *wifi;

    /* QEMU networking */
    NICState        *nic;
    NICConf          nic_conf;

    /* Operating mode */
    Esp32s3WifiMode  mode;

    /* STA interface state */
    struct {
        uint8_t              mac[ETH_ALEN];   /* Our STA MAC address  */
        Esp32s3WifiAssocState assoc_state;
        uint16_t             aid;              /* Assigned AID         */
    } sta;

    /* AP interface state */
    struct {
        uint8_t              mac[ETH_ALEN];   /* Our AP MAC address   */
        char                 ssid[WIFI_MAX_SSID_LEN + 1];
        uint8_t              ssid_len;
        uint8_t              channel;
        Esp32s3WifiApStation stations[WIFI_AP_MAX_STATIONS];
        int                  num_stations;
    } ap;

    /* Virtual AP for STA mode (the AP we "connect" to) */
    struct {
        uint8_t              bssid[ETH_ALEN]; /* Virtual AP BSSID     */
        char                 ssid[WIFI_MAX_SSID_LEN + 1];
        uint8_t              ssid_len;
        uint8_t              channel;
        uint16_t             beacon_interval; /* TU (1.024 ms)        */
        QEMUTimer           *beacon_timer;
        uint16_t             seq_num;         /* Mgmt frame seq#      */
    } vap;

    /* TX/RX sequence numbers */
    uint16_t          rx_seq_num;

    /* Frame statistics */
    uint64_t          tx_frames;
    uint64_t          rx_frames;
    uint64_t          tx_bytes;
    uint64_t          rx_bytes;
    uint64_t          mgmt_tx;
    uint64_t          mgmt_rx;

    /* Enabled flag (set after wifi init succeeds) */
    bool              enabled;
} Esp32s3WifiBackend;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * Initialize the Wi-Fi backend.
 * Called from esp32s3_wifi_init() during device creation.
 *
 * @wifi:  parent Wi-Fi device state
 * @be:    backend structure to initialize
 */
void esp32s3_wifi_backend_init(ESP32S3WifiState *wifi,
                               Esp32s3WifiBackend *be);

/**
 * Realize the backend (create QEMU NIC).
 * Called from esp32s3_wifi device realize or machine init.
 *
 * @be:       backend
 * @dev:      parent device (for NIC registration)
 * @errp:     error propagation
 */
void esp32s3_wifi_backend_realize(Esp32s3WifiBackend *be,
                                  DeviceState *dev,
                                  Error **errp);

/**
 * Reset backend state.
 *
 * @be:  backend
 */
void esp32s3_wifi_backend_reset(Esp32s3WifiBackend *be);

/**
 * Set the operating mode.
 * Adjusts internal state and starts/stops virtual AP beacons.
 *
 * @be:    backend
 * @mode:  new mode (NULL/STA/AP/APSTA)
 */
void esp32s3_wifi_backend_set_mode(Esp32s3WifiBackend *be,
                                   Esp32s3WifiMode mode);

/**
 * Set the STA MAC address.
 *
 * @be:   backend
 * @mac:  6-byte MAC address
 */
void esp32s3_wifi_backend_set_sta_mac(Esp32s3WifiBackend *be,
                                      const uint8_t mac[ETH_ALEN]);

/**
 * Set the AP MAC address + SSID.
 *
 * @be:       backend
 * @mac:      6-byte MAC address
 * @ssid:     SSID string
 * @ssid_len: length (0 to use strlen)
 */
void esp32s3_wifi_backend_set_ap_config(Esp32s3WifiBackend *be,
                                        const uint8_t mac[ETH_ALEN],
                                        const char *ssid,
                                        uint8_t ssid_len);

/**
 * Process a TX frame extracted from SLC DMA.
 * The buffer is the raw content from the DMA descriptor buffer
 * (may include hardware TX header + 802.11 frame).
 *
 * @be:   backend
 * @buf:  frame data from SLC TX descriptor
 * @len:  frame length
 */
void esp32s3_wifi_backend_tx(Esp32s3WifiBackend *be,
                             const uint8_t *buf, size_t len);

/**
 * Check whether the backend can accept an RX frame
 * (i.e., SLC RX DMA has HW-owned descriptors available).
 *
 * @be:  backend
 * @return: true if RX injection is possible
 */
bool esp32s3_wifi_backend_can_rx(Esp32s3WifiBackend *be);

/**
 * Inject an Ethernet frame into SLC RX DMA as an 802.11 data frame.
 * Called from QEMU NIC receive callback and virtual AP management.
 *
 * @be:   backend
 * @buf:  Ethernet frame (DA+SA+EtherType+payload)
 * @len:  Ethernet frame length
 * @return: number of bytes consumed, or 0 on failure
 */
int esp32s3_wifi_backend_inject_rx(Esp32s3WifiBackend *be,
                                   const uint8_t *buf, size_t len);

/**
 * Inject a raw 802.11 management frame into SLC RX DMA.
 *
 * @be:     backend
 * @frame:  complete 802.11 management frame
 * @len:    frame length
 * @return: number of bytes consumed, or 0 on failure
 */
int esp32s3_wifi_backend_inject_mgmt(Esp32s3WifiBackend *be,
                                     const uint8_t *frame, size_t len);

/**
 * Enable/disable the backend.
 * When disabled, TX frames are silently dropped and RX is blocked.
 *
 * @be:      backend
 * @enable:  true to enable
 */
void esp32s3_wifi_backend_set_enabled(Esp32s3WifiBackend *be, bool enable);
