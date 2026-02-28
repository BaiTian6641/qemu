/*
 * ESP32-S3 Wi-Fi Backend — Host network bridge
 *
 * Bridges ESP32-S3 SLC DMA Wi-Fi frames to/from the QEMU host networking
 * stack.  Supports STA, AP, and AP_STA modes with:
 *   • 802.11 ↔ Ethernet frame conversion
 *   • Virtual AP for STA mode (beacon, probe-resp, auth, assoc)
 *   • AP mode data frame bridging
 *   • Concurrent AP_STA operation
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "net/net.h"
#include "hw/irq.h"
#include "hw/net/esp32s3_wifi.h"
#include "hw/net/esp32s3_wifi_backend.h"

#define WIFI_BE_DEBUG 0

#if WIFI_BE_DEBUG
#define WIFI_BE_DPRINTF(fmt, ...) \
    qemu_log_mask(LOG_UNIMP, "esp32s3_wifi_be: " fmt, ## __VA_ARGS__)
#else
#define WIFI_BE_DPRINTF(fmt, ...) do {} while (0)
#endif

/* Default virtual AP BSSID (locally-administered) */
static const uint8_t default_vap_bssid[ETH_ALEN] = {
    0x02, 0x00, 0x00, 0x00, 0x00, 0x01
};

/* Default STA MAC (locally-administered, overridden by eFuse/config) */
static const uint8_t default_sta_mac[ETH_ALEN] = {
    0x24, 0x0A, 0xC4, 0x00, 0x00, 0x01
};

/* Default AP MAC */
static const uint8_t default_ap_mac[ETH_ALEN] = {
    0x24, 0x0A, 0xC4, 0x00, 0x00, 0x02
};

/* Supported rates (802.11b/g) for IE construction */
static const uint8_t supported_rates[] = {
    0x82, 0x84, 0x8b, 0x96,  /* 1, 2, 5.5, 11 Mbps (basic) */
    0x0c, 0x12, 0x18, 0x24   /* 6, 9, 12, 18 Mbps */
};
static const uint8_t extended_rates[] = {
    0x30, 0x48, 0x60, 0x6c   /* 24, 36, 48, 54 Mbps */
};

/* Broadcast MAC */
static const uint8_t bcast_mac[ETH_ALEN] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/* LLC/SNAP header bytes */
static const uint8_t llc_snap_hdr[6] = {
    LLC_SNAP_DSAP, LLC_SNAP_SSAP, LLC_SNAP_CTRL, 0x00, 0x00, 0x00
};

/* ================================================================== */
/*  Utility helpers                                                    */
/* ================================================================== */

static inline uint16_t le16_load(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void le16_store(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xff;
    p[1] = (v >> 8) & 0xff;
}

static inline bool is_multicast_mac(const uint8_t *mac)
{
    return (mac[0] & 0x01) != 0;
}

static inline bool is_broadcast_mac(const uint8_t *mac)
{
    return memcmp(mac, bcast_mac, ETH_ALEN) == 0;
}

/* ================================================================== */
/*  802.11 management frame builders                                   */
/* ================================================================== */

/*
 * Build a beacon / probe-response frame body.
 * Returns total frame length including 802.11 header.
 */
static int build_beacon_or_probe_resp(Esp32s3WifiBackend *be,
                                      uint8_t *buf, size_t buf_size,
                                      bool is_probe_resp,
                                      const uint8_t *da)
{
    uint8_t *p = buf;
    int hdr_len = IEEE80211_MGMT_HDR_LEN;

    if (buf_size < (size_t)(hdr_len + 12 + 2 + be->vap.ssid_len +
                            2 + sizeof(supported_rates) +
                            2 + 1 + 2 + sizeof(extended_rates) + 64)) {
        return -1;
    }

    /* --- 802.11 MAC header --- */
    uint16_t fc;
    if (is_probe_resp) {
        fc = IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_PROBE_RESP;
    } else {
        fc = IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_BEACON;
    }
    le16_store(p, fc);    p += 2;              /* Frame Control  */
    le16_store(p, 0);     p += 2;              /* Duration       */
    memcpy(p, da, ETH_ALEN);  p += ETH_ALEN;  /* Addr1 (DA)     */
    memcpy(p, be->vap.bssid, ETH_ALEN); p += ETH_ALEN; /* Addr2 (SA=BSSID) */
    memcpy(p, be->vap.bssid, ETH_ALEN); p += ETH_ALEN; /* Addr3 (BSSID) */
    /* Sequence control */
    uint16_t seq = (be->vap.seq_num++ & 0x0FFF) << 4;
    le16_store(p, seq);   p += 2;

    /* --- Fixed parameters (12 bytes) --- */
    /* Timestamp (8 bytes) — use virtual clock */
    uint64_t ts = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    memcpy(p, &ts, 8);   p += 8;
    /* Beacon interval (in TU = 1.024 ms) */
    le16_store(p, be->vap.beacon_interval);  p += 2;
    /* Capability info */
    le16_store(p, WLAN_CAPABILITY_ESS);      p += 2;

    /* --- Information Elements --- */
    /* SSID */
    *p++ = WLAN_EID_SSID;
    *p++ = be->vap.ssid_len;
    memcpy(p, be->vap.ssid, be->vap.ssid_len);
    p += be->vap.ssid_len;

    /* Supported Rates */
    *p++ = WLAN_EID_SUPP_RATES;
    *p++ = sizeof(supported_rates);
    memcpy(p, supported_rates, sizeof(supported_rates));
    p += sizeof(supported_rates);

    /* DS Parameter Set (channel) */
    *p++ = WLAN_EID_DS_PARAMS;
    *p++ = 1;
    *p++ = be->vap.channel;

    if (!is_probe_resp) {
        /* TIM (for beacon only — minimal) */
        *p++ = WLAN_EID_TIM;
        *p++ = 4;   /* length */
        *p++ = 0;   /* DTIM count */
        *p++ = 1;   /* DTIM period */
        *p++ = 0;   /* Bitmap control */
        *p++ = 0;   /* Partial virtual bitmap */
    }

    /* Extended Supported Rates */
    *p++ = WLAN_EID_EXT_SUPP_RATES;
    *p++ = sizeof(extended_rates);
    memcpy(p, extended_rates, sizeof(extended_rates));
    p += sizeof(extended_rates);

    return (int)(p - buf);
}

/*
 * Build an authentication response frame.
 * Auth algorithm 0 (Open System), seq 2, status 0 (success).
 */
static int build_auth_response(Esp32s3WifiBackend *be,
                               uint8_t *buf, size_t buf_size,
                               const uint8_t *da)
{
    uint8_t *p = buf;

    if (buf_size < IEEE80211_MGMT_HDR_LEN + 6) {
        return -1;
    }

    /* Header */
    uint16_t fc = IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_AUTH;
    le16_store(p, fc);    p += 2;
    le16_store(p, 0);     p += 2;              /* Duration */
    memcpy(p, da, ETH_ALEN);  p += ETH_ALEN;  /* Addr1 (DA) */
    memcpy(p, be->vap.bssid, ETH_ALEN); p += ETH_ALEN;
    memcpy(p, be->vap.bssid, ETH_ALEN); p += ETH_ALEN;
    uint16_t seq = (be->vap.seq_num++ & 0x0FFF) << 4;
    le16_store(p, seq);   p += 2;

    /* Body */
    le16_store(p, WLAN_AUTH_OPEN);      p += 2; /* Auth algorithm */
    le16_store(p, 2);                   p += 2; /* Auth seq# 2 (response) */
    le16_store(p, WLAN_STATUS_SUCCESS); p += 2; /* Status code */

    return (int)(p - buf);
}

/*
 * Build an association response frame.
 */
static int build_assoc_response(Esp32s3WifiBackend *be,
                                uint8_t *buf, size_t buf_size,
                                const uint8_t *da,
                                uint16_t aid)
{
    uint8_t *p = buf;

    if (buf_size < IEEE80211_MGMT_HDR_LEN + 6 +
                   2 + sizeof(supported_rates) +
                   2 + sizeof(extended_rates)) {
        return -1;
    }

    /* Header */
    uint16_t fc = IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ASSOC_RESP;
    le16_store(p, fc);    p += 2;
    le16_store(p, 0);     p += 2;
    memcpy(p, da, ETH_ALEN);  p += ETH_ALEN;
    memcpy(p, be->vap.bssid, ETH_ALEN); p += ETH_ALEN;
    memcpy(p, be->vap.bssid, ETH_ALEN); p += ETH_ALEN;
    uint16_t seq = (be->vap.seq_num++ & 0x0FFF) << 4;
    le16_store(p, seq);   p += 2;

    /* Body */
    le16_store(p, WLAN_CAPABILITY_ESS); p += 2; /* Capability */
    le16_store(p, WLAN_STATUS_SUCCESS); p += 2; /* Status */
    le16_store(p, aid | 0xC000);        p += 2; /* AID (bits 14-15 set) */

    /* Supported Rates */
    *p++ = WLAN_EID_SUPP_RATES;
    *p++ = sizeof(supported_rates);
    memcpy(p, supported_rates, sizeof(supported_rates));
    p += sizeof(supported_rates);

    /* Extended Supported Rates */
    *p++ = WLAN_EID_EXT_SUPP_RATES;
    *p++ = sizeof(extended_rates);
    memcpy(p, extended_rates, sizeof(extended_rates));
    p += sizeof(extended_rates);

    return (int)(p - buf);
}

/* ================================================================== */
/*  802.11 ↔ Ethernet frame conversion                                */
/* ================================================================== */

/*
 * Convert an 802.11 data frame to Ethernet.
 *
 * Input:  802.11 header + LLC/SNAP + payload
 * Output: Ethernet header (DA+SA+EtherType) + payload
 *
 * Returns length of Ethernet frame, or -1 on error.
 */
static int wifi_to_ethernet(const uint8_t *wifi_frame, size_t wifi_len,
                            uint8_t *eth_buf, size_t eth_buf_size)
{
    if (wifi_len < IEEE80211_DATA_HDR_LEN + IEEE80211_LLC_SNAP_LEN) {
        return -1;
    }

    uint16_t fc = le16_load(wifi_frame);
    uint16_t ftype = fc & 0x000C;
    if (ftype != IEEE80211_FTYPE_DATA) {
        return -1;  /* Not a data frame */
    }

    /* Determine header length (QoS adds 2 bytes) */
    int hdr_len = IEEE80211_DATA_HDR_LEN;
    uint16_t stype = fc & 0x00F0;
    if (stype & 0x0080) {
        hdr_len = IEEE80211_QOS_HDR_LEN;
    }

    /* Check ToDS/FromDS to determine address mapping */
    bool to_ds   = (fc & IEEE80211_FCTL_TODS) != 0;
    bool from_ds = (fc & IEEE80211_FCTL_FROMDS) != 0;

    const uint8_t *da, *sa;
    if (!to_ds && !from_ds) {
        /* IBSS: Addr1=DA, Addr2=SA, Addr3=BSSID */
        da = wifi_frame + 4;
        sa = wifi_frame + 10;
    } else if (to_ds && !from_ds) {
        /* STA→AP: Addr1=BSSID, Addr2=SA, Addr3=DA */
        da = wifi_frame + 16;
        sa = wifi_frame + 10;
    } else if (!to_ds && from_ds) {
        /* AP→STA: Addr1=DA, Addr2=BSSID, Addr3=SA */
        da = wifi_frame + 4;
        sa = wifi_frame + 16;
    } else {
        /* WDS (4-addr): Addr1=RA, Addr2=TA, Addr3=DA, Addr4=SA */
        if (wifi_len < (size_t)(hdr_len + 6 + IEEE80211_LLC_SNAP_LEN)) {
            return -1;
        }
        da = wifi_frame + 16;
        sa = wifi_frame + hdr_len;
        hdr_len += 6;  /* Addr4 */
    }

    /* Verify LLC/SNAP header */
    const uint8_t *llc = wifi_frame + hdr_len;
    if (wifi_len < (size_t)(hdr_len + IEEE80211_LLC_SNAP_LEN)) {
        return -1;
    }
    if (llc[0] != LLC_SNAP_DSAP || llc[1] != LLC_SNAP_SSAP ||
        llc[2] != LLC_SNAP_CTRL) {
        /* Not LLC/SNAP — try to handle as raw payload */
        WIFI_BE_DPRINTF("non-LLC/SNAP frame, treating as raw\n");
    }

    /* EtherType is at llc + 6 */
    uint16_t ethertype = ((uint16_t)llc[6] << 8) | llc[7];

    /* Payload starts after LLC/SNAP */
    const uint8_t *payload = wifi_frame + hdr_len + IEEE80211_LLC_SNAP_LEN;
    size_t payload_len = wifi_len - hdr_len - IEEE80211_LLC_SNAP_LEN;

    size_t eth_len = ETH_HLEN + payload_len;
    if (eth_len > eth_buf_size) {
        return -1;
    }

    /* Build Ethernet frame */
    memcpy(eth_buf, da, ETH_ALEN);           /* DA */
    memcpy(eth_buf + ETH_ALEN, sa, ETH_ALEN); /* SA */
    eth_buf[12] = (ethertype >> 8) & 0xff;
    eth_buf[13] = ethertype & 0xff;
    memcpy(eth_buf + ETH_HLEN, payload, payload_len);

    return (int)eth_len;
}

/*
 * Convert an Ethernet frame to an 802.11 data frame.
 *
 * Prepends RX metadata + 802.11 header + LLC/SNAP.
 * The direction flags (ToDS/FromDS) are set based on mode:
 *   STA mode RX: FromDS=1, ToDS=0 (AP→STA)
 *   AP mode RX:  FromDS=0, ToDS=1 (STA→AP)
 *
 * Returns total length (rx_ctrl + 802.11 + LLC/SNAP + payload).
 */
static int ethernet_to_wifi_rx(Esp32s3WifiBackend *be,
                               const uint8_t *eth_frame, size_t eth_len,
                               uint8_t *wifi_buf, size_t wifi_buf_size,
                               bool for_ap_interface)
{
    if (eth_len < ETH_HLEN) {
        return -1;
    }

    const uint8_t *da = eth_frame;
    const uint8_t *sa = eth_frame + ETH_ALEN;
    uint8_t ethertype_hi = eth_frame[12];
    uint8_t ethertype_lo = eth_frame[13];
    const uint8_t *payload = eth_frame + ETH_HLEN;
    size_t payload_len = eth_len - ETH_HLEN;

    size_t total = WIFI_RX_CTRL_SIZE + IEEE80211_DATA_HDR_LEN +
                   IEEE80211_LLC_SNAP_LEN + payload_len;
    if (total > wifi_buf_size) {
        return -1;
    }

    uint8_t *p = wifi_buf;

    /* --- RX metadata (wifi_pkt_rx_ctrl_t) --- */
    memset(p, 0, WIFI_RX_CTRL_SIZE);
    p[0] = (uint8_t)(int8_t)(-30);  /* RSSI = -30 dBm (good signal) */
    p[1] = 0x0B;                    /* rate: 11 Mbps */
    p[20] = be->vap.channel;        /* channel */
    /* sig_len at offset variable — we set frame length below */
    p += WIFI_RX_CTRL_SIZE;

    /* --- 802.11 data frame header --- */
    uint16_t fc = IEEE80211_FTYPE_DATA | IEEE80211_STYPE_DATA;
    if (for_ap_interface) {
        /* Received by our AP: ToDS=1, FromDS=0 */
        fc |= IEEE80211_FCTL_TODS;
        le16_store(p, fc);    p += 2;
        le16_store(p, 0);     p += 2;  /* Duration */
        memcpy(p, be->ap.mac, ETH_ALEN);   p += ETH_ALEN; /* Addr1=BSSID(us) */
        memcpy(p, sa, ETH_ALEN);            p += ETH_ALEN; /* Addr2=SA */
        memcpy(p, da, ETH_ALEN);            p += ETH_ALEN; /* Addr3=DA */
    } else {
        /* Received by our STA: FromDS=1, ToDS=0 */
        fc |= IEEE80211_FCTL_FROMDS;
        le16_store(p, fc);    p += 2;
        le16_store(p, 0);     p += 2;  /* Duration */
        memcpy(p, da, ETH_ALEN);            p += ETH_ALEN; /* Addr1=DA */
        memcpy(p, be->vap.bssid, ETH_ALEN); p += ETH_ALEN; /* Addr2=BSSID */
        memcpy(p, sa, ETH_ALEN);            p += ETH_ALEN; /* Addr3=SA */
    }
    /* Sequence Control */
    uint16_t seq = (be->rx_seq_num++ & 0x0FFF) << 4;
    le16_store(p, seq);  p += 2;

    /* --- LLC/SNAP header --- */
    memcpy(p, llc_snap_hdr, 6);  p += 6;
    *p++ = ethertype_hi;
    *p++ = ethertype_lo;

    /* --- Payload --- */
    memcpy(p, payload, payload_len);
    p += payload_len;

    return (int)(p - wifi_buf);
}

/* ================================================================== */
/*  SLC DMA RX injection                                               */
/* ================================================================== */

/*
 * Inject a pre-built frame (with RX metadata) into SLC0 RX DMA.
 *
 * Walks the RX descriptor chain looking for an HW-owned descriptor
 * with enough buffer space. Writes the frame data, updates length,
 * flips owner to SW, sets EOF, and raises RX_DONE + RX_EOF interrupts.
 */
static int inject_frame_to_slc_rx(Esp32s3WifiBackend *be,
                                  const uint8_t *frame, size_t frame_len)
{
    ESP32S3WifiState *s = be->wifi;

    if (!s || !s->slc0_rx_cur_desc) {
        /* No RX descriptor chain active — try the link register */
        if (s && s->slc0_rx_link) {
            s->slc0_rx_cur_desc = s->slc0_rx_link & SLC_LINK_ADDR_MASK;
        }
        if (!s || !s->slc0_rx_cur_desc) {
            WIFI_BE_DPRINTF("inject_frame: no RX descriptor available\n");
            return 0;
        }
    }

    hwaddr desc_addr = s->slc0_rx_cur_desc;
    int tries = 0;

    while (desc_addr && tries < SLC_DMA_MAX_DESCRIPTORS) {
        uint32_t word0, buf_ptr, next_ptr;

        /* Read descriptor */
        uint32_t desc_buf[3];
        cpu_physical_memory_read(desc_addr, desc_buf, LLDESC_SIZE_BYTES);
        word0    = le32_to_cpu(desc_buf[0]);
        buf_ptr  = le32_to_cpu(desc_buf[1]);
        next_ptr = le32_to_cpu(desc_buf[2]);

        /* Must be HW-owned */
        if (!(word0 & LLDESC_OWNER_MASK)) {
            /* SW owns it — try next */
            if (next_ptr == 0) {
                break;
            }
            desc_addr = next_ptr;
            tries++;
            continue;
        }

        /* Check buffer capacity */
        uint32_t buf_size = word0 & LLDESC_SIZE_MASK;
        if ((size_t)buf_size < frame_len) {
            WIFI_BE_DPRINTF("inject_frame: buffer too small (%u < %zu)\n",
                            buf_size, frame_len);
            /* Try next descriptor */
            if (next_ptr == 0) {
                break;
            }
            desc_addr = next_ptr;
            tries++;
            continue;
        }

        /* Write frame data to descriptor buffer */
        cpu_physical_memory_write(buf_ptr, frame, frame_len);

        /* Update descriptor: set length, set EOF, flip owner to SW */
        word0 &= ~LLDESC_LENGTH_MASK;
        word0 |= ((uint32_t)frame_len << LLDESC_LENGTH_SHIFT) & LLDESC_LENGTH_MASK;
        word0 |= LLDESC_EOF_MASK;
        word0 &= ~LLDESC_OWNER_MASK;  /* SW owned now */

        uint32_t w0_le = cpu_to_le32(word0);
        cpu_physical_memory_write(desc_addr, &w0_le, 4);

        /* Advance to next descriptor for future injections */
        s->slc0_rx_cur_desc = next_ptr;

        /* Raise RX_DONE + RX_EOF interrupts */
        s->slc0_int_raw |= SLC0_INT_RX_DONE | SLC0_INT_RX_EOF;
        esp32s3_wifi_slc_update_irq(s);

        WIFI_BE_DPRINTF("inject_frame: %zu bytes to desc 0x%08" HWADDR_PRIx "\n",
                        frame_len, desc_addr);
        return (int)frame_len;
    }

    WIFI_BE_DPRINTF("inject_frame: no suitable RX descriptor found\n");
    return 0;
}

/* ================================================================== */
/*  Virtual AP management (for STA mode)                               */
/* ================================================================== */

static void vap_beacon_timer_cb(void *opaque)
{
    Esp32s3WifiBackend *be = (Esp32s3WifiBackend *)opaque;

    if (!be->enabled || (be->mode != ESP32S3_WIFI_MODE_STA &&
                         be->mode != ESP32S3_WIFI_MODE_APSTA)) {
        return;
    }

    /* Build and inject a beacon frame */
    uint8_t frame_buf[512];
    int mgmt_len = build_beacon_or_probe_resp(be, frame_buf, sizeof(frame_buf),
                                               false, bcast_mac);
    if (mgmt_len > 0) {
        /* Prepend RX metadata */
        uint8_t rx_buf[WIFI_MAX_FRAME_SIZE];
        memset(rx_buf, 0, WIFI_RX_CTRL_SIZE);
        rx_buf[0] = (uint8_t)(int8_t)(-40);  /* RSSI */
        rx_buf[1] = 0x02;                    /* rate 1 Mbps */
        rx_buf[20] = be->vap.channel;

        if (WIFI_RX_CTRL_SIZE + mgmt_len <= (int)sizeof(rx_buf)) {
            memcpy(rx_buf + WIFI_RX_CTRL_SIZE, frame_buf, mgmt_len);
            inject_frame_to_slc_rx(be, rx_buf, WIFI_RX_CTRL_SIZE + mgmt_len);
            be->mgmt_rx++;
        }
    }

    /* Re-arm timer */
    int64_t interval_ns = (int64_t)be->vap.beacon_interval * 1024 * 1000;
    timer_mod(be->vap.beacon_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + interval_ns);
}

/*
 * Handle an 802.11 management frame from SLC TX DMA.
 * This is a frame the blob is transmitting (e.g., probe-req, auth, assoc-req).
 * Our virtual AP responds appropriately.
 */
static void handle_mgmt_tx_frame(Esp32s3WifiBackend *be,
                                 const uint8_t *frame, size_t len)
{
    if (len < IEEE80211_MGMT_HDR_LEN + 2) {
        return;
    }

    uint16_t fc = le16_load(frame);
    uint16_t stype = fc & 0x00F0;
    const uint8_t *sa = frame + 10;  /* Addr2 = transmitter */

    uint8_t resp_buf[512];
    uint8_t rx_buf[WIFI_MAX_FRAME_SIZE];
    int resp_len = -1;

    be->mgmt_tx++;

    switch (stype) {
    case IEEE80211_STYPE_PROBE_REQ:
        WIFI_BE_DPRINTF("mgmt TX: probe-request from %02x:%02x:%02x:%02x:%02x:%02x\n",
                        sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
        resp_len = build_beacon_or_probe_resp(be, resp_buf, sizeof(resp_buf),
                                               true, sa);
        break;

    case IEEE80211_STYPE_AUTH:
        WIFI_BE_DPRINTF("mgmt TX: auth from %02x:%02x:%02x:%02x:%02x:%02x\n",
                        sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
        be->sta.assoc_state = WIFI_ASSOC_AUTH;
        resp_len = build_auth_response(be, resp_buf, sizeof(resp_buf), sa);
        break;

    case IEEE80211_STYPE_ASSOC_REQ:
    case IEEE80211_STYPE_REASSOC_REQ:
        WIFI_BE_DPRINTF("mgmt TX: assoc-request from %02x:%02x:%02x:%02x:%02x:%02x\n",
                        sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
        be->sta.assoc_state = WIFI_ASSOC_ASSOC;
        be->sta.aid = 1;
        resp_len = build_assoc_response(be, resp_buf, sizeof(resp_buf),
                                         sa, be->sta.aid);
        break;

    case IEEE80211_STYPE_DISASSOC:
    case IEEE80211_STYPE_DEAUTH:
        WIFI_BE_DPRINTF("mgmt TX: deauth/disassoc from STA\n");
        be->sta.assoc_state = WIFI_ASSOC_IDLE;
        be->sta.aid = 0;
        break;

    default:
        WIFI_BE_DPRINTF("mgmt TX: unhandled stype=0x%04x\n", stype);
        break;
    }

    /* Inject response into RX DMA */
    if (resp_len > 0) {
        memset(rx_buf, 0, WIFI_RX_CTRL_SIZE);
        rx_buf[0] = (uint8_t)(int8_t)(-30);
        rx_buf[1] = 0x0B;
        rx_buf[20] = be->vap.channel;

        if (WIFI_RX_CTRL_SIZE + resp_len <= (int)sizeof(rx_buf)) {
            memcpy(rx_buf + WIFI_RX_CTRL_SIZE, resp_buf, resp_len);
            inject_frame_to_slc_rx(be, rx_buf, WIFI_RX_CTRL_SIZE + resp_len);
            be->mgmt_rx++;
        }
    }
}

/* ================================================================== */
/*  SLC TX frame processing                                            */
/* ================================================================== */

void esp32s3_wifi_backend_tx(Esp32s3WifiBackend *be,
                             const uint8_t *buf, size_t len)
{
    if (!be || !be->enabled || len < 2) {
        return;
    }

    uint16_t fc = le16_load(buf);
    uint16_t ftype = fc & 0x000C;

    WIFI_BE_DPRINTF("backend_tx: len=%zu fc=0x%04x type=%u\n",
                    len, fc, ftype >> 2);

    if (ftype == IEEE80211_FTYPE_MGMT) {
        /* Management frame — handle locally for virtual AP */
        if (be->mode == ESP32S3_WIFI_MODE_STA ||
            be->mode == ESP32S3_WIFI_MODE_APSTA) {
            handle_mgmt_tx_frame(be, buf, len);
        }
        return;
    }

    if (ftype != IEEE80211_FTYPE_DATA) {
        /* Control frames are silently consumed */
        return;
    }

    /* Data frame → convert to Ethernet and send via NIC */
    uint8_t eth_buf[WIFI_MAX_FRAME_SIZE];
    int eth_len = wifi_to_ethernet(buf, len, eth_buf, sizeof(eth_buf));

    if (eth_len > 0 && be->nic) {
        qemu_send_packet(qemu_get_queue(be->nic), eth_buf, eth_len);
        be->tx_frames++;
        be->tx_bytes += eth_len;
        WIFI_BE_DPRINTF("backend_tx: sent %d byte Ethernet frame\n", eth_len);
    } else if (eth_len <= 0) {
        /*
         * Fallback: if 802.11 parsing fails, try treating the buffer
         * as a raw Ethernet frame (some blob versions may shortcut).
         */
        if (len >= ETH_HLEN && be->nic) {
            qemu_send_packet(qemu_get_queue(be->nic), buf, len);
            be->tx_frames++;
            be->tx_bytes += len;
            WIFI_BE_DPRINTF("backend_tx: sent %zu bytes as raw Ethernet (fallback)\n",
                            len);
        } else {
            WIFI_BE_DPRINTF("backend_tx: frame conversion failed, dropped\n");
        }
    }
}

/* ================================================================== */
/*  QEMU NIC callbacks (host → guest RX)                               */
/* ================================================================== */

static bool wifi_nic_can_receive(NetClientState *nc)
{
    Esp32s3WifiBackend *be = qemu_get_nic_opaque(nc);
    if (!be || !be->enabled || !be->wifi) {
        return false;
    }

    /* Must have an active RX descriptor chain */
    ESP32S3WifiState *s = be->wifi;
    return (s->slc0_rx_cur_desc != 0 ||
            (s->slc0_rx_link & SLC_LINK_ADDR_MASK) != 0);
}

static ssize_t wifi_nic_receive(NetClientState *nc,
                                const uint8_t *buf, size_t size)
{
    Esp32s3WifiBackend *be = qemu_get_nic_opaque(nc);
    if (!be || !be->enabled) {
        return -1;
    }

    WIFI_BE_DPRINTF("nic_receive: %zu bytes from host\n", size);

    /* Determine which interface receives this frame */
    bool for_ap = false;
    if (be->mode == ESP32S3_WIFI_MODE_AP) {
        for_ap = true;
    } else if (be->mode == ESP32S3_WIFI_MODE_APSTA) {
        /* In AP_STA mode, route based on DA:
         * If DA matches our AP MAC or is broadcast → AP interface
         * Otherwise → STA interface */
        const uint8_t *da = buf;
        if (is_broadcast_mac(da) || is_multicast_mac(da) ||
            memcmp(da, be->ap.mac, ETH_ALEN) == 0) {
            for_ap = true;
        }
    }

    /* Convert Ethernet → 802.11 and inject into SLC RX DMA */
    uint8_t wifi_buf[WIFI_MAX_FRAME_SIZE];
    int wifi_len = ethernet_to_wifi_rx(be, buf, size, wifi_buf, sizeof(wifi_buf),
                                       for_ap);
    if (wifi_len > 0) {
        int injected = inject_frame_to_slc_rx(be, wifi_buf, wifi_len);
        if (injected > 0) {
            be->rx_frames++;
            be->rx_bytes += size;
            return size;
        }
    }

    WIFI_BE_DPRINTF("nic_receive: injection failed\n");
    return -1;
}

static void wifi_nic_link_status_changed(NetClientState *nc)
{
    Esp32s3WifiBackend *be = qemu_get_nic_opaque(nc);
    if (!be) {
        return;
    }

    bool up = !nc->link_down;
    WIFI_BE_DPRINTF("link status changed: %s\n", up ? "UP" : "DOWN");

    if (!up) {
        /* Link went down — deauth STA */
        be->sta.assoc_state = WIFI_ASSOC_IDLE;
    }
}

static NetClientInfo wifi_nic_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = wifi_nic_can_receive,
    .receive = wifi_nic_receive,
    .link_status_changed = wifi_nic_link_status_changed,
};

/* ================================================================== */
/*  Public API implementation                                          */
/* ================================================================== */

void esp32s3_wifi_backend_init(ESP32S3WifiState *wifi,
                               Esp32s3WifiBackend *be)
{
    memset(be, 0, sizeof(*be));
    be->wifi = wifi;
    be->mode = ESP32S3_WIFI_MODE_NULL;
    be->enabled = false;

    /* Default MAC addresses */
    memcpy(be->sta.mac, default_sta_mac, ETH_ALEN);
    memcpy(be->ap.mac, default_ap_mac, ETH_ALEN);

    /* Default AP config */
    snprintf(be->ap.ssid, sizeof(be->ap.ssid), "%s", "ESP32S3_AP");
    be->ap.ssid_len = strlen(be->ap.ssid);
    be->ap.channel = WIFI_DEFAULT_CHANNEL;

    /* Virtual AP defaults */
    memcpy(be->vap.bssid, default_vap_bssid, ETH_ALEN);
    snprintf(be->vap.ssid, sizeof(be->vap.ssid), "%s", WIFI_DEFAULT_SSID);
    be->vap.ssid_len = strlen(be->vap.ssid);
    be->vap.channel = WIFI_DEFAULT_CHANNEL;
    be->vap.beacon_interval = 100;  /* 100 TU ≈ 102.4 ms */
    be->vap.seq_num = 0;

    /* Create beacon timer (armed when mode becomes STA/APSTA) */
    be->vap.beacon_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        vap_beacon_timer_cb, be);
}

void esp32s3_wifi_backend_realize(Esp32s3WifiBackend *be,
                                  DeviceState *dev,
                                  Error **errp)
{
    /* Use the STA MAC as the NIC's MAC address by default */
    memcpy(be->nic_conf.macaddr.a, be->sta.mac, ETH_ALEN);

    be->nic = qemu_new_nic(&wifi_nic_info, &be->nic_conf,
                           object_get_typename(OBJECT(dev)),
                           dev->id,
                           &dev->mem_reentrancy_guard, be);

    if (be->nic) {
        qemu_format_nic_info_str(qemu_get_queue(be->nic),
                                 be->nic_conf.macaddr.a);
        WIFI_BE_DPRINTF("NIC realized, MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
                        be->sta.mac[0], be->sta.mac[1], be->sta.mac[2],
                        be->sta.mac[3], be->sta.mac[4], be->sta.mac[5]);
    } else {
        qemu_log_mask(LOG_UNIMP,
                      "esp32s3_wifi_be: no NIC created (no network backend)\n");
    }
}

void esp32s3_wifi_backend_reset(Esp32s3WifiBackend *be)
{
    be->mode = ESP32S3_WIFI_MODE_NULL;
    be->enabled = false;

    be->sta.assoc_state = WIFI_ASSOC_IDLE;
    be->sta.aid = 0;

    be->ap.num_stations = 0;
    memset(be->ap.stations, 0, sizeof(be->ap.stations));

    be->vap.seq_num = 0;
    be->rx_seq_num = 0;

    be->tx_frames = 0;
    be->rx_frames = 0;
    be->tx_bytes = 0;
    be->rx_bytes = 0;
    be->mgmt_tx = 0;
    be->mgmt_rx = 0;

    /* Stop beacon timer */
    timer_del(be->vap.beacon_timer);
}

void esp32s3_wifi_backend_set_mode(Esp32s3WifiBackend *be,
                                   Esp32s3WifiMode mode)
{
    Esp32s3WifiMode old_mode = be->mode;
    be->mode = mode;

    WIFI_BE_DPRINTF("mode change: %d → %d\n", old_mode, mode);

    /* Start/stop beacon timer based on mode */
    bool need_beacons = (mode == ESP32S3_WIFI_MODE_STA ||
                         mode == ESP32S3_WIFI_MODE_APSTA);

    if (need_beacons && be->enabled) {
        int64_t interval_ns = (int64_t)be->vap.beacon_interval * 1024 * 1000;
        timer_mod(be->vap.beacon_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + interval_ns);
        WIFI_BE_DPRINTF("beacon timer started (interval=%d TU)\n",
                        be->vap.beacon_interval);
    } else {
        timer_del(be->vap.beacon_timer);
    }

    (void)old_mode;
}

void esp32s3_wifi_backend_set_sta_mac(Esp32s3WifiBackend *be,
                                      const uint8_t mac[ETH_ALEN])
{
    memcpy(be->sta.mac, mac, ETH_ALEN);
    WIFI_BE_DPRINTF("STA MAC set: %02x:%02x:%02x:%02x:%02x:%02x\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    /* Update NIC MAC if available */
    if (be->nic) {
        memcpy(be->nic_conf.macaddr.a, mac, ETH_ALEN);
        qemu_format_nic_info_str(qemu_get_queue(be->nic),
                                be->nic_conf.macaddr.a);
    }
}

void esp32s3_wifi_backend_set_ap_config(Esp32s3WifiBackend *be,
                                        const uint8_t mac[ETH_ALEN],
                                        const char *ssid,
                                        uint8_t ssid_len)
{
    memcpy(be->ap.mac, mac, ETH_ALEN);
    if (ssid_len == 0 && ssid) {
        ssid_len = strlen(ssid);
    }
    if (ssid_len > WIFI_MAX_SSID_LEN) {
        ssid_len = WIFI_MAX_SSID_LEN;
    }
    memcpy(be->ap.ssid, ssid, ssid_len);
    be->ap.ssid[ssid_len] = '\0';
    be->ap.ssid_len = ssid_len;

    WIFI_BE_DPRINTF("AP config: MAC=%02x:%02x:%02x:%02x:%02x:%02x SSID='%s'\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                    be->ap.ssid);
}

bool esp32s3_wifi_backend_can_rx(Esp32s3WifiBackend *be)
{
    if (!be || !be->enabled || !be->wifi) {
        return false;
    }
    return wifi_nic_can_receive(be->nic ?
                                qemu_get_queue(be->nic) : NULL);
}

int esp32s3_wifi_backend_inject_rx(Esp32s3WifiBackend *be,
                                   const uint8_t *buf, size_t len)
{
    if (!be || !be->enabled) {
        return 0;
    }

    bool for_ap = (be->mode == ESP32S3_WIFI_MODE_AP);

    uint8_t wifi_buf[WIFI_MAX_FRAME_SIZE];
    int wifi_len = ethernet_to_wifi_rx(be, buf, len, wifi_buf, sizeof(wifi_buf),
                                       for_ap);
    if (wifi_len > 0) {
        return inject_frame_to_slc_rx(be, wifi_buf, wifi_len);
    }
    return 0;
}

int esp32s3_wifi_backend_inject_mgmt(Esp32s3WifiBackend *be,
                                     const uint8_t *frame, size_t len)
{
    if (!be || !be->enabled) {
        return 0;
    }

    /* Prepend RX metadata */
    uint8_t rx_buf[WIFI_MAX_FRAME_SIZE];
    if (WIFI_RX_CTRL_SIZE + len > sizeof(rx_buf)) {
        return 0;
    }

    memset(rx_buf, 0, WIFI_RX_CTRL_SIZE);
    rx_buf[0] = (uint8_t)(int8_t)(-30);
    rx_buf[1] = 0x0B;
    rx_buf[20] = be->vap.channel;
    memcpy(rx_buf + WIFI_RX_CTRL_SIZE, frame, len);

    return inject_frame_to_slc_rx(be, rx_buf, WIFI_RX_CTRL_SIZE + len);
}

void esp32s3_wifi_backend_set_enabled(Esp32s3WifiBackend *be, bool enable)
{
    be->enabled = enable;
    WIFI_BE_DPRINTF("backend %s\n", enable ? "enabled" : "disabled");

    if (enable && (be->mode == ESP32S3_WIFI_MODE_STA ||
                   be->mode == ESP32S3_WIFI_MODE_APSTA)) {
        /* Start beacon timer */
        int64_t interval_ns = (int64_t)be->vap.beacon_interval * 1024 * 1000;
        timer_mod(be->vap.beacon_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + interval_ns);
    } else {
        timer_del(be->vap.beacon_timer);
    }
}
