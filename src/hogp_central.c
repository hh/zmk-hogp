/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * HOGP Central - BLE HID Central for external pointing devices
 *
 * This module allows the keyboard to connect to BLE HID devices (mice, trackpads)
 * and forward their input through the keyboard's active output.
 *
 * Key design decisions:
 * - HOGP has a dedicated "pairing mode" that must be triggered manually
 * - During pairing mode, split keyboard scanning is suspended
 * - Only attempts connections during pairing mode to avoid conflicts
 * - Reconnects to bonded devices automatically on boot
 */

#include <zephyr/types.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <zmk/hogp/hogp.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

LOG_MODULE_REGISTER(hogp, CONFIG_ZMK_HOGP_LOG_LEVEL);

/* HID Service UUID: 0x1812 */
static struct bt_uuid_16 hid_service_uuid = BT_UUID_INIT_16(0x1812);

/* HID Report characteristic UUID: 0x2A4D */
static struct bt_uuid_16 hid_report_uuid = BT_UUID_INIT_16(0x2A4D);

/* HID Protocol Mode characteristic UUID: 0x2A4E */
static struct bt_uuid_16 hid_protocol_mode_uuid = BT_UUID_INIT_16(0x2A4E);

/* HOGP device state */
enum hogp_device_state {
    HOGP_STATE_IDLE,
    HOGP_STATE_SCANNING,
    HOGP_STATE_CONNECTING,
    HOGP_STATE_CONNECTED,
    HOGP_STATE_DISCOVERING,
    HOGP_STATE_SUBSCRIBING,
    HOGP_STATE_READY,
};

/* Discovery phases */
enum hogp_discover_phase {
    HOGP_DISCOVER_SERVICE,
    HOGP_DISCOVER_CHARACTERISTICS,
    HOGP_DISCOVER_COMPLETE,
};

struct hogp_device {
    enum hogp_device_state state;
    enum hogp_device_type device_type;  /* Mouse or iTrack - set when first report received */
    struct bt_conn *conn;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_subscribe_params report_subscribe_params;
    struct bt_gatt_discover_params sub_discover_params;  /* For CCC auto-discovery */
    struct bt_gatt_read_params read_params;  /* For Protocol Mode read */
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t report_handle;
    uint16_t report_ccc_handle;
    uint16_t protocol_mode_handle;  /* HID Protocol Mode characteristic */
    enum hogp_discover_phase discover_phase;
    bt_addr_le_t addr;
    struct k_work subscribe_work;
    uint8_t security_retry_count;  /* Limit retries to prevent infinite loop */
    /* Discovery tracking for multi-report devices */
    uint8_t notify_count;           /* Number of notifiable reports found */
    uint16_t report_handle_2;       /* Handle of 2nd notifiable report (for M720) */
};

#define HOGP_MAX_SECURITY_RETRIES 3

static struct hogp_device hogp_devices[CONFIG_ZMK_HOGP_MAX_DEVICES];

static bool hogp_pairing_mode = false;  /* Only scan/connect in pairing mode */
static bool is_scanning = false;
static bool hogp_reconnect_scanning = false;  /* Scanning for known devices to reconnect */
static bool hogp_boot_scan = true;  /* First scan after boot uses longer duration */

/* Callback for received HID reports */
static hogp_report_callback_t report_callback = NULL;

/*
 * Known HOGP device addresses - persisted to NVS for reconnect after reboot
 * Declared early so reconnect logic can check count before scanning
 */
static bt_addr_le_t hogp_known_addrs[CONFIG_ZMK_HOGP_MAX_DEVICES];
static int hogp_known_addr_count = 0;

/* Minimum RSSI to consider connecting (-85 allows more distant devices for testing) */
#define HOGP_MIN_RSSI_PAIRING -85

/* Forward declarations */
static int hogp_start_scan(void);
static void hogp_stop_scan(void);
static void hogp_subscribe_to_reports(struct hogp_device *dev);
static struct hogp_device *hogp_device_for_conn(struct bt_conn *conn);

/* Pairing mode timeout work */
static void hogp_pairing_timeout_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(hogp_pairing_timeout_work, hogp_pairing_timeout_work_handler);

/* Auto-reconnect work - scans for bonded HOGP devices after boot */
static void hogp_auto_reconnect_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(hogp_auto_reconnect_work, hogp_auto_reconnect_work_handler);

/* Work to stop reconnect scan burst */
static void hogp_reconnect_scan_stop_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(hogp_reconnect_scan_stop_work, hogp_reconnect_scan_stop_handler);

/* How long to wait after boot before auto-reconnect scan (ms) */
#define HOGP_AUTO_RECONNECT_DELAY_MS 3000
/* How long to scan on boot (seconds) - longer to catch sleepy devices */
#define HOGP_BOOT_SCAN_SEC 15
/* Periodic reconnect check interval (seconds) - how often to scan for disconnected devices */
#define HOGP_PERIODIC_RECONNECT_SEC 5
/* How long each periodic scan burst lasts (seconds) */
#define HOGP_RECONNECT_SCAN_BURST_SEC 3

/* Forward declarations */
static void hogp_do_subscribe(struct hogp_device *dev);
static int hogp_reconnect_bonded_devices(void);
static struct hogp_device *hogp_get_free_slot(void);
static void hogp_remember_addr(const bt_addr_le_t *addr);
static bool hogp_is_remembered_addr(const bt_addr_le_t *addr);

/*
 * Protocol Mode read callback - called after reading HID Protocol Mode.
 * The laptop reads this before subscribing, and iTrack may use it to decide
 * whether to send 20-byte (4-finger) or 23-byte (5-finger) reports.
 */
static uint8_t hogp_protocol_mode_read_cb(struct bt_conn *conn, uint8_t err,
                                           struct bt_gatt_read_params *params,
                                           const void *data, uint16_t length) {
    struct hogp_device *dev = hogp_device_for_conn(conn);
    if (!dev) {
        LOG_WRN("protocol_mode_read_cb: no device for conn");
        return BT_GATT_ITER_STOP;
    }

    if (err) {
        LOG_WRN("Protocol Mode read failed (err %d), subscribing anyway...", err);
    } else if (data && length >= 1) {
        uint8_t protocol_mode = ((const uint8_t *)data)[0];
        LOG_INF("Protocol Mode = 0x%02x (%s)", protocol_mode,
                protocol_mode == 0x01 ? "Report Mode" :
                protocol_mode == 0x00 ? "Boot Mode" : "Unknown");
    } else {
        LOG_WRN("Protocol Mode read returned no data");
    }

    /* Now subscribe to reports */
    dev->state = HOGP_STATE_SUBSCRIBING;
    hogp_do_subscribe(dev);

    return BT_GATT_ITER_STOP;
}

/*
 * Read Protocol Mode characteristic before subscribing to reports.
 * This mimics what the laptop does - the iTrack may use this to trigger
 * full report mode (23-byte 5-finger reports instead of 20-byte 4-finger).
 */
static void hogp_read_protocol_mode(struct hogp_device *dev) {
    if (!dev->protocol_mode_handle) {
        LOG_INF("No Protocol Mode handle, subscribing directly...");
        dev->state = HOGP_STATE_SUBSCRIBING;
        hogp_do_subscribe(dev);
        return;
    }

    LOG_INF("Reading Protocol Mode at handle 0x%04x...", dev->protocol_mode_handle);

    memset(&dev->read_params, 0, sizeof(dev->read_params));
    dev->read_params.func = hogp_protocol_mode_read_cb;
    dev->read_params.handle_count = 1;
    dev->read_params.single.handle = dev->protocol_mode_handle;
    dev->read_params.single.offset = 0;

    int err = bt_gatt_read(dev->conn, &dev->read_params);
    if (err) {
        LOG_ERR("Protocol Mode read failed to start (err %d), subscribing anyway...", err);
        dev->state = HOGP_STATE_SUBSCRIBING;
        hogp_do_subscribe(dev);
    }
}

/* Work handler to subscribe to reports (deferred from discovery or security callback) */
static void hogp_subscribe_work_handler(struct k_work *work) {
    struct hogp_device *dev = CONTAINER_OF(work, struct hogp_device, subscribe_work);

    /*
     * Select which report to subscribe to:
     * - iTrack (2 notifiable): use last (trackpad is #2)
     * - M720 (3+ notifiable): use #2 (mouse input)
     */
    if (dev->notify_count > 2 && dev->report_handle_2) {
        dev->report_handle = dev->report_handle_2;
        LOG_INF("subscribe_work: %d notifiable, using #2 (M720-style) handle=0x%04x",
                dev->notify_count, dev->report_handle);
    } else {
        LOG_INF("subscribe_work: %d notifiable, using last (iTrack-style) handle=0x%04x",
                dev->notify_count, dev->report_handle);
    }

    if (!dev->report_handle) {
        LOG_ERR("No report handle in subscribe work");
        return;
    }

    /*
     * Read Protocol Mode before subscribing - this is what the laptop does.
     * The iTrack may use this to decide whether to send full 5-finger reports.
     */
    hogp_read_protocol_mode(dev);
}

/*
 * Register callback for HID reports
 */
void hogp_register_report_callback(hogp_report_callback_t cb) {
    report_callback = cb;
}

/*
 * HID Report notification callback - called when device sends data
 */
static uint8_t hogp_report_notify_cb(struct bt_conn *conn,
                                      struct bt_gatt_subscribe_params *params,
                                      const void *data, uint16_t length) {
    if (!data) {
        LOG_INF("HID report subscription ended");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    if (zmk_debug_enabled) {
        LOG_INF("HID report: %d bytes from handle 0x%04x", length, params->value_handle);
        LOG_HEXDUMP_INF(data, MIN(length, 24), "HID raw");
    }

    /* Detect and set device type based on report length */
    struct hogp_device *dev = hogp_device_for_conn(conn);
    if (dev && dev->device_type == HOGP_DEVICE_UNKNOWN) {
        /* iTrack sends 20 or 23 byte reports, mice send shorter reports */
        if (length == 20 || length == 23) {
            dev->device_type = HOGP_DEVICE_ITRACK;
            LOG_INF("HOGP: Detected iTrack (report len=%d)", length);
        } else {
            dev->device_type = HOGP_DEVICE_MOUSE;
            LOG_INF("HOGP: Detected Mouse (report len=%d)", length);
        }
    }

    /* Forward to registered callback if any */
    if (report_callback) {
        report_callback(data, length);
    }

    return BT_GATT_ITER_CONTINUE;
}

/*
 * Subscribe callback - called when subscription completes
 */
static void hogp_subscribe_cb(struct bt_conn *conn, uint8_t err,
                               struct bt_gatt_subscribe_params *params) {
    struct hogp_device *dev = hogp_device_for_conn(conn);
    if (!dev) {
        return;
    }

    if (err) {
        LOG_ERR("Subscribe failed (err %d)", err);
        dev->state = HOGP_STATE_CONNECTED;
        return;
    }

    LOG_INF("Subscribed to HID reports at handle 0x%04x", params->value_handle);
    dev->state = HOGP_STATE_READY;

    /* Exit pairing mode on successful subscription */
    hogp_exit_pairing_mode();
}

/*
 * Find an available device slot
 */
static struct hogp_device *hogp_get_free_slot(void) {
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].state == HOGP_STATE_IDLE) {
            return &hogp_devices[i];
        }
    }
    return NULL;
}

/*
 * Find device slot by connection
 */
static struct hogp_device *hogp_device_for_conn(struct bt_conn *conn) {
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].conn == conn) {
            return &hogp_devices[i];
        }
    }
    return NULL;
}

/*
 * Check if we're already connecting/connected to this address
 */
static bool hogp_is_known_device(const bt_addr_le_t *addr) {
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].state != HOGP_STATE_IDLE) {
            if (bt_addr_le_cmp(&hogp_devices[i].addr, addr) == 0) {
                return true;
            }
        }
    }
    return false;
}

/*
 * Check if this is a public address (not random)
 */
static bool hogp_is_public_addr(const bt_addr_le_t *addr) {
    return addr->type == BT_ADDR_LE_PUBLIC;
}

/*
 * Check if we found an HID service in advertisement data
 */
static bool hogp_ad_has_hid_service(struct bt_data *data) {
    if (data->type == BT_DATA_UUID16_SOME || data->type == BT_DATA_UUID16_ALL) {
        const uint8_t *d = data->data;
        for (int i = 0; i < data->data_len; i += 2) {
            uint16_t uuid = d[i] | (d[i + 1] << 8);
            if (uuid == 0x1812) {  /* HID Service */
                return true;
            }
        }
    }
    return false;
}

/*
 * Check for Apple manufacturer data (company ID 0x004C)
 */
static bool hogp_is_apple_manufacturer_data(struct bt_data *data) {
    if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len >= 2) {
        uint16_t company_id = data->data[0] | (data->data[1] << 8);
        return (company_id == 0x004C);
    }
    return false;
}

struct hogp_parse_ctx {
    bool found_hid;
    bool is_apple_mfr;  /* Has Apple manufacturer data (0x004C) */
    char name[32];      /* Device name from advertisement */
};

/*
 * Parse advertisement data callback
 */
static bool hogp_ad_parse_cb(struct bt_data *data, void *user_data) {
    struct hogp_parse_ctx *ctx = user_data;

    if (hogp_ad_has_hid_service(data)) {
        ctx->found_hid = true;
        LOG_INF("Found HID Service UUID in advertisement");
        return false;  /* Stop parsing */
    }

    if (hogp_is_apple_manufacturer_data(data)) {
        ctx->is_apple_mfr = true;
    }

    /* Extract device name (AD type 8 or 9) */
    if (data->type == BT_DATA_NAME_SHORTENED || data->type == BT_DATA_NAME_COMPLETE) {
        size_t len = MIN(data->data_len, sizeof(ctx->name) - 1);
        memcpy(ctx->name, data->data, len);
        ctx->name[len] = '\0';
    }

    return true;  /* Continue parsing */
}

/*
 * BLE scan callback - called for each discovered device
 * Handles both pairing mode and reconnect scanning
 */
static void hogp_scan_recv(const struct bt_le_scan_recv_info *info,
                           struct net_buf_simple *buf) {
    /* Log that we're receiving scan results (every ~50th result to avoid spam) */
    static int scan_count = 0;
    scan_count++;
    if (scan_count % 50 == 1) {
        LOG_DBG("HOGP scan (count=%d, pairing=%d, reconnect=%d)",
                scan_count, hogp_pairing_mode, hogp_reconnect_scanning);
    }

    /* Check if we're already handling this device */
    if (hogp_is_known_device(info->addr)) {
        return;
    }

    /* Reconnect scanning mode - look for known addresses */
    if (hogp_reconnect_scanning && !hogp_pairing_mode) {
        /* Resolve RPA to identity address using stored IRK (if available) */
        const bt_addr_le_t *id_addr = bt_lookup_id_addr(BT_ID_DEFAULT, info->addr);

        if (hogp_is_remembered_addr(id_addr)) {
            char addr_str[BT_ADDR_LE_STR_LEN];
            char id_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
            bt_addr_le_to_str(id_addr, id_str, sizeof(id_str));
            if (bt_addr_le_cmp(info->addr, id_addr) != 0) {
                LOG_INF("Resolved RPA %s -> identity %s", addr_str, id_str);
            }
            LOG_INF("Found known HOGP device: %s RSSI:%d - reconnecting", addr_str, info->rssi);

            /* Get a slot and connect */
            struct hogp_device *dev = hogp_get_free_slot();
            if (!dev) {
                LOG_WRN("No free slots for reconnect");
                return;
            }

            dev->state = HOGP_STATE_CONNECTING;
            bt_addr_le_copy(&dev->addr, info->addr);

            /* Must stop scan before bt_conn_le_create */
            int err = bt_le_scan_stop();
            if (err && err != -EALREADY) {
                LOG_WRN("Scan stop returned %d", err);
            }
            is_scanning = false;

            k_msleep(50);

            err = bt_conn_le_create(info->addr, BT_CONN_LE_CREATE_CONN,
                                    BT_LE_CONN_PARAM_DEFAULT, &dev->conn);
            if (err) {
                LOG_ERR("Reconnect failed (err %d)", err);
                dev->state = HOGP_STATE_IDLE;
            } else {
                LOG_INF("Reconnection initiated to %s", addr_str);
            }

            /* Check if we need to reconnect more devices */
            int connected_count = 0;
            for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
                if (hogp_devices[i].state != HOGP_STATE_IDLE) {
                    connected_count++;
                }
            }

            if (connected_count < hogp_known_addr_count && hogp_get_free_slot()) {
                /* More known devices to find - restart scan with fresh timeout */
                LOG_INF("Connected %d/%d known devices, continuing scan...",
                        connected_count, hogp_known_addr_count);
                k_msleep(100);
                err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, NULL);
                if (!err) {
                    is_scanning = true;
                    /* Reset the scan burst timeout to give more time for remaining devices */
                    k_work_cancel_delayable(&hogp_reconnect_scan_stop_work);
                    k_work_schedule(&hogp_reconnect_scan_stop_work,
                                    K_SECONDS(HOGP_RECONNECT_SCAN_BURST_SEC));
                } else {
                    LOG_WRN("Failed to restart scan (err %d)", err);
                    hogp_reconnect_scanning = false;
                    k_work_cancel_delayable(&hogp_reconnect_scan_stop_work);
                    k_work_schedule(&hogp_auto_reconnect_work,
                                    K_SECONDS(HOGP_PERIODIC_RECONNECT_SEC));
                }
            } else {
                /* All known devices connected or no free slots */
                LOG_INF("All %d known devices connected", connected_count);
                hogp_reconnect_scanning = false;
                k_work_cancel_delayable(&hogp_reconnect_scan_stop_work);
            }
        }
        /* During reconnect scanning, only look for known devices */
        return;
    }

    /* Only process new devices during pairing mode */
    if (!hogp_pairing_mode) {
        return;
    }

    /* Skip weak signals - device should be close for pairing */
    if (info->rssi < HOGP_MIN_RSSI_PAIRING) {
        return;
    }

    struct hogp_parse_ctx ctx = {
        .found_hid = false,
        .is_apple_mfr = false,
        .name = {0},
    };

    /* Parse advertisement data */
    bt_data_parse(buf, hogp_ad_parse_cb, &ctx);

    /* Log all named devices so we can see what's being discovered */
    char addr_str[BT_ADDR_LE_STR_LEN];
    bool should_connect = false;

    if (ctx.found_hid) {
        bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
        LOG_INF("*** HID: %s '%s' RSSI:%d ***", addr_str,
                ctx.name[0] ? ctx.name : "unnamed", info->rssi);
        should_connect = true;
    } else if (ctx.name[0]) {
        /* Log named devices so we can see if ProtoArc appears without HID flag */
        bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
        LOG_INF("Device: '%s' %s RSSI:%d", ctx.name, addr_str, info->rssi);
    }

    if (!should_connect) {
        return;
    }

    /* Check if we have a free slot */
    struct hogp_device *dev = hogp_get_free_slot();
    if (!dev) {
        LOG_WRN("No free HOGP slots");
        return;
    }

    LOG_INF("Connecting to: %s", addr_str);

    dev->state = HOGP_STATE_CONNECTING;
    bt_addr_le_copy(&dev->addr, info->addr);

    /* Try to stop any active scan first - bt_conn_le_create needs this */
    int err = bt_le_scan_stop();
    if (err && err != -EALREADY) {
        LOG_WRN("Scan stop returned %d (continuing anyway)", err);
    }

    /* Small delay to let scan stop settle */
    k_msleep(50);

    /* Create connection */
    err = bt_conn_le_create(info->addr, BT_CONN_LE_CREATE_CONN,
                            BT_LE_CONN_PARAM_DEFAULT, &dev->conn);
    if (err) {
        LOG_ERR("Failed to create connection (err %d)", err);
        dev->state = HOGP_STATE_IDLE;
        is_scanning = false;
    } else {
        LOG_INF("Connection initiated to %s", addr_str);
        hogp_pairing_mode = false;
        is_scanning = false;
    }
}

static struct bt_le_scan_cb hogp_scan_cb = {
    .recv = hogp_scan_recv,
};

/*
 * Stop BLE scanning
 */
static void hogp_stop_scan(void) {
    if (!is_scanning) {
        return;
    }

    int err = bt_le_scan_stop();
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to stop scan (err %d)", err);
    }
    is_scanning = false;
    LOG_DBG("HOGP scanning stopped");
}

/*
 * Start BLE scanning for HID devices
 */
static int hogp_start_scan(void) {
    if (is_scanning) {
        return 0;
    }

    if (!hogp_pairing_mode) {
        LOG_DBG("Not in pairing mode, not starting scan");
        return 0;
    }

    if (!hogp_get_free_slot()) {
        LOG_DBG("All HOGP slots in use");
        return 0;
    }

    /* Stop any existing scan first */
    int err = bt_le_scan_stop();
    if (err && err != -EALREADY) {
        LOG_WRN("Scan stop returned %d (continuing anyway)", err);
    }

    /* Small delay to let scan stop settle */
    k_msleep(50);

    /* Start scan with NULL callback - results go to registered callbacks */
    err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
    if (err) {
        LOG_ERR("Failed to start HOGP scan (err %d)", err);
        return err;
    }

    LOG_INF("HOGP scan started");
    is_scanning = true;
    return 0;
}

/*
 * Enter pairing mode - listens for HID devices for a limited time
 */
void hogp_enter_pairing_mode(void) {
    if (hogp_pairing_mode) {
        LOG_INF("Already in pairing mode");
        return;
    }

    LOG_INF("=== HOGP PAIRING MODE STARTED ===");
    LOG_INF("Listening for BLE HID devices for %d seconds...",
            CONFIG_ZMK_HOGP_SCAN_DURATION_SEC);
    LOG_INF("Put your BLE mouse/trackpad in pairing mode now!");

    hogp_pairing_mode = true;

    /* Start our own BLE scan */
    hogp_start_scan();

    /* Set timeout to exit pairing mode */
    k_work_schedule(&hogp_pairing_timeout_work,
                    K_SECONDS(CONFIG_ZMK_HOGP_SCAN_DURATION_SEC));
}

/*
 * Exit pairing mode
 */
void hogp_exit_pairing_mode(void) {
    if (!hogp_pairing_mode) {
        return;
    }

    LOG_INF("=== HOGP PAIRING MODE ENDED ===");

    hogp_pairing_mode = false;
    hogp_stop_scan();

    /* Cancel timeout if still pending */
    k_work_cancel_delayable(&hogp_pairing_timeout_work);
}

/*
 * Pairing mode timeout handler
 */
static void hogp_pairing_timeout_work_handler(struct k_work *work) {
    LOG_INF("Pairing mode timeout");
    hogp_exit_pairing_mode();
}

/*
 * Stop reconnect scan burst and schedule next one
 */
static void hogp_reconnect_scan_stop_handler(struct k_work *work) {
    if (!hogp_reconnect_scanning) {
        return;
    }

    LOG_DBG("Reconnect scan burst complete");

    if (is_scanning) {
        hogp_stop_scan();
    }

    hogp_reconnect_scanning = false;

    /* Schedule next reconnect check */
    k_work_schedule(&hogp_auto_reconnect_work, K_SECONDS(HOGP_PERIODIC_RECONNECT_SEC));
}

/*
 * Start a reconnect scan burst (passive scan for known devices)
 */
void hogp_start_reconnect_scan(void) {
    if (is_scanning || hogp_pairing_mode) {
        LOG_DBG("Cannot start reconnect scan: scanning=%d, pairing=%d",
                is_scanning, hogp_pairing_mode);
        return;
    }

    /* Use longer scan duration on boot to catch sleepy devices */
    int scan_duration = hogp_boot_scan ? HOGP_BOOT_SCAN_SEC : HOGP_RECONNECT_SCAN_BURST_SEC;
    LOG_INF("Starting reconnect scan (%d sec, boot=%d)", scan_duration, hogp_boot_scan);
    hogp_reconnect_scanning = true;

    /* Start passive scan - lower power, catches devices waking up */
    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, NULL);
    if (err) {
        LOG_WRN("Reconnect scan start failed (err %d)", err);
        hogp_reconnect_scanning = false;
        /* Retry later */
        k_work_schedule(&hogp_auto_reconnect_work, K_SECONDS(HOGP_PERIODIC_RECONNECT_SEC));
        return;
    }

    is_scanning = true;

    /* Schedule scan stop */
    k_work_schedule(&hogp_reconnect_scan_stop_work, K_SECONDS(scan_duration));

    /* After first scan, use shorter periodic duration */
    hogp_boot_scan = false;
}

/*
 * Auto-reconnect work handler
 */
static void hogp_auto_reconnect_work_handler(struct k_work *work) {
    bool need_reconnect = false;
    bool all_ready = true;

    if (hogp_pairing_mode) {
        LOG_DBG("Auto-reconnect: already in pairing mode, will retry later");
        k_work_schedule(&hogp_auto_reconnect_work, K_SECONDS(HOGP_PERIODIC_RECONNECT_SEC));
        return;
    }

    /* Check device states */
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].state == HOGP_STATE_READY) {
            /* Device is working, nothing to do */
            continue;
        } else if (hogp_devices[i].state >= HOGP_STATE_CONNECTED) {
            /* Connected but not ready - might be stuck */
            LOG_INF("Auto-reconnect: slot %d in state %d, not READY", i, hogp_devices[i].state);
            all_ready = false;
        } else if (hogp_devices[i].state != HOGP_STATE_IDLE) {
            /* In some transitional state */
            all_ready = false;
        } else {
            /* IDLE - check if we have a bond to reconnect to */
            all_ready = false;
            need_reconnect = true;
        }
    }

    /* If all slots are READY, just schedule next check and return */
    if (all_ready) {
        LOG_DBG("Auto-reconnect: all devices ready");
        k_work_schedule(&hogp_auto_reconnect_work, K_SECONDS(HOGP_PERIODIC_RECONNECT_SEC));
        return;
    }

#if IS_ENABLED(CONFIG_ZMK_HOGP_AUTO_RECONNECT)
    if (need_reconnect && hogp_known_addr_count > 0) {
        /* Start passive scan burst to catch device waking up */
        hogp_start_reconnect_scan();
        return;  /* Don't schedule another check - scan stop handler will do it */
    }
#endif

    /* Always schedule next periodic check */
    k_work_schedule(&hogp_auto_reconnect_work, K_SECONDS(HOGP_PERIODIC_RECONNECT_SEC));
}

/*
 * Actually perform the GATT subscription
 */
static void hogp_do_subscribe(struct hogp_device *dev) {
    memset(&dev->report_subscribe_params, 0, sizeof(dev->report_subscribe_params));
    dev->report_subscribe_params.notify = hogp_report_notify_cb;
    dev->report_subscribe_params.subscribe = hogp_subscribe_cb;  /* Called when subscription completes */
    dev->report_subscribe_params.value = BT_GATT_CCC_NOTIFY;
    dev->report_subscribe_params.value_handle = dev->report_handle;
    dev->report_subscribe_params.ccc_handle = 0;
    dev->report_subscribe_params.disc_params = &dev->sub_discover_params;
    dev->report_subscribe_params.end_handle = dev->service_end_handle;

    LOG_INF("Subscribing to reports at value_handle=0x%04x (CCC in 0x%04x-0x%04x)",
            dev->report_subscribe_params.value_handle,
            dev->report_handle, dev->service_end_handle);

    int err = bt_gatt_subscribe(dev->conn, &dev->report_subscribe_params);
    if (err && err != -EALREADY) {
        LOG_ERR("Subscribe failed (err %d)", err);
        dev->state = HOGP_STATE_CONNECTED;
    } else {
        LOG_INF("Subscription initiated successfully");
    }
}

/*
 * Subscribe to HID report notifications
 */
static void hogp_subscribe_to_reports(struct hogp_device *dev) {
    if (!dev->report_handle) {
        LOG_ERR("No report handle to subscribe to");
        return;
    }

    dev->state = HOGP_STATE_SUBSCRIBING;

    bt_security_t current = bt_conn_get_security(dev->conn);
    LOG_INF("Current security level: %d", current);

    if (current >= BT_SECURITY_L2) {
        LOG_INF("Already have security level %d, subscribing...", current);
        hogp_do_subscribe(dev);
        return;
    }

    LOG_INF("Requesting security level 2 (current: %d)...", current);
    int err = bt_conn_set_security(dev->conn, BT_SECURITY_L2);
    if (err == -EALREADY) {
        LOG_INF("Security already sufficient, subscribing...");
        hogp_do_subscribe(dev);
    } else if (err) {
        LOG_WRN("Failed to set security (err %d) - trying subscribe anyway", err);
        hogp_do_subscribe(dev);
    } else {
        LOG_INF("Security request pending, will subscribe when ready...");
    }
}

/*
 * GATT discovery callback
 */
static uint8_t hogp_discover_cb(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params) {
    struct hogp_device *dev = hogp_device_for_conn(conn);
    if (!dev) {
        LOG_WRN("discover_cb: no device for conn!");
        return BT_GATT_ITER_STOP;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&dev->addr, addr_str, sizeof(addr_str));
    LOG_INF("discover_cb: %s phase=%d attr=%s notify_cnt=%d",
            addr_str, dev->discover_phase, attr ? "valid" : "NULL", dev->notify_count);

    if (!attr) {
        if (dev->discover_phase == HOGP_DISCOVER_SERVICE) {
            /* Discovery ended without finding HID service - not an HID device */
            LOG_WRN("HID service not found - not an HID device");
            bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            return BT_GATT_ITER_STOP;

        } else if (dev->discover_phase == HOGP_DISCOVER_CHARACTERISTICS) {
            dev->discover_phase = HOGP_DISCOVER_COMPLETE;

            /*
             * Select which report to subscribe to:
             * - iTrack (2 notifiable): use last (trackpad is #2)
             * - M720 (4 notifiable): use #2 (mouse input)
             * Rule: if >2 notifiable reports, use #2; otherwise use last
             */
            if (dev->notify_count > 2 && dev->report_handle_2) {
                dev->report_handle = dev->report_handle_2;
                LOG_INF("Discovery: %d notifiable reports, using #2 (M720-style)", dev->notify_count);
            } else {
                LOG_INF("Discovery: %d notifiable reports, using last (iTrack-style)", dev->notify_count);
            }

            if (dev->report_handle) {
                LOG_INF("Subscribing to handle 0x%04x...", dev->report_handle);
                hogp_subscribe_to_reports(dev);
            } else {
                LOG_WRN("No HID report characteristic found");
                bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            }
            return BT_GATT_ITER_STOP;
        }

        return BT_GATT_ITER_STOP;
    }

    if (dev->discover_phase == HOGP_DISCOVER_SERVICE) {
        struct bt_gatt_service_val *service = attr->user_data;
        dev->service_start_handle = attr->handle;
        dev->service_end_handle = service->end_handle;
        LOG_INF("Found HID service: handles 0x%04x-0x%04x - stopping to discover characteristics",
                dev->service_start_handle, dev->service_end_handle);

        /* IMPORTANT: Return STOP here, not CONTINUE.
         * If we return CONTINUE, Zephyr will search for more HID services,
         * and there's a race condition with incoming ATT requests from the
         * peripheral that can cause the error response to be dropped.
         * Instead, we stop here and manually start characteristic discovery.
         */
        dev->discover_phase = HOGP_DISCOVER_CHARACTERISTICS;
        dev->protocol_mode_handle = 0;
        /* Discover ALL characteristics to find both Report and Protocol Mode */
        dev->discover_params.uuid = NULL;
        dev->discover_params.start_handle = dev->service_start_handle;
        dev->discover_params.end_handle = dev->service_end_handle;
        dev->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

        int err = bt_gatt_discover(dev->conn, &dev->discover_params);
        if (err) {
            LOG_ERR("Characteristic discovery failed to start (err %d)", err);
            dev->state = HOGP_STATE_CONNECTED;
        }
        return BT_GATT_ITER_STOP;

    } else if (dev->discover_phase == HOGP_DISCOVER_CHARACTERISTICS) {
        struct bt_gatt_chrc *chrc = attr->user_data;

        /* Check if this is Protocol Mode characteristic (0x2A4E) */
        if (bt_uuid_cmp(chrc->uuid, &hid_protocol_mode_uuid.uuid) == 0) {
            dev->protocol_mode_handle = chrc->value_handle;
            LOG_INF("Found Protocol Mode at handle 0x%04x", chrc->value_handle);
            return BT_GATT_ITER_CONTINUE;
        }

        /* Check if this is a Report characteristic (0x2A4D) */
        if (bt_uuid_cmp(chrc->uuid, &hid_report_uuid.uuid) != 0) {
            /* Not a Report, skip it */
            return BT_GATT_ITER_CONTINUE;
        }

        LOG_INF("Found HID Report at handle 0x%04x (value: 0x%04x) props=0x%02x",
                attr->handle, chrc->value_handle, chrc->properties);

        if (chrc->properties & BT_GATT_CHRC_NOTIFY) {
            dev->notify_count++;
            LOG_INF("  -> Notifiable #%d (handle 0x%04x)", dev->notify_count, chrc->value_handle);

            /* Save handle #2 for M720-style devices (4 notifiable reports) */
            if (dev->notify_count == 2) {
                dev->report_handle_2 = chrc->value_handle;
            }
            /* Always save last as fallback (works for iTrack with 2 reports) */
            dev->report_handle = chrc->value_handle;

            /*
             * Subscribe immediately when we have enough info:
             * - After 2 notifiable reports (covers iTrack)
             * - Or after 3+ (use #2 for M720-style)
             * Zephyr characteristic discovery doesn't call back with NULL,
             * so we must decide when to stop ourselves.
             */
            if (dev->notify_count >= 2) {
                /* Use a delayed work to subscribe after discovery settles */
                k_work_submit(&dev->subscribe_work);
                dev->discover_phase = HOGP_DISCOVER_COMPLETE;
                return BT_GATT_ITER_STOP;
            }
        }
        return BT_GATT_ITER_CONTINUE;
    }

    return BT_GATT_ITER_CONTINUE;
}

/*
 * Start GATT service discovery
 */
static void hogp_start_discovery(struct hogp_device *dev) {
    dev->state = HOGP_STATE_DISCOVERING;
    dev->discover_phase = HOGP_DISCOVER_SERVICE;
    dev->service_start_handle = 0;
    dev->service_end_handle = 0;
    dev->report_handle = 0;
    dev->notify_count = 0;
    dev->report_handle_2 = 0;

    dev->discover_params.uuid = &hid_service_uuid.uuid;
    dev->discover_params.func = hogp_discover_cb;
    dev->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    dev->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    dev->discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(dev->conn, &dev->discover_params);
    if (err) {
        LOG_ERR("GATT discovery failed (err %d)", err);
        dev->state = HOGP_STATE_CONNECTED;
    }
}

/*
 * Connection callback
 */
static void hogp_connected(struct bt_conn *conn, uint8_t err) {
    struct hogp_device *dev = hogp_device_for_conn(conn);

    if (!dev) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    if (err) {
        LOG_ERR("Failed to connect to %s (err %d)", addr_str, err);
        bt_conn_unref(dev->conn);
        dev->conn = NULL;
        dev->state = HOGP_STATE_IDLE;
        if (hogp_pairing_mode) {
            hogp_start_scan();
        }
        return;
    }

    LOG_INF("Connected to HOGP device: %s", addr_str);
    dev->state = HOGP_STATE_CONNECTED;
    dev->security_retry_count = 0;

    LOG_INF("Requesting security level 2 for HID access...");
    int sec_err = bt_conn_set_security(dev->conn, BT_SECURITY_L2);
    if (sec_err == -EALREADY) {
        LOG_INF("Security already sufficient, starting discovery...");
        k_msleep(100);
        hogp_start_discovery(dev);
    } else if (sec_err) {
        LOG_WRN("Failed to request security (err %d), trying discovery anyway...", sec_err);
        k_msleep(100);
        hogp_start_discovery(dev);
    } else {
        LOG_INF("Security request pending, will discover after pairing...");
    }
}

/*
 * Disconnection callback
 */
static void hogp_disconnected(struct bt_conn *conn, uint8_t reason) {
    struct hogp_device *dev = hogp_device_for_conn(conn);

    if (!dev) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    LOG_INF("HOGP device disconnected: %s (reason 0x%02x)", addr_str, reason);

    bt_conn_unref(dev->conn);
    dev->conn = NULL;
    dev->state = HOGP_STATE_IDLE;
    dev->report_handle = 0;

    /* Clear mouse state to release any stuck buttons */
    zmk_hid_mouse_clear();
    zmk_endpoints_send_mouse_report();
    LOG_INF("HOGP: Cleared mouse state on disconnect");

    LOG_INF("HOGP: Will attempt reconnect in 3 seconds...");
    k_work_schedule(&hogp_auto_reconnect_work, K_SECONDS(3));
}

/*
 * Security changed callback
 */
static void hogp_security_changed(struct bt_conn *conn, bt_security_t level,
                                   enum bt_security_err err) {
    struct hogp_device *dev = hogp_device_for_conn(conn);
    if (!dev) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    if (err) {
        dev->security_retry_count++;
        LOG_ERR("Security failed for %s: level %d err %d (retry %d/%d)",
                addr_str, level, err, dev->security_retry_count, HOGP_MAX_SECURITY_RETRIES);

        if (dev->security_retry_count >= HOGP_MAX_SECURITY_RETRIES) {
            LOG_ERR("Max security retries reached, disconnecting device");
            bt_conn_disconnect(dev->conn, BT_HCI_ERR_AUTH_FAIL);
            return;
        }

        if (dev->state == HOGP_STATE_CONNECTED) {
            LOG_INF("Security failed, trying discovery anyway...");
            hogp_start_discovery(dev);
        } else if (dev->state == HOGP_STATE_SUBSCRIBING && dev->report_handle) {
            LOG_INF("Security failed, trying subscribe without encryption...");
            hogp_do_subscribe(dev);
        }
        return;
    }

    LOG_INF("Security changed for %s: level %d", addr_str, level);

    if (dev->state == HOGP_STATE_CONNECTED && level >= BT_SECURITY_L2) {
        /* Check if this is a reconnection to an already-bonded device.
         * If so, pairing_complete will NOT be called - start discovery now.
         * For new pairing, pairing_complete will be called and we start there.
         * We detect reconnection by checking if we already know this device. */
        if (hogp_is_remembered_addr(&dev->addr)) {
            /* Reconnection to known device - start discovery immediately */
            LOG_INF("Reconnected to known bonded device, starting HID discovery...");
            hogp_start_discovery(dev);
        } else {
            /* New pairing in progress - wait for pairing_complete callback.
             * This avoids the Zephyr SMP bug where concurrent SMP and GATT ops fail. */
            LOG_INF("Security level %d achieved, waiting for pairing_complete...", level);
        }
    }
    else if (dev->state == HOGP_STATE_SUBSCRIBING && dev->report_handle && level >= BT_SECURITY_L2) {
        LOG_INF("Security established, subscribing to reports...");
        k_work_submit(&dev->subscribe_work);
    }
}

/*
 * Pairing complete callback - SMP is truly finished, safe to start GATT discovery
 */
static void hogp_pairing_complete(struct bt_conn *conn, bool bonded) {
    struct hogp_device *dev = hogp_device_for_conn(conn);
    if (!dev) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&dev->addr, addr_str, sizeof(addr_str));
    LOG_INF("Pairing complete for %s (bonded=%d)", addr_str, bonded);

    /* Remember the IDENTITY address for auto-reconnect (not RPA) */
    if (bonded) {
        /* After pairing, bt_conn_get_dst returns the identity address if IRK was exchanged */
        const bt_addr_le_t *conn_addr = bt_conn_get_dst(conn);
        /* Also try to resolve via IRK lookup in case conn_addr is still RPA */
        const bt_addr_le_t *id_addr = bt_lookup_id_addr(BT_ID_DEFAULT, conn_addr);

        char id_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(id_addr, id_str, sizeof(id_str));

        if (bt_addr_le_cmp(conn_addr, id_addr) != 0) {
            LOG_INF("Device has identity address: %s (was %s)", id_str, addr_str);
        }

        hogp_remember_addr(id_addr);
        LOG_INF("Remembered HOGP device for auto-reconnect: %s", id_str);
    }

    if (dev->state == HOGP_STATE_CONNECTED) {
        LOG_INF("SMP finished, starting HID discovery...");
        hogp_start_discovery(dev);
    }
}

static void hogp_pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
    struct hogp_device *dev = hogp_device_for_conn(conn);
    if (!dev) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&dev->addr, addr_str, sizeof(addr_str));
    LOG_WRN("Pairing failed for %s (reason %d), trying discovery anyway...", addr_str, reason);

    if (dev->state == HOGP_STATE_CONNECTED) {
        hogp_start_discovery(dev);
    }
}

static struct bt_conn_auth_info_cb hogp_auth_info_cb = {
    .pairing_complete = hogp_pairing_complete,
    .pairing_failed = hogp_pairing_failed,
};

BT_CONN_CB_DEFINE(hogp_conn_callbacks) = {
    .connected = hogp_connected,
    .disconnected = hogp_disconnected,
    .security_changed = hogp_security_changed,
};

/*
 * Get state name for logging
 */
static const char *hogp_state_name(enum hogp_device_state state) {
    switch (state) {
        case HOGP_STATE_IDLE: return "IDLE";
        case HOGP_STATE_SCANNING: return "SCANNING";
        case HOGP_STATE_CONNECTING: return "CONNECTING";
        case HOGP_STATE_CONNECTED: return "CONNECTED";
        case HOGP_STATE_DISCOVERING: return "DISCOVERING";
        case HOGP_STATE_SUBSCRIBING: return "SUBSCRIBING";
        case HOGP_STATE_READY: return "READY";
        default: return "UNKNOWN";
    }
}

/* hogp_known_addrs and hogp_known_addr_count declared earlier in file */
static int hogp_settings_register_err = -999;  /* Track registration result */
#if IS_ENABLED(CONFIG_SETTINGS)
static bool hogp_settings_loaded = false;
#endif

/*
 * Print HOGP status
 */
void hogp_print_status(void) {
    LOG_INF("=== HOGP Status ===");
    LOG_INF("Pairing mode: %s", hogp_pairing_mode ? "ON" : "OFF");
    LOG_INF("Scanning: %s", is_scanning ? "YES" : "NO");
    LOG_INF("Reconnect scan: %s", hogp_reconnect_scanning ? "YES" : "NO");
    LOG_INF("Known devices: %d", hogp_known_addr_count);

    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        struct hogp_device *dev = &hogp_devices[i];
        if (dev->state != HOGP_STATE_IDLE) {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(&dev->addr, addr_str, sizeof(addr_str));
            LOG_INF("Slot %d: %s state=%s handle=0x%04x",
                    i, addr_str, hogp_state_name(dev->state), dev->report_handle);
        } else {
            LOG_INF("Slot %d: (empty)", i);
        }
    }
    LOG_INF("===================");
}

/*
 * Dump NVS settings state for debugging
 */
void hogp_dump_nvs_state(void) {
    LOG_INF("=== HOGP NVS State ===");
    LOG_INF("settings_register err: %d", hogp_settings_register_err);
#if IS_ENABLED(CONFIG_SETTINGS)
    LOG_INF("settings_loaded: %s", hogp_settings_loaded ? "YES" : "NO");
#else
    LOG_INF("settings_loaded: N/A (CONFIG_SETTINGS disabled)");
#endif
    LOG_INF("known_addr_count: %d", hogp_known_addr_count);
    for (int i = 0; i < hogp_known_addr_count; i++) {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&hogp_known_addrs[i], addr_str, sizeof(addr_str));
        LOG_INF("  addr[%d]: %s", i, addr_str);
    }
    LOG_INF("======================");
}

/*
 * Get current HOGP indicator state for LED display
 */
enum hogp_indicator_state hogp_get_indicator_state(void) {
    /* Check if any device is connected (any state >= CONNECTED means we have an active connection) */
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].state >= HOGP_STATE_CONNECTED) {
            /* CONNECTED, DISCOVERING, SUBSCRIBING, or READY - all mean we're connected */
            return HOGP_INDICATOR_CONNECTED;
        }
    }

    /* Check if in pairing mode (user-initiated scan) */
    if (hogp_pairing_mode) {
        return HOGP_INDICATOR_PAIRING;
    }

    /* Check if reconnect scanning for known devices */
    if (hogp_reconnect_scanning) {
        return HOGP_INDICATOR_SCANNING;
    }

    /* Check if actively trying to connect */
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].state == HOGP_STATE_CONNECTING) {
            return HOGP_INDICATOR_SCANNING;
        }
    }

    /* If we have known devices but not connected, we're waiting to reconnect */
    if (hogp_known_addr_count > 0) {
        return HOGP_INDICATOR_SCANNING;
    }

    /* No known devices - idle state */
    return HOGP_INDICATOR_IDLE;
}

/*
 * Get indicator state for a specific device type (mouse or iTrack)
 */
enum hogp_indicator_state hogp_get_device_indicator_state(enum hogp_device_type type) {
    /* Check if a device of this type is connected */
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].device_type == type &&
            hogp_devices[i].state >= HOGP_STATE_CONNECTED) {
            return HOGP_INDICATOR_CONNECTED;
        }
    }

    /* Check if in pairing/scanning mode */
    if (hogp_pairing_mode) {
        return HOGP_INDICATOR_PAIRING;
    }

    if (hogp_reconnect_scanning) {
        return HOGP_INDICATOR_SCANNING;
    }

    /* Not connected */
    return HOGP_INDICATOR_IDLE;
}

/*
 * Set the device type for a slot (called when first report received)
 */
void hogp_set_device_type(struct bt_conn *conn, enum hogp_device_type type) {
    struct hogp_device *dev = hogp_device_for_conn(conn);
    if (dev) {
        dev->device_type = type;
        LOG_INF("HOGP: Device type set to %s", type == HOGP_DEVICE_MOUSE ? "MOUSE" : "ITRACK");
    }
}

/*
 * Check if address is a known HOGP device (in our slots or matches a slot's addr)
 */
static bool hogp_is_hogp_device_addr(const bt_addr_le_t *addr) {
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].state != HOGP_STATE_IDLE) {
            if (bt_addr_le_cmp(&hogp_devices[i].addr, addr) == 0) {
                return true;
            }
        }
    }
    return false;
}

/*
 * Save HOGP known addresses to NVS
 */
static void hogp_save_known_addrs(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    int err;
    for (int i = 0; i < hogp_known_addr_count; i++) {
        char setting_name[24];
        snprintf(setting_name, sizeof(setting_name), "hogp/addr/%d", i);
        err = settings_save_one(setting_name, &hogp_known_addrs[i], sizeof(bt_addr_le_t));
        if (err) {
            LOG_ERR("Failed to save %s: %d", setting_name, err);
        }
    }
    /* Save the count */
    err = settings_save_one("hogp/count", &hogp_known_addr_count, sizeof(hogp_known_addr_count));
    if (err) {
        LOG_ERR("Failed to save hogp/count: %d", err);
    }
    LOG_INF("Saved %d HOGP device addresses to NVS", hogp_known_addr_count);
#endif
}

/*
 * Check if an address is currently connected
 */
static bool hogp_is_addr_connected(const bt_addr_le_t *addr) {
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].state != HOGP_STATE_IDLE) {
            if (bt_addr_le_cmp(&hogp_devices[i].addr, addr) == 0) {
                return true;
            }
        }
    }
    return false;
}

/*
 * Remember an address as belonging to an HOGP device
 */
static void hogp_remember_addr(const bt_addr_le_t *addr) {
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    /* Check if already known */
    for (int i = 0; i < hogp_known_addr_count; i++) {
        if (bt_addr_le_cmp(&hogp_known_addrs[i], addr) == 0) {
            return;  /* Already tracked */
        }
    }

    /* Add if space available */
    if (hogp_known_addr_count < CONFIG_ZMK_HOGP_MAX_DEVICES) {
        bt_addr_le_copy(&hogp_known_addrs[hogp_known_addr_count++], addr);
        LOG_INF("Remembering HOGP device: %s", addr_str);
        hogp_save_known_addrs();
        return;
    }

    /* No space - try to replace a stale (disconnected) address */
    for (int i = 0; i < hogp_known_addr_count; i++) {
        if (!hogp_is_addr_connected(&hogp_known_addrs[i])) {
            char old_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(&hogp_known_addrs[i], old_str, sizeof(old_str));
            LOG_INF("Replacing stale address %s with %s", old_str, addr_str);
            bt_addr_le_copy(&hogp_known_addrs[i], addr);
            hogp_save_known_addrs();
            return;
        }
    }

    LOG_WRN("Cannot remember %s - all slots in use by connected devices", addr_str);
}

/*
 * Check if address is a remembered HOGP device
 */
static bool hogp_is_remembered_addr(const bt_addr_le_t *addr) {
    for (int i = 0; i < hogp_known_addr_count; i++) {
        if (bt_addr_le_cmp(&hogp_known_addrs[i], addr) == 0) {
            return true;
        }
    }
    return false;
}

/*
 * Try to directly connect to a remembered HOGP device address
 */
static int hogp_try_direct_connect(const bt_addr_le_t *addr) {
    struct hogp_device *dev = hogp_get_free_slot();
    if (!dev) {
        LOG_WRN("No free HOGP slots for direct connect");
        return -ENOMEM;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("Direct connect to bonded HOGP device: %s", addr_str);

    dev->state = HOGP_STATE_CONNECTING;
    bt_addr_le_copy(&dev->addr, addr);

    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                                BT_LE_CONN_PARAM_DEFAULT, &dev->conn);
    if (err) {
        LOG_ERR("Direct connect failed (err %d)", err);
        dev->state = HOGP_STATE_IDLE;
        return err;
    }

    LOG_INF("Direct connection initiated to %s", addr_str);
    return 0;
}

/*
 * Callback for bt_foreach_bond to find and connect to HOGP devices
 */
struct hogp_reconnect_ctx {
    int attempted;
    int connected;
};

static void hogp_bond_reconnect_visitor(const struct bt_bond_info *info, void *user_data) {
    struct hogp_reconnect_ctx *ctx = user_data;

    /* Only try to connect to remembered HOGP devices */
    if (!hogp_is_remembered_addr(&info->addr)) {
        return;
    }

    /* Check if already connected to this device */
    if (hogp_is_hogp_device_addr(&info->addr)) {
        LOG_DBG("Already connected to this HOGP device");
        return;
    }

    /* Check if we have a free slot */
    struct hogp_device *free_slot = hogp_get_free_slot();
    if (!free_slot) {
        LOG_DBG("No free slots for reconnect");
        return;
    }

    ctx->attempted++;
    if (hogp_try_direct_connect(&info->addr) == 0) {
        ctx->connected++;
    }
}

/*
 * Try to reconnect to all bonded HOGP devices
 */
static int hogp_reconnect_bonded_devices(void) {
    struct hogp_reconnect_ctx ctx = { .attempted = 0, .connected = 0 };

    bt_foreach_bond(BT_ID_DEFAULT, hogp_bond_reconnect_visitor, &ctx);

    LOG_INF("Reconnect: attempted %d, initiated %d", ctx.attempted, ctx.connected);
    return ctx.connected;
}

/*
 * Context for selective bond clearing
 */
struct hogp_unpair_ctx {
    int count;
    bool hogp_only;  /* true = only HOGP devices, false = only host devices */
};

/*
 * Callback for bt_foreach_bond - selectively unpair devices
 */
static void hogp_unpair_visitor(const struct bt_bond_info *info, void *user_data) {
    struct hogp_unpair_ctx *ctx = user_data;
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&info->addr, addr_str, sizeof(addr_str));

    bool is_hogp = hogp_is_hogp_device_addr(&info->addr) ||
                   hogp_is_remembered_addr(&info->addr);

    /* Check if we should unpair this device */
    if (ctx->hogp_only && !is_hogp) {
        LOG_INF("Skipping host bond: %s", addr_str);
        return;
    }
    if (!ctx->hogp_only && is_hogp) {
        LOG_INF("Skipping HOGP bond: %s", addr_str);
        return;
    }

    LOG_INF("Unpairing %s: %s", ctx->hogp_only ? "HOGP" : "host", addr_str);

    int err = bt_unpair(BT_ID_DEFAULT, &info->addr);
    if (err) {
        LOG_WRN("Failed to unpair %s (err %d)", addr_str, err);
    } else {
        LOG_INF("Unpaired: %s", addr_str);
        ctx->count++;
    }
}

/*
 * Clear only HOGP device bonds (not host/laptop bonds)
 */
void hogp_clear_bonds(void) {
    LOG_INF("=== Clearing HOGP Bonds Only ===");

    /* First, remember current device addresses before disconnecting */
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        struct hogp_device *dev = &hogp_devices[i];
        if (dev->state != HOGP_STATE_IDLE) {
            hogp_remember_addr(&dev->addr);
        }
    }

    /* Disconnect HOGP devices */
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        struct hogp_device *dev = &hogp_devices[i];
        if (dev->conn) {
            LOG_INF("Disconnecting HOGP slot %d...", i);
            bt_conn_disconnect(dev->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
        memset(&dev->addr, 0, sizeof(dev->addr));
        dev->state = HOGP_STATE_IDLE;
    }

    k_msleep(200);

    /* Only unpair HOGP devices */
    struct hogp_unpair_ctx ctx = { .count = 0, .hogp_only = true };
    bt_foreach_bond(BT_ID_DEFAULT, hogp_unpair_visitor, &ctx);

    /* Clear remembered addresses from RAM and NVS */
    hogp_known_addr_count = 0;
    memset(hogp_known_addrs, 0, sizeof(hogp_known_addrs));
#if IS_ENABLED(CONFIG_SETTINGS)
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        char setting_name[24];
        snprintf(setting_name, sizeof(setting_name), "hogp/addr/%d", i);
        settings_delete(setting_name);
    }
    settings_save_one("hogp/count", &hogp_known_addr_count, sizeof(hogp_known_addr_count));
#endif

    LOG_INF("Cleared %d HOGP bonds (host bonds preserved)", ctx.count);
    LOG_INF("=== HOGP Bond Clear Complete ===");
}

/*
 * Clear only NVS settings - no BLE operations, safe from serial thread
 */
void hogp_clear_nvs_only(void) {
    LOG_INF("=== Clearing HOGP NVS Only ===");

    /* Clear RAM state */
    hogp_known_addr_count = 0;
    memset(hogp_known_addrs, 0, sizeof(hogp_known_addrs));

#if IS_ENABLED(CONFIG_SETTINGS)
    /* Clear NVS */
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        char setting_name[24];
        snprintf(setting_name, sizeof(setting_name), "hogp/addr/%d", i);
        settings_delete(setting_name);
    }
    settings_save_one("hogp/count", &hogp_known_addr_count, sizeof(hogp_known_addr_count));
#endif

    LOG_INF("NVS cleared. known_addr_count=%d", hogp_known_addr_count);
    LOG_INF("=== HOGP NVS Clear Complete ===");
}

/*
 * Clear only host device bonds (laptops, not HOGP mice/trackpads)
 */
void hogp_clear_host_bonds(void) {
    LOG_INF("=== Clearing Host Bonds Only ===");

    /* Remember current HOGP addresses so we don't unpair them */
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        struct hogp_device *dev = &hogp_devices[i];
        if (dev->state != HOGP_STATE_IDLE) {
            hogp_remember_addr(&dev->addr);
        }
    }

    k_msleep(100);

    /* Only unpair non-HOGP devices (hosts) */
    struct hogp_unpair_ctx ctx = { .count = 0, .hogp_only = false };
    bt_foreach_bond(BT_ID_DEFAULT, hogp_unpair_visitor, &ctx);

    LOG_INF("Cleared %d host bonds (HOGP bonds preserved)", ctx.count);
    LOG_INF("=== Host Bond Clear Complete ===");
}

/*============================================================================
 * Settings persistence - load/save HOGP device addresses to NVS
 *============================================================================*/

#if IS_ENABLED(CONFIG_SETTINGS)

/* Track which address slots were actually loaded from NVS */
static bool hogp_addr_loaded[CONFIG_ZMK_HOGP_MAX_DEVICES];

static int hogp_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;

    printk("[HOGP] settings_set: name='%s' len=%d\n", name, (int)len);

    if (settings_name_steq(name, "count", &next) && !next) {
        /* We ignore the saved count - will recalculate in commit based on
         * actually loaded addresses. This fixes the bug where count=2 but
         * only 1 address was actually saved/loaded. */
        int saved_count;
        if (len != sizeof(saved_count)) {
            return -EINVAL;
        }
        int err = read_cb(cb_arg, &saved_count, sizeof(saved_count));
        if (err < 0) {
            LOG_ERR("Failed to read HOGP addr count: %d", err);
            return err;
        }
        LOG_INF("NVS says HOGP device count: %d (will verify)", saved_count);
    } else if (settings_name_steq(name, "addr", &next) && next) {
        int idx = atoi(next);
        if (idx < 0 || idx >= CONFIG_ZMK_HOGP_MAX_DEVICES) {
            return -EINVAL;
        }
        if (len != sizeof(bt_addr_le_t)) {
            return -EINVAL;
        }
        int err = read_cb(cb_arg, &hogp_known_addrs[idx], sizeof(bt_addr_le_t));
        if (err < 0) {
            LOG_ERR("Failed to read HOGP addr %d: %d", idx, err);
            return err;
        }
        hogp_addr_loaded[idx] = true;  /* Mark this slot as loaded */
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&hogp_known_addrs[idx], addr_str, sizeof(addr_str));
        LOG_INF("Loaded HOGP device %d: %s", idx, addr_str);
    }
    return 0;
}

static int hogp_settings_commit(void) {
    hogp_settings_loaded = true;

    /* Compact loaded addresses and recalculate count.
     * This ensures hogp_known_addr_count matches reality. */
    int actual_count = 0;
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_addr_loaded[i]) {
            if (actual_count != i) {
                /* Move to fill gap */
                bt_addr_le_copy(&hogp_known_addrs[actual_count], &hogp_known_addrs[i]);
                memset(&hogp_known_addrs[i], 0, sizeof(bt_addr_le_t));
            }
            actual_count++;
        }
    }
    hogp_known_addr_count = actual_count;

    printk("[HOGP] settings_commit: verified count=%d\n", hogp_known_addr_count);
    return 0;
}

static struct settings_handler hogp_settings_handler = {
    .name = "hogp",
    .h_set = hogp_settings_set,
    .h_commit = hogp_settings_commit,
};

#endif /* CONFIG_SETTINGS */

/* Settings init moved to hogp_init to ensure it runs after settings_subsys_init */

/*
 * Initialize HOGP central - BT callbacks and auto-reconnect
 */
static int hogp_init(void) {
    LOG_INF("HOGP Central initializing...");

    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        hogp_devices[i].state = HOGP_STATE_IDLE;
        hogp_devices[i].conn = NULL;
        k_work_init(&hogp_devices[i].subscribe_work, hogp_subscribe_work_handler);
    }

    bt_le_scan_cb_register(&hogp_scan_cb);
    bt_conn_auth_info_cb_register(&hogp_auth_info_cb);

#if IS_ENABLED(CONFIG_SETTINGS)
    /* Register settings handler and load our subtree.
     * We do this in hogp_init (after main() has called settings_subsys_init)
     * rather than in early SYS_INIT to ensure settings subsystem is ready. */
    hogp_settings_register_err = settings_register(&hogp_settings_handler);
    LOG_INF("HOGP: settings_register returned %d", hogp_settings_register_err);

    /* Load our settings subtree - this calls our h_set handler */
    int load_err = settings_load_subtree("hogp");
    LOG_INF("HOGP: settings_load_subtree returned %d", load_err);
#endif

    /* Debug: dump loaded settings state */
    LOG_INF("HOGP settings state: loaded=%d, count=%d", hogp_settings_loaded, hogp_known_addr_count);
    for (int i = 0; i < hogp_known_addr_count; i++) {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&hogp_known_addrs[i], addr_str, sizeof(addr_str));
        LOG_INF("  Known device %d: %s", i, addr_str);
    }

#if IS_ENABLED(CONFIG_ZMK_HOGP_AUTO_RECONNECT)
    LOG_INF("HOGP: Will scan for bonded devices in %d ms", HOGP_AUTO_RECONNECT_DELAY_MS);
    k_work_schedule(&hogp_auto_reconnect_work, K_MSEC(HOGP_AUTO_RECONNECT_DELAY_MS));
#endif

    LOG_INF("HOGP Central initialized");
    return 0;
}

/* Initialize after BLE is ready */
SYS_INIT(hogp_init, APPLICATION, 91);
