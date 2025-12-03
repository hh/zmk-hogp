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

#include <zmk/hogp/hogp.h>

LOG_MODULE_REGISTER(hogp, CONFIG_ZMK_HOGP_LOG_LEVEL);

/* HID Service UUID: 0x1812 */
static struct bt_uuid_16 hid_service_uuid = BT_UUID_INIT_16(0x1812);

/* HID Report characteristic UUID: 0x2A4D */
static struct bt_uuid_16 hid_report_uuid = BT_UUID_INIT_16(0x2A4D);

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
    struct bt_conn *conn;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_subscribe_params report_subscribe_params;
    struct bt_gatt_discover_params sub_discover_params;  /* For CCC auto-discovery */
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t report_handle;
    uint16_t report_ccc_handle;
    enum hogp_discover_phase discover_phase;
    bt_addr_le_t addr;
    struct k_work subscribe_work;
    uint8_t security_retry_count;  /* Limit retries to prevent infinite loop */
};

#define HOGP_MAX_SECURITY_RETRIES 3

static struct hogp_device hogp_devices[CONFIG_ZMK_HOGP_MAX_DEVICES];

static bool hogp_pairing_mode = false;  /* Only scan/connect in pairing mode */
static bool is_scanning = false;

/* Callback for received HID reports */
static hogp_report_callback_t report_callback = NULL;

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

/* How long to wait after boot before auto-reconnect scan (ms) */
#define HOGP_AUTO_RECONNECT_DELAY_MS 5000
/* How long to scan for bonded devices during auto-reconnect (seconds) */
#define HOGP_AUTO_RECONNECT_SCAN_SEC 10

/* Forward declaration */
static void hogp_do_subscribe(struct hogp_device *dev);

/* Work handler to subscribe to reports (deferred from security callback) */
static void hogp_subscribe_work_handler(struct k_work *work) {
    struct hogp_device *dev = CONTAINER_OF(work, struct hogp_device, subscribe_work);
    if (dev->state != HOGP_STATE_SUBSCRIBING) {
        LOG_WRN("Device not in SUBSCRIBING state: %d", dev->state);
        return;
    }
    if (!dev->report_handle) {
        LOG_ERR("No report handle in subscribe work");
        return;
    }
    hogp_do_subscribe(dev);
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

    LOG_DBG("HID report received: %d bytes", length);
    LOG_HEXDUMP_DBG(data, length, "HID report");

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
 * Only processes devices when in pairing mode
 */
static void hogp_scan_recv(const struct bt_le_scan_recv_info *info,
                           struct net_buf_simple *buf) {
    /* Log that we're receiving scan results (every ~50th result to avoid spam) */
    static int scan_count = 0;
    scan_count++;
    if (scan_count % 50 == 1) {
        LOG_INF("HOGP scan callback active (count=%d, pairing=%d)", scan_count, hogp_pairing_mode);
    }

    /* Only process during pairing mode */
    if (!hogp_pairing_mode) {
        return;
    }

    /* Skip weak signals - device should be close for pairing */
    if (info->rssi < HOGP_MIN_RSSI_PAIRING) {
        return;
    }

    /* Check if we're already handling this device */
    if (hogp_is_known_device(info->addr)) {
        return;
    }

    struct hogp_parse_ctx ctx = {
        .found_hid = false,
        .is_apple_mfr = false,
        .name = {0},
    };

    /* Parse advertisement data */
    bt_data_parse(buf, hogp_ad_parse_cb, &ctx);

    /* Only log devices that are interesting: HID devices or named devices */
    char addr_str[BT_ADDR_LE_STR_LEN];
    bool is_public = hogp_is_public_addr(info->addr);
    bool should_connect = false;

    if (ctx.found_hid) {
        bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
        LOG_INF("*** HID DEVICE: %s '%s' (RSSI %d) ***", addr_str,
                ctx.name[0] ? ctx.name : "unnamed", info->rssi);
        should_connect = true;
    } else if (ctx.name[0]) {
        /* Only log named non-HID devices at debug level */
        bt_addr_le_to_str(info->addr, addr_str, sizeof(addr_str));
        LOG_DBG("Device: %s '%s' (RSSI %d, %s)", addr_str, ctx.name, info->rssi,
                is_public ? "pub" : "rnd");
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
 * Auto-reconnect work handler
 */
static void hogp_auto_reconnect_work_handler(struct k_work *work) {
    if (hogp_pairing_mode) {
        LOG_DBG("Auto-reconnect: already in pairing mode");
        return;
    }

    /* Check if we already have a connected device */
    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        if (hogp_devices[i].state >= HOGP_STATE_CONNECTED) {
            LOG_DBG("Auto-reconnect: already connected");
            return;
        }
    }

#if IS_ENABLED(CONFIG_ZMK_HOGP_AUTO_RECONNECT)
    LOG_INF("Auto-reconnect: scanning for bonded HOGP devices...");

    hogp_pairing_mode = true;
    int err = hogp_start_scan();
    if (err) {
        LOG_WRN("Auto-reconnect scan failed: %d", err);
        hogp_pairing_mode = false;
        k_work_schedule(&hogp_auto_reconnect_work, K_SECONDS(30));
        return;
    }

    k_work_schedule(&hogp_pairing_timeout_work, K_SECONDS(HOGP_AUTO_RECONNECT_SCAN_SEC));
#endif
}

/*
 * Actually perform the GATT subscription
 */
static void hogp_do_subscribe(struct hogp_device *dev) {
    memset(&dev->report_subscribe_params, 0, sizeof(dev->report_subscribe_params));
    dev->report_subscribe_params.notify = hogp_report_notify_cb;
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
        return BT_GATT_ITER_STOP;
    }

    if (!attr) {
        if (dev->discover_phase == HOGP_DISCOVER_SERVICE) {
            if (dev->service_start_handle == 0) {
                LOG_WRN("HID service not found - not an HID device");
                bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                return BT_GATT_ITER_STOP;
            }

            LOG_INF("HID service found (0x%04x-0x%04x), discovering characteristics...",
                    dev->service_start_handle, dev->service_end_handle);

            dev->discover_phase = HOGP_DISCOVER_CHARACTERISTICS;
            dev->discover_params.uuid = &hid_report_uuid.uuid;
            dev->discover_params.start_handle = dev->service_start_handle;
            dev->discover_params.end_handle = dev->service_end_handle;
            dev->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

            int err = bt_gatt_discover(dev->conn, &dev->discover_params);
            if (err) {
                LOG_ERR("Characteristic discovery failed (err %d)", err);
                dev->state = HOGP_STATE_CONNECTED;
            }
            return BT_GATT_ITER_STOP;

        } else if (dev->discover_phase == HOGP_DISCOVER_CHARACTERISTICS) {
            dev->discover_phase = HOGP_DISCOVER_COMPLETE;

            if (dev->report_handle) {
                LOG_INF("Discovery complete, subscribing to reports...");
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
        LOG_DBG("Found HID service: handles 0x%04x-0x%04x",
                dev->service_start_handle, dev->service_end_handle);
        return BT_GATT_ITER_CONTINUE;

    } else if (dev->discover_phase == HOGP_DISCOVER_CHARACTERISTICS) {
        struct bt_gatt_chrc *chrc = attr->user_data;
        LOG_INF("Found HID Report characteristic at handle 0x%04x (value: 0x%04x)",
                attr->handle, chrc->value_handle);

        dev->report_handle = chrc->value_handle;
        if (chrc->properties & BT_GATT_CHRC_NOTIFY) {
            LOG_INF("This report supports notifications - subscribing now");
            dev->discover_phase = HOGP_DISCOVER_COMPLETE;
            hogp_subscribe_to_reports(dev);
            return BT_GATT_ITER_STOP;
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
        LOG_INF("Security established (level %d), starting HID discovery...", level);
        hogp_start_discovery(dev);
    }
    else if (dev->state == HOGP_STATE_SUBSCRIBING && dev->report_handle && level >= BT_SECURITY_L2) {
        LOG_INF("Security established, subscribing to reports...");
        k_work_submit(&dev->subscribe_work);
    }
}

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

/*
 * Print HOGP status
 */
void hogp_print_status(void) {
    LOG_INF("=== HOGP Status ===");
    LOG_INF("Pairing mode: %s", hogp_pairing_mode ? "ON" : "OFF");
    LOG_INF("Scanning: %s", is_scanning ? "YES" : "NO");

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
 * Callback for bt_foreach_bond - unpair devices
 */
static void hogp_unpair_visitor(const struct bt_bond_info *info, void *user_data) {
    int *count = user_data;
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&info->addr, addr_str, sizeof(addr_str));

    LOG_INF("Found bond: %s", addr_str);

    int err = bt_unpair(BT_ID_DEFAULT, &info->addr);
    if (err) {
        LOG_WRN("Failed to unpair %s (err %d)", addr_str, err);
    } else {
        LOG_INF("Unpaired: %s", addr_str);
        (*count)++;
    }
}

/*
 * Clear HOGP bonds
 */
void hogp_clear_bonds(void) {
    LOG_INF("=== Clearing Bonds ===");

    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        struct hogp_device *dev = &hogp_devices[i];
        if (dev->conn) {
            LOG_INF("Disconnecting slot %d...", i);
            bt_conn_disconnect(dev->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
        memset(&dev->addr, 0, sizeof(dev->addr));
        dev->state = HOGP_STATE_IDLE;
    }

    k_msleep(200);

    int count = 0;
    bt_foreach_bond(BT_ID_DEFAULT, hogp_unpair_visitor, &count);

    LOG_INF("Cleared %d bonds", count);
    LOG_INF("NOTE: Split keyboard may need to reconnect");
    LOG_INF("=== Bond Clear Complete ===");
}

/*
 * Initialize HOGP central
 */
static int hogp_init(void) {
    LOG_INF("HOGP Central initializing...");

    for (int i = 0; i < CONFIG_ZMK_HOGP_MAX_DEVICES; i++) {
        hogp_devices[i].state = HOGP_STATE_IDLE;
        hogp_devices[i].conn = NULL;
        k_work_init(&hogp_devices[i].subscribe_work, hogp_subscribe_work_handler);
    }

    bt_le_scan_cb_register(&hogp_scan_cb);

#if IS_ENABLED(CONFIG_ZMK_HOGP_AUTO_RECONNECT)
    LOG_INF("HOGP: Will scan for bonded devices in %d ms", HOGP_AUTO_RECONNECT_DELAY_MS);
    k_work_schedule(&hogp_auto_reconnect_work, K_MSEC(HOGP_AUTO_RECONNECT_DELAY_MS));
#endif

    LOG_INF("HOGP Central initialized");
    return 0;
}

/* Initialize after BLE is ready */
SYS_INIT(hogp_init, APPLICATION, 91);
