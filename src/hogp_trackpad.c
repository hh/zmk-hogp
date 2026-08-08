/*
 * Copyright (c) 2024 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * HOGP Trackpad Output - Converts trackpad reports to ADG multitouch
 *
 * This module receives HID reports from HOGP-connected trackpads
 * and converts them to Apple ADG Chapter 15 format for full
 * multitouch gesture support on iPad, macOS, and Linux.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <zmk/hogp/hogp.h>
#include <zmk/hid.h>
#include <zmk/hog.h>
#include <zmk/endpoints.h>

/* External iTrack handler for 23-byte reports */
extern void hogp_itrack_process(const uint8_t *data, uint16_t len);

LOG_MODULE_REGISTER(hogp_trackpad, CONFIG_ZMK_HOGP_LOG_LEVEL);

/*
 * ProtoArc report format (19 bytes, no Report ID prefix via BLE):
 *   Bytes 0-3:   Finger 0
 *   Bytes 4-7:   Finger 1
 *   Bytes 8-11:  Finger 2
 *   Bytes 12-15: Finger 3
 *   Bytes 16-17: Scan Time
 *   Byte 18:     Contact Count (7 bits) | Button (bit 7)
 *
 * Finger format (same as ADG!):
 *   Byte 0: Flags (Confidence, Tip Switch, Contact ID)
 *   Byte 1: X[7:0]
 *   Byte 2: X[11:8] | Y[3:0]
 *   Byte 3: Y[11:4]
 */

#define PROTOARC_REPORT_LEN 19
#define PROTOARC_MAX_FINGERS 4

/* Scan time counter (increments by ~80 per report at 125Hz = 8ms) */
static uint16_t scan_time_counter = 0;

/* Trackpad report buffer */
static struct zmk_hid_trackpad_report trackpad_report = {
    .report_id = ZMK_HID_REPORT_ID_TRACKPAD,
};

/*
 * Parse ProtoArc finger data
 *
 * ProtoArc and ADG use the same 12-bit packed format for X/Y,
 * but the flags byte layout differs slightly.
 */
static void parse_protoarc_finger(const uint8_t *data,
                                  uint8_t *id, bool *tip, bool *conf,
                                  uint16_t *x, uint16_t *y)
{
    uint8_t flags = data[0];

    /* ProtoArc flags: Confidence(0), TipSwitch(1), ContactID(2-4), Pad(5-7) */
    *conf = flags & 0x01;
    *tip = (flags >> 1) & 0x01;
    *id = (flags >> 2) & 0x07;

    /* X and Y are 12-bit packed (same as ADG) */
    *x = data[1] | ((data[2] & 0x0F) << 8);
    *y = (data[2] >> 4) | (data[3] << 4);
}

/*
 * Process trackpad report and convert to ADG format
 */
static void hogp_trackpad_process_report(const uint8_t *data, uint16_t len)
{
    /* Route 23-byte iTrack reports to the iTrack handler */
    if (len == 23) {
        hogp_itrack_process(data, len);
        return;
    }

    if (len != PROTOARC_REPORT_LEN) {
        LOG_DBG("Unexpected report length: %d (expected %d or 23)", len, PROTOARC_REPORT_LEN);
        return;
    }

    /* Parse button and contact count from last byte */
    uint8_t last_byte = data[18];
    bool button = (last_byte & 0x80) != 0;
    uint8_t contact_count = last_byte & 0x7F;

    /* Clear report body */
    memset(&trackpad_report.body, 0, sizeof(trackpad_report.body));

    /* Set scan time and buttons */
    trackpad_report.body.scan_time = scan_time_counter;
    trackpad_report.body.buttons = button ? 0x01 : 0x00;

    /* Convert each finger */
    for (int i = 0; i < PROTOARC_MAX_FINGERS && i < contact_count; i++) {
        const uint8_t *finger_data = &data[i * 4];

        uint8_t id;
        bool tip, conf;
        uint16_t x, y;

        parse_protoarc_finger(finger_data, &id, &tip, &conf, &x, &y);

        if (tip) {
            zmk_hid_trackpad_finger_set(&trackpad_report.body.fingers[i], id, tip, conf, x, y);
            LOG_DBG("Finger %d: id=%d x=%d y=%d", i, id, x, y);
        }
    }

    /* Clear remaining finger slots */
    for (int i = contact_count; i < ZMK_HID_TRACKPAD_MAX_FINGERS; i++) {
        zmk_hid_trackpad_finger_clear(&trackpad_report.body.fingers[i]);
    }

    /* Send to host via BLE HOG */
    int ret = zmk_hog_send_trackpad_report(&trackpad_report.body);
    if (ret < 0) {
        LOG_WRN("Failed to send trackpad report: %d", ret);
    }

    /* Increment scan time (~8ms at 125Hz = 80 units of 100us) */
    scan_time_counter += 80;
}

/*
 * Initialize HOGP trackpad output
 */
static int hogp_trackpad_init(void)
{
    LOG_INF("HOGP Trackpad (ADG multitouch) initializing...");

    scan_time_counter = 0;

    /* Register as the report callback */
    hogp_register_report_callback(hogp_trackpad_process_report);

    LOG_INF("HOGP Trackpad initialized - forwarding as ADG multitouch");
    return 0;
}

/* Initialize after HOGP central */
SYS_INIT(hogp_trackpad_init, APPLICATION, 92);
