/*
 * Copyright (c) 2024 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * HOGP iTrack Handler - Brydge iTrack ADG Trackpad
 *
 * Pure passthrough: forward iTrack reports unchanged to USB HID.
 * The USB HID descriptor matches the iTrack format exactly.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <zmk/hogp/hogp.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

LOG_MODULE_REGISTER(hogp_itrack, CONFIG_ZMK_HOGP_LOG_LEVEL);

#define ITRACK_REPORT_LEN_5FINGER   23  /* Full mode: scan(2) + btn(1) + 5 fingers(20) */

/*
 * Pure passthrough - copy raw iTrack data directly to USB HID report.
 * The zmk_hid_trackpad_report_body is exactly 23 bytes matching iTrack format.
 */
void hogp_itrack_process(const uint8_t *data, uint16_t len)
{
    if (len != ITRACK_REPORT_LEN_5FINGER) {
        LOG_WRN("iTrack unexpected length: %d (expected %d)", len, ITRACK_REPORT_LEN_5FINGER);
        return;
    }

    struct zmk_hid_trackpad_report *report = zmk_hid_get_trackpad_report();

    /* Direct memcpy - formats match exactly */
    memcpy(&report->body, data, len);

    if (zmk_debug_enabled) {
        LOG_DBG("iTrack passthrough: %d bytes", len);
    }

    int ret = zmk_endpoints_send_trackpad_report();
    if (ret < 0) {
        LOG_WRN("Failed to send trackpad report: %d", ret);
    }
}
