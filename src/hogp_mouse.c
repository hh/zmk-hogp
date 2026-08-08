/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * HOGP Mouse Output - Handles HID input from BLE mice and trackpads
 *
 * Supports two device types:
 * 1. Standard HID mice (e.g., Logitech M720) - relative X/Y, buttons, wheel
 * 2. Multitouch trackpads (e.g., ProtoArc) - absolute position, converted to relative
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <zmk/hogp/hogp.h>
#include <zmk/hid.h>
#include <zmk/hog.h>
#include <zmk/endpoints.h>
#include <string.h>

LOG_MODULE_REGISTER(hogp_mouse, CONFIG_ZMK_HOGP_LOG_LEVEL);

/*============================================================================
 * Standard Mouse Support (e.g., Logitech M720 Triathlon)
 *
 * Report format (7 bytes, no report ID from BLE):
 *   Byte 0:   Buttons [7:0] (left=0x01, right=0x02, middle=0x04, back=0x08, fwd=0x10)
 *   Byte 1:   Buttons [15:8] (unused on most mice)
 *   Byte 2:   X low 8 bits
 *   Byte 3:   X[11:8] (low nibble) | Y[3:0] (high nibble)
 *   Byte 4:   Y high 8 bits
 *   Byte 5:   Wheel (signed int8)
 *   Byte 6:   H-scroll / wheel tilt (signed int8)
 *
 * Note: BLE HOGP strips the report ID, so we receive 7 bytes not 8.
 *============================================================================*/

#define MOUSE_REPORT_LEN 7

/* Button masks */
#define MOUSE_BTN_LEFT   0x01
#define MOUSE_BTN_RIGHT  0x02
#define MOUSE_BTN_MIDDLE 0x04
#define MOUSE_BTN_BACK   0x08
#define MOUSE_BTN_FWD    0x10


/*
 * Process standard mouse HID report - direct passthrough
 *
 * We directly set the mouse report fields instead of using the reference-counting
 * button APIs. This gives faithful 1:1 passthrough of the mouse state.
 */
static void process_mouse_report(const uint8_t *data, uint16_t len) {
    if (len < MOUSE_REPORT_LEN) {
        LOG_WRN("Mouse report too short: %d bytes", len);
        return;
    }

    uint8_t buttons = data[0];

    /* Parse 12-bit X (signed) */
    int16_t x_raw = data[2] | ((data[3] & 0x0F) << 8);
    if (x_raw & 0x800) {
        x_raw -= 0x1000;  /* Sign extend from 12 bits */
    }

    /* Parse 12-bit Y (signed) */
    int16_t y_raw = (data[3] >> 4) | (data[4] << 4);
    if (y_raw & 0x800) {
        y_raw -= 0x1000;  /* Sign extend from 12 bits */
    }

    int8_t wheel = (int8_t)data[5];
    int8_t hscroll = (int8_t)data[6];

    /* Direct passthrough: get the report and set fields directly */
    struct zmk_hid_mouse_report *report = zmk_hid_get_mouse_report();
    report->body.buttons = buttons;  /* Direct button state - no reference counting */
    report->body.d_x = x_raw;
    report->body.d_y = y_raw;
    report->body.d_scroll_y = wheel;
    report->body.d_scroll_x = hscroll;

    /* Send the report */
    zmk_endpoints_send_mouse_report();

    /* Debug logging for non-trivial events */
    if (zmk_debug_enabled && (buttons || x_raw || y_raw || wheel || hscroll)) {
        LOG_DBG("Mouse: btn=0x%02x x=%d y=%d wheel=%d hscroll=%d",
                buttons, x_raw, y_raw, wheel, hscroll);
    }
}

/*============================================================================
 * Trackpad Support (e.g., ProtoArc)
 *
 * Report format (19 bytes, no report ID from BLE):
 *   Bytes 0-3:   Finger 1 data
 *   Bytes 4-7:   Finger 2 data
 *   Bytes 8-11:  Finger 3 data
 *   Bytes 12-15: Finger 4 data
 *   Bytes 16-17: Scan Time (16-bit little endian)
 *   Byte 18:     Contact Count (7 bits) | Button (bit 7)
 *
 * Finger data format (4 bytes each):
 *   Byte 0: Flags (confidence, tip_switch, contact_id)
 *   Bytes 1-3: X and Y coordinates (12-bit each, packed)
 *============================================================================*/

#define TRACKPAD_REPORT_LEN 19
#define TRACKPAD_FINGER_SIZE 4

/* Finger tracking state for trackpad */
struct finger_track {
    bool active;
    uint16_t prev_x;
    uint16_t prev_y;
    bool has_prev;
};

static struct finger_track primary_finger;
static struct finger_track scroll_track;
static bool trackpad_button_pressed = false;
static uint8_t prev_contact_count = 0;

#define SCROLL_DIVISOR 8

#ifndef CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY
#define CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY 1
#endif

/*
 * Parse trackpad finger data
 */
static bool parse_trackpad_finger(const uint8_t *finger, uint16_t *x, uint16_t *y) {
    uint8_t flags = finger[0];
    bool tip_switch = (flags >> 1) & 0x01;

    *x = finger[1] | ((finger[2] & 0x0F) << 8);
    *y = (finger[2] >> 4) | (finger[3] << 4);

    return tip_switch;
}

/*
 * Process trackpad report (multitouch -> relative mouse)
 */
static void process_trackpad_report(const uint8_t *data, uint16_t len) {
    if (len < TRACKPAD_REPORT_LEN) {
        if (zmk_debug_enabled) {
            LOG_DBG("Trackpad report too short: %d bytes", len);
        }
        return;
    }

    uint8_t last_byte = data[len - 1];
    bool button = (last_byte & 0x80) != 0;
    uint8_t contact_count = last_byte & 0x7F;

    /* Handle button state change */
    if (button != trackpad_button_pressed) {
        trackpad_button_pressed = button;
        zmk_hid_mouse_clear();
        if (button) {
            zmk_hid_mouse_button_press(0);
            if (zmk_debug_enabled) {
                LOG_INF("Trackpad click");
            }
        } else {
            zmk_hid_mouse_button_release(0);
        }
        zmk_endpoints_send_mouse_report();
    }

    /* Parse finger 1 */
    uint16_t x1, y1;
    bool tip1 = parse_trackpad_finger(&data[0], &x1, &y1);

    /* Detect mode transition */
    if (contact_count != prev_contact_count) {
        primary_finger.has_prev = false;
        scroll_track.has_prev = false;
        prev_contact_count = contact_count;
        if (zmk_debug_enabled) {
            LOG_DBG("Trackpad mode: %d fingers", contact_count);
        }
    }

    if (contact_count == 2) {
        /* Two-finger scroll */
        if (tip1 && scroll_track.has_prev) {
            int16_t dy = (int16_t)y1 - (int16_t)scroll_track.prev_y;

            if (dy > -150 && dy < 150) {
                int8_t scroll_y = -(dy / SCROLL_DIVISOR);
                if (scroll_y != 0) {
                    zmk_hid_mouse_clear();
                    zmk_hid_mouse_scroll_set(0, scroll_y);
                    zmk_endpoints_send_mouse_report();
                }
            }
        }
        scroll_track.prev_x = x1;
        scroll_track.prev_y = y1;
        scroll_track.has_prev = tip1;
        scroll_track.active = tip1;

    } else if (contact_count == 1 && tip1) {
        /* Single-finger move */
        if (primary_finger.has_prev) {
            int16_t dx = (int16_t)x1 - (int16_t)primary_finger.prev_x;
            int16_t dy = (int16_t)y1 - (int16_t)primary_finger.prev_y;

            if (dx > -150 && dx < 150 && dy > -150 && dy < 150) {
                dx = dx / CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY;
                dy = dy / CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY;

                if (dx != 0 || dy != 0) {
                    zmk_hid_mouse_clear();
                    if (trackpad_button_pressed) {
                        zmk_hid_mouse_button_press(0);
                    }
                    zmk_hid_mouse_movement_set(dx, dy);
                    zmk_endpoints_send_mouse_report();
                }
            }
        }
        primary_finger.prev_x = x1;
        primary_finger.prev_y = y1;
        primary_finger.has_prev = true;
        primary_finger.active = true;

    } else if (contact_count == 0) {
        if (primary_finger.active || scroll_track.active) {
            primary_finger.active = false;
            primary_finger.has_prev = false;
            scroll_track.active = false;
            scroll_track.has_prev = false;
            zmk_hid_mouse_clear();
        }
    }
}

/*============================================================================
 * Report Dispatcher - Routes reports to appropriate handler by length
 *============================================================================*/

#if IS_ENABLED(CONFIG_ZMK_HOGP_ITRACK_OUTPUT)
/* iTrack handler in hogp_itrack.c */
extern void hogp_itrack_process(const uint8_t *data, uint16_t len);
#define ITRACK_REPORT_LEN_FINGERS 20
#define ITRACK_REPORT_LEN_FULL 23
#define ITRACK_KEYBOARD_LEN 8
#endif

static void hogp_mouse_process_report(const uint8_t *data, uint16_t len) {
    if (zmk_debug_enabled) {
        LOG_DBG("HID report received: %d bytes", len);
    }

    if (len == MOUSE_REPORT_LEN) {
        /* Standard mouse (e.g., Logitech M720) */
        process_mouse_report(data, len);
#if IS_ENABLED(CONFIG_ZMK_HOGP_ITRACK_OUTPUT)
    } else if (len == ITRACK_REPORT_LEN_FINGERS || len == ITRACK_REPORT_LEN_FULL) {
        /* Brydge iTrack ADG trackpad */
        hogp_itrack_process(data, len);
    } else if (len == ITRACK_KEYBOARD_LEN) {
        /* iTrack keyboard report - ignore */
        if (zmk_debug_enabled) {
            LOG_DBG("Ignoring iTrack keyboard report");
        }
#endif
    } else if (len == TRACKPAD_REPORT_LEN) {
        /* Multitouch trackpad (e.g., ProtoArc) */
        process_trackpad_report(data, len);
    } else if (len >= 4 && len < MOUSE_REPORT_LEN) {
        /* Possibly a simpler mouse format - try to handle it */
        LOG_WRN("Unknown short report (%d bytes), attempting mouse parse", len);
        /* For very basic mice: btn, x, y (3 bytes) or btn, x, y, wheel (4 bytes) */
        uint8_t buttons = data[0];
        int8_t x = (len > 1) ? (int8_t)data[1] : 0;
        int8_t y = (len > 2) ? (int8_t)data[2] : 0;
        int8_t wheel = (len > 3) ? (int8_t)data[3] : 0;

        zmk_hid_mouse_clear();
        if (buttons & 0x01) zmk_hid_mouse_button_press(0);
        if (buttons & 0x02) zmk_hid_mouse_button_press(1);
        if (buttons & 0x04) zmk_hid_mouse_button_press(2);
        zmk_hid_mouse_movement_set(x, y);
        if (wheel != 0) zmk_hid_mouse_scroll_set(0, wheel);
        zmk_endpoints_send_mouse_report();
    } else {
        LOG_WRN("Unknown HID report length: %d bytes", len);
        LOG_HEXDUMP_DBG(data, len, "Unknown report");
    }
}

/*============================================================================
 * Initialization
 *============================================================================*/

static int hogp_mouse_init(void) {
    LOG_INF("HOGP Mouse/Trackpad handler initializing...");

    /* Reset state */
    memset(&primary_finger, 0, sizeof(primary_finger));
    memset(&scroll_track, 0, sizeof(scroll_track));
    trackpad_button_pressed = false;
    prev_contact_count = 0;

    /* Register as the report callback */
    hogp_register_report_callback(hogp_mouse_process_report);

    LOG_INF("HOGP Mouse/Trackpad handler ready (sensitivity=%d)",
            CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY);
    return 0;
}

SYS_INIT(hogp_mouse_init, APPLICATION, 92);
