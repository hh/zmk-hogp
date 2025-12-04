/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * HOGP Mouse Output - Converts trackpad reports to mouse movement
 *
 * This module receives HID reports from HOGP-connected trackpads
 * and converts single-finger touch to relative mouse movement.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <zmk/hogp/hogp.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>

LOG_MODULE_REGISTER(hogp_mouse, CONFIG_ZMK_HOGP_LOG_LEVEL);

/* ProtoArc trackpad HID report format (from actual HID descriptor):
 *
 * BLE HOGP notifications do NOT include the Report ID - it's implicit
 * from which characteristic was subscribed to.
 *
 * For Report ID 5 characteristic, we receive 19 bytes (20 - 1 for ID):
 *   Bytes 0-3:   Finger 1 data
 *   Bytes 4-7:   Finger 2 data
 *   Bytes 8-11:  Finger 3 data
 *   Bytes 12-15: Finger 4 data
 *   Bytes 16-17: Scan Time (16-bit little endian)
 *   Byte 18:     Contact Count (7 bits) | Button (bit 7)
 *
 * Finger data format (4 bytes each):
 *   Byte 0: Flags
 *     - Bit 0: Confidence
 *     - Bit 1: Tip Switch (1 = finger touching)
 *     - Bits 2-4: Contact ID (0-7)
 *     - Bits 5-7: Padding
 *   Bytes 1-3: X and Y coordinates (12-bit each, packed)
 *     - Byte 1: X[7:0]
 *     - Byte 2: X[11:8] (low nibble) | Y[3:0] (high nibble)
 *     - Byte 3: Y[11:4]
 *
 * Coordinate ranges (from HID descriptor):
 *   X: 0-2557 (physical 0-165mm)
 *   Y: 0-1154 (physical 0-110mm)
 */

#define PROTOARC_REPORT_LEN 19  /* Report ID 5 data WITHOUT the ID prefix */
#define PROTOARC_FINGER_SIZE 4
#define PROTOARC_MIN_REPORT_SIZE 4  /* At least 1 finger */

/* Finger tracking state */
struct finger_track {
    bool active;
    uint16_t prev_x;
    uint16_t prev_y;
    bool has_prev;  /* True after first position captured */
};

static struct finger_track primary_finger;
static struct finger_track scroll_track;  /* For two-finger scroll */
static bool button_pressed = false;
static uint8_t prev_contact_count = 0;

/* Scroll divisor - higher = slower scroll */
#define SCROLL_DIVISOR 8

/* Sensitivity divisor - higher = slower cursor */
#ifndef CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY
#define CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY 1
#endif

/*
 * Parse coordinates from a finger data block (4 bytes)
 *
 * Finger data format:
 *   Byte 0: Flags (confidence, tip_switch, contact_id)
 *   Byte 1: X[7:0]
 *   Byte 2: X[11:8] (low nibble) | Y[3:0] (high nibble)
 *   Byte 3: Y[11:4]
 *
 * Returns tip_switch status (true if finger touching)
 */
static bool parse_finger(const uint8_t *finger, uint16_t *x, uint16_t *y) {
    uint8_t flags = finger[0];
    bool tip_switch = (flags >> 1) & 0x01;

    /* Parse 12-bit packed X and Y coordinates */
    *x = finger[1] | ((finger[2] & 0x0F) << 8);
    *y = (finger[2] >> 4) | (finger[3] << 4);

    return tip_switch;
}

/*
 * Process trackpad report and convert to mouse movement/scroll
 *
 * BLE HOGP notifications don't include Report ID - data starts at finger 1
 */
static void hogp_mouse_process_report(const uint8_t *data, uint16_t len) {
    if (len < PROTOARC_MIN_REPORT_SIZE) {
        LOG_DBG("Report too short: %d bytes", len);
        return;
    }

    /* For 19-byte reports (Report ID 5 without the ID prefix):
     * - Bytes 0-15: 4 fingers × 4 bytes
     * - Bytes 16-17: Scan Time
     * - Byte 18: Contact Count (7 bits) | Button (bit 7)
     */
    if (len != PROTOARC_REPORT_LEN) {
        LOG_DBG("Unexpected report length: %d (expected %d)", len, PROTOARC_REPORT_LEN);
    }

    /* Button and contact count are in the LAST byte */
    uint8_t last_byte = data[len - 1];
    bool button = (last_byte & 0x80) != 0;
    uint8_t contact_count = last_byte & 0x7F;

    /* Handle button state change */
    if (button != button_pressed) {
        button_pressed = button;
        zmk_hid_mouse_clear();  /* Clear movement/scroll before button event */
        if (button) {
            zmk_hid_mouse_button_press(0);  /* Left button */
            LOG_INF("Click");
        } else {
            zmk_hid_mouse_button_release(0);
        }
        zmk_endpoints_send_mouse_report();
    }

    /* Parse finger 1 (bytes 0-3) */
    uint16_t x1, y1;
    bool tip1 = parse_finger(&data[0], &x1, &y1);

    /* Detect mode transition (1 finger <-> 2 fingers) */
    if (contact_count != prev_contact_count) {
        /* Reset tracking on mode change to avoid jumps */
        primary_finger.has_prev = false;
        scroll_track.has_prev = false;
        prev_contact_count = contact_count;
        LOG_DBG("Mode change: %d fingers", contact_count);
    }

    if (contact_count == 2) {
        /* TWO-FINGER SCROLL MODE */
        /* Use finger 1 Y delta for vertical scroll */
        if (tip1 && scroll_track.has_prev) {
            int16_t dy = (int16_t)y1 - (int16_t)scroll_track.prev_y;

            /* Skip large jumps */
            if (dy > -150 && dy < 150) {
                /* Accumulate and scale for scroll */
                int8_t scroll_y = -(dy / SCROLL_DIVISOR);  /* Negative = natural scroll */
                if (scroll_y != 0) {
                    zmk_hid_mouse_clear();  /* Clear movement before scroll */
                    zmk_hid_mouse_scroll_set(0, scroll_y);
                    zmk_endpoints_send_mouse_report();
                    LOG_DBG("Scroll: %d", scroll_y);
                }
            }
        }
        scroll_track.prev_x = x1;
        scroll_track.prev_y = y1;
        scroll_track.has_prev = tip1;
        scroll_track.active = tip1;

    } else if (contact_count == 1 && tip1) {
        /* SINGLE-FINGER MOVE MODE */
        if (primary_finger.has_prev) {
            int16_t dx = (int16_t)x1 - (int16_t)primary_finger.prev_x;
            int16_t dy = (int16_t)y1 - (int16_t)primary_finger.prev_y;

            /* Skip large jumps */
            if (dx > -150 && dx < 150 && dy > -150 && dy < 150) {
                /* Apply sensitivity scaling */
                dx = dx / CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY;
                dy = dy / CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY;

                if (dx != 0 || dy != 0) {
                    zmk_hid_mouse_clear();  /* Clear scroll before movement */
                    /* Re-apply button state if held (for click-and-drag) */
                    if (button_pressed) {
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
        /* No fingers - clear state */
        if (primary_finger.active || scroll_track.active) {
            primary_finger.active = false;
            primary_finger.has_prev = false;
            scroll_track.active = false;
            scroll_track.has_prev = false;
            zmk_hid_mouse_clear();
        }
    }
}

/*
 * Initialize HOGP mouse output
 */
static int hogp_mouse_init(void) {
    LOG_INF("HOGP Mouse output initializing...");

    memset(&primary_finger, 0, sizeof(primary_finger));
    memset(&scroll_track, 0, sizeof(scroll_track));
    prev_contact_count = 0;

    /* Register as the report callback */
    hogp_register_report_callback(hogp_mouse_process_report);

    LOG_INF("HOGP Mouse output initialized (sensitivity=%d)",
            CONFIG_ZMK_HOGP_MOUSE_SENSITIVITY);
    return 0;
}

/* Initialize after HOGP central */
SYS_INIT(hogp_mouse_init, APPLICATION, 92);
