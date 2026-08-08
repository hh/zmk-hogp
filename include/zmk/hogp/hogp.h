/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Global debug flag - toggle via !debug / !nodebug serial commands
 */
extern bool zmk_debug_enabled;

/**
 * @brief Enter HOGP pairing mode
 *
 * Starts BLE scanning for HID devices. Will timeout after
 * CONFIG_ZMK_HOGP_SCAN_DURATION_SEC seconds.
 */
void hogp_enter_pairing_mode(void);

/**
 * @brief Exit HOGP pairing mode
 *
 * Stops scanning for HID devices.
 */
void hogp_exit_pairing_mode(void);

/**
 * @brief Print HOGP status to log
 *
 * Logs current pairing mode, scanning state, and connected devices.
 */
void hogp_print_status(void);

/**
 * @brief Clear HOGP device bonds only
 *
 * Disconnects any connected HOGP devices and removes their bonds.
 * Host device bonds (laptops, etc.) are preserved.
 */
void hogp_clear_bonds(void);

/**
 * @brief Clear host device bonds only
 *
 * Removes bonds for host devices (laptops, etc.).
 * HOGP device bonds (mice, trackpads) are preserved.
 */
void hogp_clear_host_bonds(void);

/**
 * @brief Clear NVS settings only (no BLE operations)
 *
 * Clears the stored HOGP device addresses from NVS without
 * touching BLE connections. Safe to call from serial thread.
 * Reboot required for changes to take effect.
 */
void hogp_clear_nvs_only(void);

/**
 * @brief Start a reconnect scan for known devices
 *
 * Triggers a passive BLE scan to find and reconnect to
 * known HOGP devices that may have woken up.
 */
void hogp_start_reconnect_scan(void);

/**
 * @brief Callback type for received HID reports
 *
 * @param data Pointer to HID report data
 * @param len Length of HID report in bytes
 */
typedef void (*hogp_report_callback_t)(const uint8_t *data, uint16_t len);

/**
 * @brief Register callback for HID reports
 *
 * The callback will be invoked for each HID report received from
 * connected HOGP devices. Only one callback can be registered at a time.
 *
 * @param cb Callback function, or NULL to unregister
 */
void hogp_register_report_callback(hogp_report_callback_t cb);

/**
 * @brief Dump NVS settings state for debugging
 *
 * Logs the current state of NVS-stored HOGP device addresses.
 */
void hogp_dump_nvs_state(void);

/**
 * @brief HOGP indicator states for LED display
 */
enum hogp_indicator_state {
    HOGP_INDICATOR_IDLE,       /* No known devices - dim pulse */
    HOGP_INDICATOR_PAIRING,    /* Pairing mode - fast blink */
    HOGP_INDICATOR_SCANNING,   /* Reconnect scanning - slow blink */
    HOGP_INDICATOR_CONNECTED,  /* Connected and ready - solid */
};

/**
 * @brief Get current HOGP indicator state for LED display
 *
 * @return Current indicator state
 */
enum hogp_indicator_state hogp_get_indicator_state(void);

/**
 * @brief HOGP device types for per-device LED indicators
 */
enum hogp_device_type {
    HOGP_DEVICE_UNKNOWN,   /* Not yet determined */
    HOGP_DEVICE_MOUSE,     /* Mouse (M720, etc.) */
    HOGP_DEVICE_ITRACK,    /* iTrack trackpad */
    HOGP_DEVICE_COUNT
};

/**
 * @brief Get indicator state for a specific device type
 *
 * @param type The device type to query
 * @return Indicator state for that device type
 */
enum hogp_indicator_state hogp_get_device_indicator_state(enum hogp_device_type type);

/**
 * @brief Set the device type for a connected device
 *
 * Called by report handlers when they determine the device type.
 *
 * @param conn The BLE connection
 * @param type The device type
 */
void hogp_set_device_type(struct bt_conn *conn, enum hogp_device_type type);
