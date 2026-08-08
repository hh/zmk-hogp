/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Simple serial command handler for development/debugging.
 * Commands must be prefixed with '!' to avoid confusion with log output.
 * Commands:
 *   !reboot  - Soft reset the keyboard
 *   !boot    - Enter UF2 bootloader mode for flashing
 *   !bootright - (central) Invoke bootloader on the split peripheral
 *   !ble     - Switch to BLE output
 *   !usb     - Switch to USB output
 *   !forget  - Clear BLE profile bonds (keeps split connection)
 *   !nuke    - Clear ALL bonds via bt_unpair (on a peripheral this
 *              clears the split bond - serial alternative to a
 *              settings-reset flash)
 *   !pair    - Enter HOGP pairing mode (if HOGP enabled)
 *   !unpair  - Exit HOGP pairing mode
 *   !scan    - Start reconnect scan for known HOGP devices
 *   !hogp    - Show HOGP status
 *   !clear   - Clear HOGP device bonds (preserves host bonds)
 *   !clearhosts - Clear host bonds (preserves HOGP bonds)
 *   !nvs     - Dump HOGP NVS state
 *   !nvsclear - Clear HOGP NVS settings only (no disconnect)
 *   !prof    - Show BLE profile status
 *   !debug / !nodebug - Toggle verbose runtime debug logging
 *   !version - Show build version info
 *   !help    - Show available commands
 *
 * Also broadcasts BT profile changes for external bridge sync:
 *   Output: "BT:X" where X is profile index (0-4)
 */

/* Build version - generated at compile time */
#include "zmk_build_version.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>

#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/endpoints_types.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/behavior.h>
#include <zmk/split/bluetooth/central.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_HOGP)
#include <zmk/hogp/hogp.h>
#endif

LOG_MODULE_REGISTER(serial_cmd, CONFIG_ZMK_SERIAL_CMD_LOG_LEVEL);

/* Split peripherals have no endpoints or BLE profiles, so transport and
 * profile-bond commands only exist on the central (or a non-split keyboard).
 * Peripherals keep: !reboot, !boot, !nuke, !version, !help. */
#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define SERIAL_CMD_CENTRAL 1
#else
#define SERIAL_CMD_CENTRAL 0
#endif

#if DT_HAS_CHOSEN(zephyr_console)

#define SERIAL_CMD_MAX_LEN 32
#define SERIAL_CMD_STACK_SIZE 512
#define SERIAL_CMD_PRIORITY 10

/* RST_UF2 is typically 0x57 for Adafruit nRF52 bootloader */
#ifndef RST_UF2
#define RST_UF2 0x57
#endif

static char cmd_buf[SERIAL_CMD_MAX_LEN];
static int cmd_pos = 0;
static bool cmd_active = false;  /* True after seeing '!' prefix */

/* Global debug flag - can be toggled at runtime via !debug / !nodebug */
bool zmk_debug_enabled = false;

#define CMD_PREFIX '!'

static void process_command(const char *cmd) {
    /* Trim leading/trailing whitespace */
    while (*cmd == ' ' || *cmd == '\t') cmd++;

    size_t len = strlen(cmd);
    while (len > 0 && (cmd[len-1] == ' ' || cmd[len-1] == '\t' ||
                       cmd[len-1] == '\r' || cmd[len-1] == '\n')) {
        len--;
    }

    if (len == 0) {
        return;
    }

    LOG_INF("Serial command: '%.*s'", (int)len, cmd);

    if (strncmp(cmd, "reboot", 6) == 0 || strncmp(cmd, "reset", 5) == 0) {
        LOG_INF("Rebooting...");
        k_msleep(100);  /* Let log flush */
        sys_reboot(SYS_REBOOT_COLD);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    /* Must be matched before "boot" — that prefix test would swallow it. */
    } else if (strncmp(cmd, "bootright", 9) == 0 || strncmp(cmd, "bootperiph", 10) == 0) {
        LOG_INF("Invoking bootloader on the peripheral over the split link...");
        struct zmk_behavior_binding binding = {.behavior_dev = "bootloader"};
        struct zmk_behavior_binding_event event = {
            .position = 0,
            .timestamp = k_uptime_get(),
        };
        int err = zmk_split_bt_invoke_behavior(0, &binding, event, true);
        if (err) {
            LOG_ERR("Failed to reach peripheral (err %d) — is the right half awake?", err);
        } else {
            LOG_INF("Peripheral should now present its UF2 drive over USB.");
        }
#endif /* CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

    } else if (strncmp(cmd, "boot", 4) == 0 || strncmp(cmd, "flash", 5) == 0 ||
               strncmp(cmd, "dfu", 3) == 0) {
        LOG_INF("Entering bootloader mode...");
        k_msleep(100);  /* Let log flush */
        sys_reboot(RST_UF2);

    } else if (strncmp(cmd, "nuke", 4) == 0) {
        /* Nuclear option: clear ALL bonds via bt_unpair.
         * On a split peripheral this clears the split bond too - a serial
         * alternative to flashing settings-reset when halves red-blink. */
        LOG_INF("NUKING all BLE bonds (bt_unpair NULL)...");
        int err = bt_unpair(BT_ID_DEFAULT, NULL);
        if (err) {
            LOG_ERR("bt_unpair failed: %d", err);
        } else {
            LOG_INF("All bonds nuked. Re-pair everything.");
        }

#if SERIAL_CMD_CENTRAL
    } else if (strncmp(cmd, "ble", 3) == 0) {
        LOG_INF("Switching to BLE output...");
        zmk_endpoints_select_transport(ZMK_TRANSPORT_BLE);

    } else if (strncmp(cmd, "usb", 3) == 0) {
        LOG_INF("Switching to USB output...");
        zmk_endpoints_select_transport(ZMK_TRANSPORT_USB);

    } else if (strncmp(cmd, "forget", 6) == 0) {
        /* Clear BLE profile bonds (but NOT split connection) */
        LOG_INF("Clearing all BLE profile bonds...");
        zmk_ble_clear_all_bonds();
        LOG_INF("Bonds cleared. Re-pair with host to reconnect.");

    } else if (strncmp(cmd, "profiles", 8) == 0 || strncmp(cmd, "prof", 4) == 0) {
        /* Show BLE profile status */
        LOG_INF("=== BLE Profiles ===");
        int active = zmk_ble_active_profile_index();
        LOG_INF("Active profile: %d / %d", active, ZMK_BLE_PROFILE_COUNT);
        bt_addr_le_t *addr = zmk_ble_active_profile_addr();
        if (addr && bt_addr_le_cmp(addr, BT_ADDR_LE_ANY) != 0) {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
            LOG_INF("Active profile addr: %s", addr_str);
        } else {
            LOG_INF("Active profile addr: (empty/unpaired)");
        }
        LOG_INF("====================");
#endif /* SERIAL_CMD_CENTRAL */

#if IS_ENABLED(CONFIG_ZMK_HOGP)
    } else if (strncmp(cmd, "pair", 4) == 0) {
        LOG_INF("Starting HOGP pairing mode...");
        hogp_enter_pairing_mode();

    } else if (strncmp(cmd, "unpair", 6) == 0) {
        LOG_INF("Stopping HOGP pairing mode...");
        hogp_exit_pairing_mode();

    } else if (strncmp(cmd, "scan", 4) == 0) {
        LOG_INF("Starting reconnect scan for known devices...");
        hogp_start_reconnect_scan();

    } else if (strncmp(cmd, "hogp", 4) == 0 || strncmp(cmd, "status", 6) == 0) {
        hogp_print_status();

    } else if (strncmp(cmd, "clearhosts", 10) == 0) {
        LOG_INF("Clearing host bonds only (HOGP preserved)...");
        hogp_clear_host_bonds();

    } else if (strncmp(cmd, "clear", 5) == 0) {
        LOG_INF("Clearing HOGP bonds only (hosts preserved)...");
        hogp_clear_bonds();

    } else if (strncmp(cmd, "nvsclear", 8) == 0) {
        /* Clear NVS only - no BLE operations, won't block */
        LOG_INF("Clearing HOGP NVS settings only (no disconnect)...");
        hogp_clear_nvs_only();
        LOG_INF("NVS cleared. Reboot to take effect.");

    } else if (strncmp(cmd, "nvs", 3) == 0) {
        hogp_dump_nvs_state();
#endif /* CONFIG_ZMK_HOGP */

    } else if (strncmp(cmd, "debug", 5) == 0) {
        zmk_debug_enabled = true;
        LOG_INF("Debug logging ENABLED");

    } else if (strncmp(cmd, "nodebug", 7) == 0) {
        zmk_debug_enabled = false;
        LOG_INF("Debug logging DISABLED");

    } else if (strncmp(cmd, "version", 7) == 0 || strncmp(cmd, "ver", 3) == 0) {
        LOG_INF("=== Build Info ===");
        LOG_INF("Version: %s", ZMK_BUILD_VERSION);
        LOG_INF("Commit:  %s", ZMK_BUILD_COMMIT);
        LOG_INF("Branch:  %s", ZMK_BUILD_BRANCH);
        LOG_INF("Built:   %s UTC", ZMK_BUILD_TIME);
        LOG_INF("==================");

    } else if (strncmp(cmd, "help", 4) == 0) {
        LOG_INF("Commands: !reboot, !boot, !nuke, !debug, !nodebug, !version"
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
                ", !bootright"
#endif
#if SERIAL_CMD_CENTRAL
                ", !ble, !usb, !forget, !prof"
#endif
#if IS_ENABLED(CONFIG_ZMK_HOGP)
                ", !pair, !unpair, !scan, !hogp, !clear, !clearhosts, !nvs, !nvsclear"
#endif
                ", !help");

    } else {
        LOG_WRN("Unknown command: '%.*s'", (int)len, cmd);
    }
}

static void serial_cmd_thread(void *p1, void *p2, void *p3) {
    const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

    if (!device_is_ready(uart)) {
        LOG_ERR("Console UART not ready");
        return;
    }

    LOG_INF("=== ZMK HOGP Build: %s ===", ZMK_BUILD_VERSION);
    LOG_INF("Built: %s UTC", ZMK_BUILD_TIME);
    LOG_INF("Serial command handler ready. Use !command (e.g., !boot, !version, !help)");

    while (1) {
        uint8_t c;

        /* Poll for incoming character */
        if (uart_poll_in(uart, &c) == 0) {
            if (c == CMD_PREFIX) {
                /* Start of command - reset and activate */
                cmd_pos = 0;
                cmd_active = true;
            } else if (c == '\r' || c == '\n') {
                if (cmd_active && cmd_pos > 0) {
                    cmd_buf[cmd_pos] = '\0';
                    process_command(cmd_buf);
                }
                cmd_pos = 0;
                cmd_active = false;
            } else if (cmd_active) {
                /* Only collect chars after seeing prefix */
                if (c >= 'a' && c <= 'z') {
                    if (cmd_pos < SERIAL_CMD_MAX_LEN - 1) {
                        cmd_buf[cmd_pos++] = c;
                    }
                } else if (c == 0x7f || c == 0x08) {  /* Backspace/DEL */
                    if (cmd_pos > 0) {
                        cmd_pos--;
                    }
                }
                /* Ignore other chars while command active */
            }
            /* Ignore all chars when not in command mode */
        }

        k_msleep(10);  /* Don't spin too fast */
    }
}

K_THREAD_DEFINE(serial_cmd_tid, SERIAL_CMD_STACK_SIZE,
                serial_cmd_thread, NULL, NULL, NULL,
                SERIAL_CMD_PRIORITY, 0, 1000);  /* Start after 1 second */

#if SERIAL_CMD_CENTRAL
/*
 * BLE Profile Change Listener
 * Broadcasts profile changes to serial for external bridge sync.
 * Format: "BT:X" where X is profile index (0-4)
 */
static int serial_cmd_event_listener(const zmk_event_t *eh) {
    if (as_zmk_ble_active_profile_changed(eh)) {
        struct zmk_ble_active_profile_changed *ev = as_zmk_ble_active_profile_changed(eh);
        /* Use printk for guaranteed output (not filtered by log level) */
        printk("BT:%d\n", ev->index);
        LOG_INF("BT profile changed to %d", ev->index);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(serial_cmd, serial_cmd_event_listener);
ZMK_SUBSCRIPTION(serial_cmd, zmk_ble_active_profile_changed);
#endif /* SERIAL_CMD_CENTRAL */

#endif /* DT_HAS_CHOSEN(zephyr_console) */
