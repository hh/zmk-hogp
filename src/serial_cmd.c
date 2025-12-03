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
 *   !ble     - Switch to BLE output
 *   !usb     - Switch to USB output
 *   !forget  - Clear BLE profile bonds (keeps split connection)
 *   !pair    - Enter HOGP pairing mode (if HOGP enabled)
 *   !unpair  - Exit HOGP pairing mode
 *   !hogp    - Show HOGP status
 *   !clear   - Clear HOGP device bonds
 *   !help    - Show available commands
 *
 * Also broadcasts BT profile changes for external bridge sync:
 *   Output: "BT:X" where X is profile index (0-4)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include <string.h>

#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/endpoints_types.h>

#if IS_ENABLED(CONFIG_ZMK_HOGP)
#include <zmk/hogp/hogp.h>
#endif

LOG_MODULE_REGISTER(serial_cmd, CONFIG_ZMK_SERIAL_CMD_LOG_LEVEL);

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

    } else if (strncmp(cmd, "boot", 4) == 0 || strncmp(cmd, "flash", 5) == 0 ||
               strncmp(cmd, "dfu", 3) == 0) {
        LOG_INF("Entering bootloader mode...");
        k_msleep(100);  /* Let log flush */
        sys_reboot(RST_UF2);

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

#if IS_ENABLED(CONFIG_ZMK_HOGP)
    } else if (strncmp(cmd, "pair", 4) == 0) {
        LOG_INF("Starting HOGP pairing mode...");
        hogp_enter_pairing_mode();

    } else if (strncmp(cmd, "unpair", 6) == 0) {
        LOG_INF("Stopping HOGP pairing mode...");
        hogp_exit_pairing_mode();

    } else if (strncmp(cmd, "hogp", 4) == 0 || strncmp(cmd, "status", 6) == 0) {
        hogp_print_status();

    } else if (strncmp(cmd, "clear", 5) == 0) {
        LOG_INF("Clearing all HOGP bonds...");
        hogp_clear_bonds();
#endif /* CONFIG_ZMK_HOGP */

    } else if (strncmp(cmd, "help", 4) == 0) {
        LOG_INF("Commands: !reboot, !boot, !ble, !usb, !forget"
#if IS_ENABLED(CONFIG_ZMK_HOGP)
                ", !pair, !unpair, !hogp, !clear"
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

    LOG_INF("Serial command handler ready. Use !command (e.g., !boot, !reboot, !help)");

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

#endif /* DT_HAS_CHOSEN(zephyr_console) */
