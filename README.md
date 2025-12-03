# ZMK HOGP Module

BLE HID Central (HOGP - HID over GATT Profile) module for ZMK firmware.

This module allows ZMK keyboards to connect to external BLE HID devices (mice, trackpads) and forward their input through the keyboard.

## Features

- **HOGP Central**: Connect to BLE HID devices as a central
- **Auto-reconnect**: Automatically reconnect to bonded devices on boot
- **Pairing mode**: Manual pairing with configurable timeout
- **Serial commands**: Debug and control via USB serial console

## Installation

### 1. Add to your `west.yml`:

```yaml
manifest:
  remotes:
    - name: hh
      url-base: https://github.com/hh
  projects:
    - name: zmk-hogp
      remote: hh
      revision: main
```

### 2. Run `west update`

### 3. Apply the ZMK patch (required for legacy pairing)

Many BLE trackpads use LE Legacy pairing, but upstream ZMK forces Secure Connections Only. Apply the included patch:

```bash
./zmk-hogp/scripts/apply-patches.sh
```

Or manually patch `zmk/app/Kconfig`:

```diff
 menuconfig ZMK_BLE
     select BT
     select BT_SMP
-    select BT_SMP_SC_PAIR_ONLY
+    select BT_SMP_SC_PAIR_ONLY if !ZMK_HOGP
```

## Configuration

Add to your board's `_defconfig` or `prj.conf`:

```kconfig
# Enable HOGP central
CONFIG_ZMK_HOGP=y
CONFIG_ZMK_HOGP_LOG_LEVEL_DBG=y

# Enable serial commands (auto-enabled with HOGP)
CONFIG_ZMK_SERIAL_CMD=y
CONFIG_ZMK_USB_LOGGING=y

# Required: BLE Central role
CONFIG_BT_CENTRAL=y
CONFIG_BT_GATT_CLIENT=y

# Required: Allow Legacy Pairing for trackpads
CONFIG_BT_SMP_SC_PAIR_ONLY=n
CONFIG_BT_SMP_ENFORCE_MITM=n

# Recommended: Bonding settings
CONFIG_BT_KEYS_OVERWRITE_OLDEST=y
CONFIG_BT_MAX_PAIRED=7
```

## Serial Commands

Commands must be prefixed with `!`:

| Command | Description |
|---------|-------------|
| `!pair` | Enter HOGP pairing mode (30 sec scan) |
| `!unpair` | Exit pairing mode |
| `!hogp` | Show HOGP connection status |
| `!clear` | Clear HOGP device bonds |
| `!forget` | Clear all BLE bonds |
| `!boot` | Enter UF2 bootloader |
| `!reboot` | Soft reset keyboard |
| `!ble` | Switch to BLE output |
| `!usb` | Switch to USB output |
| `!help` | Show available commands |

## Usage

1. Build and flash firmware with USB logging enabled
2. Open serial console: `picocom /dev/ttyACM0 -b 115200`
3. Put your trackpad in pairing mode
4. Send `!pair` to start scanning
5. Device will connect when HID Service UUID (0x1812) is found
6. Switch keyboard to BLE output (`!ble`) to forward trackpad data to host

## Known Working Devices

| Device | MAC Pattern | Notes |
|--------|-------------|-------|
| ProtoArc T1 Plus | `EC:FD:72:xx:xx:xx` | Advertises without name, uses Legacy pairing |

## Troubleshooting

### "Security failed err 4" during pairing

This means the trackpad uses Legacy pairing but ZMK is forcing Secure Connections Only.

**Fix**: Apply the ZMK patch (see Installation step 3) and ensure these configs are set:
```kconfig
CONFIG_BT_SMP_SC_PAIR_ONLY=n
CONFIG_BT_SMP_ENFORCE_MITM=n
```

### No serial port appears after flashing

Ensure USB logging is enabled:
```kconfig
CONFIG_ZMK_USB_LOGGING=y
```

### Trackpad connects but cursor doesn't move

The trackpad data flows through BLE, not USB. Switch the keyboard to BLE output:
```
!ble
```

## API

For custom report handling:

```c
#include <zmk/hogp/hogp.h>

void my_report_handler(const uint8_t *data, uint16_t len) {
    // Process HID report (e.g., forward to host)
}

hogp_register_report_callback(my_report_handler);
```

## Architecture

```
┌─────────────────┐     BLE HOGP     ┌──────────────┐
│   BLE Trackpad  │ ───────────────> │   Keyboard   │
│  (HID Peripheral)│                  │ (HID Central)│
└─────────────────┘                  └──────┬───────┘
                                            │
                                            │ BLE/USB HID
                                            v
                                     ┌──────────────┐
                                     │     Host     │
                                     │   Computer   │
                                     └──────────────┘
```

## License

MIT License - See [LICENSE](LICENSE)
