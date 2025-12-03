# ZMK HOGP Module

BLE HID Central (HOGP - HID over GATT Profile) module for ZMK firmware.

This module allows ZMK keyboards to connect to external BLE HID devices (mice, trackpads) and receive their input reports.

## Features

- **HOGP Central**: Connect to BLE HID devices as a central
- **Auto-reconnect**: Automatically reconnect to bonded devices on boot
- **Pairing mode**: Manual pairing with timeout
- **Serial commands**: Debug and control via USB serial console

## Installation

Add to your `west.yml`:

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

Then run `west update`.

## Configuration

Add to your `prj.conf` or board config:

```kconfig
# Enable HOGP central
CONFIG_ZMK_HOGP=y

# Enable serial commands (optional, enabled by default with HOGP)
CONFIG_ZMK_SERIAL_CMD=y

# Required: Allow Legacy Pairing for some trackpads
CONFIG_BT_SMP_SC_PAIR_ONLY=n
```

## Serial Commands

Commands must be prefixed with `!`:

| Command | Description |
|---------|-------------|
| `!pair` | Enter HOGP pairing mode (30 sec) |
| `!unpair` | Exit pairing mode |
| `!hogp` | Show HOGP connection status |
| `!clear` | Clear HOGP device bonds |
| `!forget` | Clear BLE profile bonds |
| `!boot` | Enter UF2 bootloader |
| `!reboot` | Soft reset keyboard |
| `!ble` | Switch to BLE output |
| `!usb` | Switch to USB output |
| `!help` | Show available commands |

## Usage

1. Enable HOGP in your config
2. Build and flash firmware
3. Connect to USB serial console
4. Send `!pair` to start scanning
5. Put your BLE trackpad/mouse in pairing mode
6. Device will connect automatically

## API

For Phase 2 integration (forwarding HID reports to host):

```c
#include <zmk/hogp/hogp.h>

// Register callback for received HID reports
void my_report_handler(const uint8_t *data, uint16_t len) {
    // Process HID report
}

hogp_register_report_callback(my_report_handler);
```

## License

MIT License - See [LICENSE](LICENSE)
