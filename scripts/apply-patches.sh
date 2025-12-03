#!/bin/bash
# Apply required patches to upstream ZMK for HOGP support
# Run this after `west update` if using upstream ZMK

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ZMK_DIR="${ZMK_DIR:-zmk}"

if [[ ! -d "$ZMK_DIR/app" ]]; then
    echo "Error: ZMK directory not found at $ZMK_DIR"
    echo "Run from your west workspace root, or set ZMK_DIR"
    exit 1
fi

echo "Applying HOGP patches to ZMK..."

# Check if patch already applied
if grep -q "ZMK_HOGP" "$ZMK_DIR/app/Kconfig" 2>/dev/null; then
    echo "Patch already applied (ZMK_HOGP conditional found)"
    exit 0
fi

# Apply the SC_PAIR_ONLY patch
patch -p1 -d "$ZMK_DIR" < "$SCRIPT_DIR/../patches/zmk-sc-pair-only.patch"

echo "Patches applied successfully!"
echo ""
echo "This patch makes BT_SMP_SC_PAIR_ONLY conditional on ZMK_HOGP,"
echo "allowing legacy BLE pairing for external HID devices like trackpads."
