#!/bin/bash
# RTL8723BS rtw88 Driver Diagnostic Script
# Run this script as: sudo ./diagnose.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Run this script as root: sudo ./diagnose.sh"
    exit 1
fi

GIT_HASH=$(git -c safe.directory="$PWD" rev-parse --short=8 HEAD 2>/dev/null) || {
    echo "ERROR: Unable to determine git commit hash"
    exit 1
}

OUTDIR="logs-rtw88-$GIT_HASH"
mkdir -p "$OUTDIR"

echo "=== RTL8723BS Diagnostic Test ==="
echo "Git commit: $GIT_HASH"
echo "Output directory: $OUTDIR"
echo ""

# Helper function
run_cmd() {
    local name="$1"
    shift
    echo "Running: $name"
    "$@" > "$OUTDIR/$name" 2>&1 || echo "Command failed" >> "$OUTDIR/$name"
}

# ============================================================================
# Test 0: Preparation & Interface Discovery
# ============================================================================
echo "[0/6] Preparing environment..."

echo "Building and installing current driver modules..."
make clean
make
make install
depmod -a

# Enable Dynamic Debugging for our custom prints
mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
echo "module rtw_* +p" > /sys/kernel/debug/dynamic_debug/control

dmesg -C

# 1. Unload both drivers completely to clear the target interface
modprobe -r rtw_8723bs 2>/dev/null || true
modprobe -r r8723bs 2>/dev/null || true
sleep 2

# 2. Record which interfaces exist BEFORE we load our driver (e.g., swlan0, wlan0)
EXISTING_IFACES=$(iw dev | awk '$1=="Interface" {print $2}')

# 3. Load the new driver
modprobe rtw_8723bs
sleep 3
echo 0x40 > /sys/module/rtw_core/parameters/debug_mask 2>/dev/null || true

# 4. Find the NEW interface that just appeared
IFACE=""
for i in $(iw dev | awk '$1=="Interface" {print $2}'); do
    if ! echo "$EXISTING_IFACES" | grep -qw "$i"; then
        IFACE="$i"
        break
    fi
done

# Fallback just in case the interface name didn't change for some reason
if [ -z "$IFACE" ]; then
    IFACE=$(iw dev | awk '$1=="Interface" {print $2}' | tail -n 1) 
fi

if [ -z "$IFACE" ]; then
    echo "ERROR: No wireless interface detected by mac80211! Exiting."
    exit 1
fi
echo "Detected rtw88 interface: $IFACE"

# ============================================================================
# Test 1: Fresh Module Load
# ============================================================================
echo "[1/6] Fresh module load..."

dmesg > "$OUTDIR/test-01-init.log"

ip link set "$IFACE" up 2>/dev/null || echo "$IFACE not found" >> "$OUTDIR/test-01-init.log"
sleep 2

iw dev "$IFACE" info > "$OUTDIR/test-01-iw-info.log" 2>&1 || echo "iw failed" >> "$OUTDIR/test-01-iw-info.log"

for f in /sys/kernel/debug/ieee80211/phy*/rtw88/mac_0; do
    if [ -e "$f" ] && [ -f "$f" ]; then
        cat "$f" > "$OUTDIR/test-01-mac0.txt"
        break
    fi
done
[ ! -f "$OUTDIR/test-01-mac0.txt" ] && echo "mac_0 not accessible" > "$OUTDIR/test-01-mac0.txt"

for f in /sys/kernel/debug/ieee80211/phy*/rtw88/mac_1; do
    if [ -e "$f" ] && [ -f "$f" ]; then
        cat "$f" > "$OUTDIR/test-01-mac1.txt"
        break
    fi
done
[ ! -f "$OUTDIR/test-01-mac1.txt" ] && echo "mac_1 not accessible" > "$OUTDIR/test-01-mac1.txt"

# ============================================================================
# Test 2: Single Channel 6 Listen
# ============================================================================
echo "[2/6] Listen on channel 6..."

dmesg -C

iw dev "$IFACE" set channel 6 2>/dev/null || echo "set channel failed" >> "$OUTDIR/test-02-chan6.log"
sleep 10

dmesg > "$OUTDIR/test-02-chan6.log"

# ============================================================================
# Test 3: Full Wi-Fi Scan
# ============================================================================
echo "[3/6] Full Wi-Fi scan..."

dmesg -C

# Run scan and capture output
iw dev "$IFACE" scan > "$OUTDIR/test-03-scan-output.txt" 2>&1 || echo "scan failed" >> "$OUTDIR/test-03-scan-output.txt"

# Get logs after scan
dmesg > "$OUTDIR/test-03-scan-log.txt"

# Get register state after scan
for f in /sys/kernel/debug/ieee80211/phy*/rtw88/mac_0; do
    if [ -e "$f" ] && [ -f "$f" ]; then
        cat "$f" > "$OUTDIR/test-03-mac0-after-scan.txt"
        break
    fi
done
[ ! -f "$OUTDIR/test-03-mac0-after-scan.txt" ] && echo "mac_0 not accessible" > "$OUTDIR/test-03-mac0-after-scan.txt"

for f in /sys/kernel/debug/ieee80211/phy*/rtw88/rf_dump; do
    if [ -e "$f" ] && [ -f "$f" ]; then
        cat "$f" > "$OUTDIR/test-03-rf-dump-after-scan.txt"
        break
    fi
done
[ ! -f "$OUTDIR/test-03-rf-dump-after-scan.txt" ] && echo "rf_dump not accessible" > "$OUTDIR/test-03-rf-dump-after-scan.txt"

# ============================================================================
# Test 4: TX Ping Test
# ============================================================================
echo "[4/6] TX ping test..."

dmesg -C

ip addr add 192.168.100.185/24 dev "$IFACE" 2>/dev/null || echo "addr add failed" >> "$OUTDIR/test-04-ping.log"

# Try to ping - may fail but that's ok
timeout 5 ping -I "$IFACE" 192.168.1.1 -c 3 > "$OUTDIR/test-04-ping-output.txt" 2>&1 || echo "ping failed or timed out" >> "$OUTDIR/test-04-ping-output.txt"

dmesg > "$OUTDIR/test-04-ping-log.txt"

# ============================================================================
# Test 5: All MAC Registers (Moved up)
# ============================================================================
echo "[5/6] Dump all MAC registers..."

{
    for f in /sys/kernel/debug/ieee80211/phy*/rtw88/mac_*; do
        if [ -e "$f" ] && [ -f "$f" ]; then
            echo "=== $(basename $f) ==="
            cat "$f"
            echo ""
        fi
    done
} > "$OUTDIR/test-05-all-mac-registers.txt" 2>&1 || echo "no mac registers" >> "$OUTDIR/test-05-all-mac-registers.txt"

# ============================================================================
# Test 6: Staging Driver Comparison
# ============================================================================
echo "[6/6] Check staging driver..."

dmesg -C

# 1. Clean up IP and unload rtw88
ip addr flush dev "$IFACE" 2>/dev/null || true
modprobe -r rtw_8723bs 2>/dev/null
sleep 2

# 2. Record existing interfaces before loading staging
BEFORE_STAGING=$(iw dev | awk '$1=="Interface" {print $2}')

# 3. Load Staging
modprobe r8723bs 2>/dev/null || echo "Staging driver r8723bs failed to load" > "$OUTDIR/test-06-staging-regs.txt"
sleep 3

# 4. Find the NEW interface spawned by staging
STAGING_IFACE=""
for i in $(iw dev | awk '$1=="Interface" {print $2}'); do
    if ! echo "$BEFORE_STAGING" | grep -qw "$i"; then
        STAGING_IFACE="$i"
        break
    fi
done

echo "Detected staging interface: ${STAGING_IFACE:-NOT_FOUND}"

if [ -n "$STAGING_IFACE" ]; then
    ip link set "$STAGING_IFACE" up 2>/dev/null
    sleep 2
    # The legacy driver uses procfs, not debugfs
    if [ -f "/proc/net/rtl8723bs/$STAGING_IFACE/registers" ]; then
        cat "/proc/net/rtl8723bs/$STAGING_IFACE/registers" > "$OUTDIR/test-06-staging-regs.txt"
    else
        # Some staging versions use a different proc name
        cat /proc/net/rtl8723bs/*/registers 2>/dev/null > "$OUTDIR/test-06-staging-regs.txt" || \
        echo "Staging procfs registers not found" >> "$OUTDIR/test-06-staging-regs.txt"
    fi
else
    echo "Staging interface not found - results may be invalid" >> "$OUTDIR/test-06-staging-regs.txt"
fi

dmesg > "$OUTDIR/test-06-staging-dmesg.log"

# ============================================================================
# Summary
# ============================================================================
echo ""
echo "=== Diagnostic Complete ==="
echo "Files saved to: $OUTDIR"
echo ""

ls -la "$OUTDIR"

# Create tar archive
TARFILE="diagnostic-$GIT_HASH.tar.gz"
tar -czf "$TARFILE" "$OUTDIR"
echo ""
echo "Archive created: $TARFILE"
echo ""
echo "Upload $TARFILE for analysis."
