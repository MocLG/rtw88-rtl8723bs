#!/bin/bash
# RTL8723BS rtw88 Driver Diagnostic Script
# Run this script as: sudo ./diagnose.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Run this script as root: sudo ./diagnose.sh"
    exit 1
fi

for cmd in git make depmod modprobe iw ip dmesg timeout tar ping awk grep; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "ERROR: Required command not found: $cmd"
        exit 1
    fi
done

GIT_HASH=$(git -c safe.directory="$PWD" rev-parse --short=8 HEAD 2>/dev/null) || {
    echo "ERROR: Unable to determine git commit hash"
    exit 1
}

OUTDIR="logs-rtw88-$GIT_HASH"
TARFILE="diagnostic-$GIT_HASH.tar.gz"
rm -f "$TARFILE"
if [ -e "$OUTDIR" ]; then
    rm -rf "$OUTDIR"
fi
mkdir -p "$OUTDIR"

echo "=== RTL8723BS Diagnostic Test ==="
echo "Git commit: $GIT_HASH"
echo "Output directory: $OUTDIR"
echo ""

clear_dmesg() {
    dmesg -C 2>/dev/null || true
}

append_if_nonempty() {
    local dst="$1"
    local title="$2"
    local src="$3"

    if [ -s "$src" ]; then
        {
            echo ""
            echo "=== $title ==="
            cat "$src"
        } >> "$dst"
    fi
}

list_managed_ifaces() {
    iw dev | awk '
        $1 == "Interface" {
            iface = $2
            next
        }
        $1 == "type" && iface != "" {
            if ($2 == "managed")
                print iface
            iface = ""
        }
    '
}

iface_in_list() {
    local needle="$1"
    local list="$2"

    printf '%s\n' "$list" | grep -Fxq "$needle"
}

debug_dir_for_iface() {
    local iface="$1"
    local phy
    local dir
    local found=""
    local count=0

    phy=$(iw dev "$iface" info 2>/dev/null | awk '$1 == "wiphy" { print "phy" $2; exit }')
    if [ -n "$phy" ] && [ -d "/sys/kernel/debug/ieee80211/$phy/rtw88" ]; then
        echo "/sys/kernel/debug/ieee80211/$phy/rtw88"
        return
    fi

    for dir in /sys/kernel/debug/ieee80211/phy*/rtw88; do
        if [ -d "$dir" ]; then
            found="$dir"
            count=$((count + 1))
        fi
    done

    if [ "$count" -eq 1 ]; then
        echo "$found"
    fi
}

dump_rtw88_file() {
    local name="$1"
    local out="$2"

    if [ -n "$RTW88_DEBUG_DIR" ] && [ -f "$RTW88_DEBUG_DIR/$name" ]; then
        cat "$RTW88_DEBUG_DIR/$name" > "$out" 2>&1 || \
            echo "$name not accessible" > "$out"
    else
        echo "$name not accessible" > "$out"
    fi
}

# ============================================================================
# Test 0: Preparation & Interface Discovery
# ============================================================================
echo "[0/6] Preparing environment..."

echo "Building and installing current driver modules..."
{
    echo "+ make clean"
    make clean
    echo "+ make"
    make
    echo "+ make install"
    make install
    echo "+ depmod -a"
    depmod -a
} > "$OUTDIR/test-00-build-install.log" 2>&1 || {
    echo "ERROR: build/install failed; see $OUTDIR/test-00-build-install.log"
    exit 1
}

# Enable Dynamic Debugging for our custom prints
{
    mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
    if [ -w /sys/kernel/debug/dynamic_debug/control ]; then
        echo "module rtw_* +p" > /sys/kernel/debug/dynamic_debug/control || \
            echo "dynamic_debug write failed"
    else
        echo "dynamic_debug control not writable or unavailable"
    fi
} > "$OUTDIR/test-00-setup.log" 2>&1

clear_dmesg

# 1. Unload both drivers completely to clear the target interface
modprobe -r rtw_8723bs 2>/dev/null || true
modprobe -r r8723bs 2>/dev/null || true
sleep 2

# 2. Record which managed interfaces exist BEFORE we load our driver.
EXISTING_IFACES=$(list_managed_ifaces || true)
EXISTING_IW_DEV=$(iw dev 2>&1 || true)

# 3. Load the new driver
modprobe rtw_8723bs >> "$OUTDIR/test-00-setup.log" 2>&1
sleep 3
if [ -w /sys/kernel/debug/dynamic_debug/control ]; then
    echo "module rtw_* +p" > /sys/kernel/debug/dynamic_debug/control 2>> "$OUTDIR/test-00-setup.log" || \
        echo "dynamic_debug post-load write failed" >> "$OUTDIR/test-00-setup.log"
fi
echo 0x40 > /sys/module/rtw_core/parameters/debug_mask 2>> "$OUTDIR/test-00-setup.log" || \
    echo "rtw_core debug_mask unavailable" >> "$OUTDIR/test-00-setup.log"

# 4. Find the NEW managed interface that just appeared
IFACE=""
RTW88_DEBUG_DIR=""
CURRENT_IFACES=$(list_managed_ifaces || true)
CURRENT_IW_DEV=$(iw dev 2>&1 || true)
{
    echo "Before rtw88 load managed interfaces:"
    echo "$EXISTING_IFACES"
    echo ""
    echo "Before rtw88 load iw dev:"
    echo "$EXISTING_IW_DEV"
    echo ""
    echo "After rtw88 load managed interfaces:"
    echo "$CURRENT_IFACES"
    echo ""
    echo "After rtw88 load iw dev:"
    echo "$CURRENT_IW_DEV"
} > "$OUTDIR/test-00-interfaces.txt"

for i in $CURRENT_IFACES; do
    if ! iface_in_list "$i" "$EXISTING_IFACES"; then
        IFACE="$i"

        # Prefer the new interface whose phy exposes rtw88 debugfs.
        RTW88_DEBUG_DIR=$(debug_dir_for_iface "$i")
        if [ -n "$RTW88_DEBUG_DIR" ]; then
            break
        fi
    fi
done

if [ -z "$IFACE" ]; then
    echo "ERROR: No new rtw88 wireless interface detected. See $OUTDIR/test-00-interfaces.txt"
    exit 1
fi
if [ -z "$RTW88_DEBUG_DIR" ]; then
    RTW88_DEBUG_DIR=$(debug_dir_for_iface "$IFACE")
fi
{
    echo ""
    echo "Selected rtw88 interface: $IFACE"
    echo "Selected rtw88 debugfs: ${RTW88_DEBUG_DIR:-NOT_FOUND}"
} >> "$OUTDIR/test-00-interfaces.txt"
echo "Detected rtw88 interface: $IFACE"

# ============================================================================
# Test 1: Fresh Module Load
# ============================================================================
echo "[1/6] Fresh module load..."

IFACE_UP_LOG="$OUTDIR/test-01-iface-up.txt"
ip link set "$IFACE" up > "$IFACE_UP_LOG" 2>&1 || \
    echo "ip link set $IFACE up failed" >> "$IFACE_UP_LOG"
sleep 2

dmesg > "$OUTDIR/test-01-init.log"
append_if_nonempty "$OUTDIR/test-01-init.log" "ip link set output" "$IFACE_UP_LOG"

iw dev "$IFACE" info > "$OUTDIR/test-01-iw-info.log" 2>&1 || echo "iw failed" >> "$OUTDIR/test-01-iw-info.log"

dump_rtw88_file mac_0 "$OUTDIR/test-01-mac0.txt"
dump_rtw88_file mac_1 "$OUTDIR/test-01-mac1.txt"

# ============================================================================
# Test 2: Single Channel 6 Listen
# ============================================================================
echo "[2/6] Listen on channel 6..."

clear_dmesg

CHAN6_CMD_LOG="$OUTDIR/test-02-chan6-cmd.txt"
iw dev "$IFACE" set channel 6 > "$CHAN6_CMD_LOG" 2>&1 || \
    echo "set channel failed" >> "$CHAN6_CMD_LOG"
sleep 10

dmesg > "$OUTDIR/test-02-chan6.log"
append_if_nonempty "$OUTDIR/test-02-chan6.log" "iw set channel output" "$CHAN6_CMD_LOG"

# ============================================================================
# Test 3: Full Wi-Fi Scan
# ============================================================================
echo "[3/6] Full Wi-Fi scan..."

clear_dmesg

# Run scan and capture output
timeout 60 iw dev "$IFACE" scan > "$OUTDIR/test-03-scan-output.txt" 2>&1 || \
    echo "scan failed or timed out" >> "$OUTDIR/test-03-scan-output.txt"

# Get logs after scan
dmesg > "$OUTDIR/test-03-scan-log.txt"

# Get register state after scan
dump_rtw88_file mac_0 "$OUTDIR/test-03-mac0-after-scan.txt"
dump_rtw88_file rf_dump "$OUTDIR/test-03-rf-dump-after-scan.txt"

# ============================================================================
# Test 4: TX Ping Test
# ============================================================================
echo "[4/6] TX ping test..."

clear_dmesg

PING_CMD_LOG="$OUTDIR/test-04-ping-cmd.txt"
ip addr add 192.168.100.185/24 dev "$IFACE" > "$PING_CMD_LOG" 2>&1 || \
    echo "addr add failed" >> "$PING_CMD_LOG"

# Try to ping - may fail but that's ok
timeout 5 ping -I "$IFACE" 192.168.1.1 -c 3 > "$OUTDIR/test-04-ping-output.txt" 2>&1 || echo "ping failed or timed out" >> "$OUTDIR/test-04-ping-output.txt"

dmesg > "$OUTDIR/test-04-ping-log.txt"
append_if_nonempty "$OUTDIR/test-04-ping-log.txt" "ip addr add output" "$PING_CMD_LOG"

# ============================================================================
# Test 5: All MAC Registers (Moved up)
# ============================================================================
echo "[5/6] Dump all MAC registers..."

{
    found=0
    if [ -n "$RTW88_DEBUG_DIR" ]; then
        for f in "$RTW88_DEBUG_DIR"/mac_*; do
            if [ ! -e "$f" ] || [ ! -f "$f" ]; then
                continue
            fi
            found=1
            echo "=== $(basename "$f") ==="
            cat "$f"
            echo ""
        done
    fi
    if [ "$found" -eq 0 ]; then
        echo "no mac registers"
    fi
} > "$OUTDIR/test-05-all-mac-registers.txt" 2>&1

# ============================================================================
# Test 6: Staging Driver Comparison
# ============================================================================
echo "[6/6] Check staging driver..."

clear_dmesg

# 1. Clean up IP and unload rtw88
ip addr flush dev "$IFACE" 2>/dev/null || true
RTW88_UNLOAD_LOG="$OUTDIR/test-06-rtw88-unload.txt"
ip link set "$IFACE" down >> "$RTW88_UNLOAD_LOG" 2>&1 || \
    echo "ip link set $IFACE down failed" >> "$RTW88_UNLOAD_LOG"
modprobe -r rtw_8723bs >> "$RTW88_UNLOAD_LOG" 2>&1 || \
    echo "rtw_8723bs unload failed" >> "$RTW88_UNLOAD_LOG"
sleep 2

# 2. Record existing managed interfaces before loading staging
BEFORE_STAGING=$(list_managed_ifaces || true)
BEFORE_STAGING_IW_DEV=$(iw dev 2>&1 || true)

# 3. Load Staging
STAGING_LOAD_LOG="$OUTDIR/test-06-staging-load.txt"
modprobe r8723bs > "$STAGING_LOAD_LOG" 2>&1 || \
    echo "Staging driver r8723bs failed to load" >> "$STAGING_LOAD_LOG"
sleep 3

# 4. Find the NEW interface spawned by staging
STAGING_IFACE=""
CURRENT_STAGING_IFACES=$(list_managed_ifaces || true)
CURRENT_STAGING_IW_DEV=$(iw dev 2>&1 || true)
{
    echo "Before staging load managed interfaces:"
    echo "$BEFORE_STAGING"
    echo ""
    echo "Before staging load iw dev:"
    echo "$BEFORE_STAGING_IW_DEV"
    echo ""
    echo "After staging load managed interfaces:"
    echo "$CURRENT_STAGING_IFACES"
    echo ""
    echo "After staging load iw dev:"
    echo "$CURRENT_STAGING_IW_DEV"
} > "$OUTDIR/test-06-staging-interfaces.txt"

for i in $CURRENT_STAGING_IFACES; do
    if ! iface_in_list "$i" "$BEFORE_STAGING"; then
        STAGING_IFACE="$i"
        break
    fi
done

echo "Detected staging interface: ${STAGING_IFACE:-NOT_FOUND}"

if [ -n "$STAGING_IFACE" ]; then
    STAGING_UP_LOG="$OUTDIR/test-06-staging-iface-up.txt"
    ip link set "$STAGING_IFACE" up > "$STAGING_UP_LOG" 2>&1 || \
        echo "ip link set $STAGING_IFACE up failed" >> "$STAGING_UP_LOG"
    sleep 2
    iw dev "$STAGING_IFACE" info > "$OUTDIR/test-06-staging-iw-info.log" 2>&1 || \
        echo "staging iw info failed" >> "$OUTDIR/test-06-staging-iw-info.log"

    dmesg > "$OUTDIR/test-06-staging-init.log"
    append_if_nonempty "$OUTDIR/test-06-staging-init.log" "ip link set output" "$STAGING_UP_LOG"

    clear_dmesg
    STAGING_CHAN6_CMD_LOG="$OUTDIR/test-06-staging-chan6-cmd.txt"
    iw dev "$STAGING_IFACE" set channel 6 > "$STAGING_CHAN6_CMD_LOG" 2>&1 || \
        echo "staging set channel failed" >> "$STAGING_CHAN6_CMD_LOG"
    sleep 10
    dmesg > "$OUTDIR/test-06-staging-chan6.log"
    append_if_nonempty "$OUTDIR/test-06-staging-chan6.log" "iw set channel output" "$STAGING_CHAN6_CMD_LOG"

    clear_dmesg
    timeout 60 iw dev "$STAGING_IFACE" scan > "$OUTDIR/test-06-staging-scan-output.txt" 2>&1 || \
        echo "staging scan failed or timed out" >> "$OUTDIR/test-06-staging-scan-output.txt"
    dmesg > "$OUTDIR/test-06-staging-scan-log.txt"

    {
        ls -la /proc/net/rtl8723bs 2>&1 || true
        ls -la /proc/net/rtl8723bs/* 2>&1 || true
    } > "$OUTDIR/test-06-staging-proc-tree.txt"

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
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-iw-info.log"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-init.log"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-chan6.log"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-scan-output.txt"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-scan-log.txt"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-proc-tree.txt"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-iface-up.txt"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-chan6-cmd.txt"
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
tar -czf "$TARFILE" "$OUTDIR"
echo ""
echo "Archive created: $TARFILE"
echo ""
echo "Upload $TARFILE for analysis."
