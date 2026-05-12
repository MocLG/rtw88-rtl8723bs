#!/bin/bash
# RTL8723BS rtw88 Driver Diagnostic Script
# Run this script as: sudo ./diagnose.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Run this script as root: sudo ./diagnose.sh"
    exit 1
fi

for cmd in git make depmod modprobe iw ip dmesg timeout tar ping awk grep readlink; do
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

driver_for_iface() {
    local iface="$1"
    local path

    if [ -z "$iface" ]; then
        echo "NOT_FOUND"
        return
    fi

    path=$(readlink -f "/sys/class/net/$iface/device/driver" 2>/dev/null || true)
    if [ -n "$path" ] && [ -e "$path" ]; then
        echo "${path##*/}"
    else
        echo "NOT_FOUND"
    fi
}

module_for_iface() {
    local iface="$1"
    local path
    local driver
    local candidate

    if [ -z "$iface" ]; then
        echo "NOT_FOUND"
        return
    fi

    path=$(readlink -f "/sys/class/net/$iface/device/driver/module" 2>/dev/null || true)
    if [ -n "$path" ] && [ -e "$path" ]; then
        echo "${path##*/}"
        return
    fi

    driver=$(driver_for_iface "$iface")
    if [ -n "$driver" ] && [ -d "/sys/module/$driver" ]; then
        echo "$driver"
        return
    fi

    if [ "$driver" = "rtl8723bs" ]; then
        candidate="r8723bs"
        if [ -d "/sys/module/$candidate" ]; then
            echo "$candidate"
            return
        fi
    fi

    echo "NOT_FOUND"
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

unused_monitor_iface() {
    local candidate

    for candidate in diagmon0 diagmon1 diagmon2; do
        if ! iw dev "$candidate" info >/dev/null 2>&1; then
            echo "$candidate"
            return
        fi
    done
}

run_channel6_listen() {
    local base_iface="$1"
    local cmd_log="$2"
    local dmesg_log="$3"
    local mon_iface
    local listen_iface="$base_iface"
    local created_monitor=0
    local switched_monitor=0
    local managed_fallback=0
    local channel_set=0

    mon_iface=$(unused_monitor_iface)
    : > "$cmd_log"

    {
        echo "Base interface: $base_iface"
        echo "Base driver: $(driver_for_iface "$base_iface")"
        echo "Base module: $(module_for_iface "$base_iface")"
        echo "Monitor candidate: ${mon_iface:-NONE_AVAILABLE}"
        echo "+ ip link set $base_iface down"
        ip link set "$base_iface" down || true

        if [ -n "$mon_iface" ]; then
            echo "+ iw dev $base_iface interface add $mon_iface type monitor"
            if iw dev "$base_iface" interface add "$mon_iface" type monitor; then
                created_monitor=1
                listen_iface="$mon_iface"
            else
                echo "temporary monitor interface add failed"
            fi
        fi

        if [ "$created_monitor" -eq 0 ]; then
            echo "+ iw dev $base_iface set type monitor"
            if iw dev "$base_iface" set type monitor; then
                switched_monitor=1
                listen_iface="$base_iface"
            else
                echo "switch to monitor mode failed; falling back to managed set-channel"
                managed_fallback=1
                listen_iface="$base_iface"
            fi
        fi

        if [ "$managed_fallback" -eq 1 ]; then
            echo "+ iw dev $listen_iface set channel 6"
            if iw dev "$listen_iface" set channel 6; then
                channel_set=1
            else
                echo "set channel failed"
            fi
            echo "+ ip link set $listen_iface up"
            ip link set "$listen_iface" up || true
        else
            echo "+ ip link set $listen_iface up"
            ip link set "$listen_iface" up || true
            echo "+ iw dev $listen_iface set channel 6"
            if iw dev "$listen_iface" set channel 6; then
                channel_set=1
            else
                echo "set channel failed"
            fi
        fi
        echo "listen interface: $listen_iface"
        echo "channel set: $channel_set"
    } >> "$cmd_log" 2>&1

    sleep 10
    dmesg > "$dmesg_log"

    {
        echo "+ restoring $base_iface"
        if [ "$created_monitor" -eq 1 ]; then
            echo "+ ip link set $listen_iface down"
            ip link set "$listen_iface" down || true
            echo "+ iw dev $listen_iface del"
            iw dev "$listen_iface" del || true
        elif [ "$switched_monitor" -eq 1 ]; then
            echo "+ ip link set $base_iface down"
            ip link set "$base_iface" down || true
            echo "+ iw dev $base_iface set type managed"
            iw dev "$base_iface" set type managed || true
        fi

        echo "+ ip link set $base_iface up"
        ip link set "$base_iface" up || true
    } >> "$cmd_log" 2>&1

    append_if_nonempty "$dmesg_log" "iw channel listen output" "$cmd_log"
}

staging_proc_dirs() {
    local dir

    for dir in /proc/net/r8723bs /proc/net/rtl8723bs /proc/net/rtl8723b /proc/net/*8723*; do
        if [ -d "$dir" ]; then
            echo "$dir"
        fi
    done
}

dump_staging_proc_tree() {
    local iface="$1"
    local out="$2"
    local proc_dirs
    local dir
    local child

    proc_dirs=$(staging_proc_dirs || true)
    {
        echo "Staging interface: ${iface:-NOT_FOUND}"
        echo "Staging netdev driver: $(driver_for_iface "$iface")"
        echo "Staging netdev module: $(module_for_iface "$iface")"
        echo ""
        echo "Module directory /sys/module/r8723bs:"
        if [ -d /sys/module/r8723bs ]; then
            ls -la /sys/module/r8723bs
        else
            echo "not found"
        fi
        echo ""
        echo "SDIO driver directory /sys/bus/sdio/drivers/rtl8723bs:"
        if [ -d /sys/bus/sdio/drivers/rtl8723bs ]; then
            ls -la /sys/bus/sdio/drivers/rtl8723bs
        else
            echo "not found"
        fi
        echo ""
        echo "Candidate staging /proc/net directories:"
        if [ -n "$proc_dirs" ]; then
            printf '%s\n' "$proc_dirs"
            while IFS= read -r dir; do
                [ -n "$dir" ] || continue
                echo ""
                echo "=== $dir ==="
                ls -la "$dir"
                for child in "$dir"/*; do
                    [ -e "$child" ] || continue
                    echo ""
                    echo "=== $child ==="
                    ls -la "$child"
                done
            done <<< "$proc_dirs"
        else
            echo "none found"
        fi
    } > "$out" 2>&1
}

dump_staging_registers() {
    local iface="$1"
    local out="$2"
    local proc_dirs
    local dir
    local reg
    local found=0

    proc_dirs=$(staging_proc_dirs || true)
    {
        if [ -n "$proc_dirs" ]; then
            while IFS= read -r dir; do
                [ -n "$dir" ] || continue
                for reg in "$dir/$iface/registers" "$dir"/*/registers "$dir"/registers; do
                    [ -f "$reg" ] || continue
                    found=1
                    echo "=== $reg ==="
                    cat "$reg"
                    echo ""
                done
            done <<< "$proc_dirs"
        fi

        if [ "$found" -eq 0 ]; then
            echo "Staging procfs registers not found"
            echo "Note: staging module is r8723bs; SDIO driver name may be rtl8723bs."
            echo "This staging tree appears not to expose /proc/net register dumps on this kernel."
        fi
    } > "$out" 2>&1
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
echo 0xc0 > /sys/module/rtw_core/parameters/debug_mask 2>> "$OUTDIR/test-00-setup.log" || \
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
    echo "Selected rtw88 driver: $(driver_for_iface "$IFACE")"
    echo "Selected rtw88 module: $(module_for_iface "$IFACE")"
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
run_channel6_listen "$IFACE" "$CHAN6_CMD_LOG" "$OUTDIR/test-02-chan6.log"

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
{
    echo ""
    echo "Selected staging interface: ${STAGING_IFACE:-NOT_FOUND}"
    echo "Selected staging driver: $(driver_for_iface "$STAGING_IFACE")"
    echo "Selected staging module: $(module_for_iface "$STAGING_IFACE")"
} >> "$OUTDIR/test-06-staging-interfaces.txt"

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
    run_channel6_listen "$STAGING_IFACE" "$STAGING_CHAN6_CMD_LOG" "$OUTDIR/test-06-staging-chan6.log"

    clear_dmesg
    timeout 60 iw dev "$STAGING_IFACE" scan > "$OUTDIR/test-06-staging-scan-output.txt" 2>&1 || \
        echo "staging scan failed or timed out" >> "$OUTDIR/test-06-staging-scan-output.txt"
    dmesg > "$OUTDIR/test-06-staging-scan-log.txt"

    dump_staging_proc_tree "$STAGING_IFACE" "$OUTDIR/test-06-staging-proc-tree.txt"
    dump_staging_registers "$STAGING_IFACE" "$OUTDIR/test-06-staging-regs.txt"
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
