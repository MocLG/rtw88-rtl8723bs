#!/bin/bash
# RTL8723BS rtw88 Driver Diagnostic Script
# Usage: sudo ./diagnose.sh [SSID PASSWORD]
#   SSID PASSWORD - optional WPA/WPA2-PSK AP credentials for connection test

set -e

AP_SSID="${1:-}"
AP_PASSWORD="${2:-}"

if [ "$#" -gt 0 ] && [ "$#" -ne 2 ]; then
    echo "Usage: sudo ./diagnose.sh [SSID PASSWORD]"
    echo "Provide both SSID and WPA/WPA2 password, or no arguments to skip connection."
    exit 1
fi

if [ "$#" -eq 2 ] && { [ -z "$AP_SSID" ] || [ -z "$AP_PASSWORD" ]; }; then
    echo "Usage: sudo ./diagnose.sh [SSID PASSWORD]"
    echo "SSID and password must both be non-empty."
    exit 1
fi

if [ "$#" -eq 2 ]; then
    if [[ "$AP_SSID" == *$'\n'* || "$AP_PASSWORD" == *$'\n'* ]]; then
        echo "ERROR: SSID/password must not contain newline characters."
        exit 1
    fi

    if ! { [ "${#AP_PASSWORD}" -ge 8 ] && [ "${#AP_PASSWORD}" -le 63 ]; } &&
       ! [[ "$AP_PASSWORD" =~ ^[[:xdigit:]]{64}$ ]]; then
        echo "ERROR: WPA/WPA2 password must be 8-63 chars, or a 64-char hex PSK."
        exit 1
    fi
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Run this script as root: sudo ./diagnose.sh"
    exit 1
fi

for cmd in git make depmod modprobe insmod modinfo iw ip dmesg timeout tar ping awk grep readlink systemctl mktemp sha256sum uname; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "ERROR: Required command not found: $cmd"
        exit 1
    fi
done

if [ -n "$AP_SSID" ]; then
    for cmd in wpa_supplicant; do
        if ! command -v "$cmd" >/dev/null 2>&1; then
            echo "ERROR: Required command not found for WPA connection test: $cmd"
            exit 1
        fi
    done
fi

RESTORE_SERVICES=""
for svc in NetworkManager wpa_supplicant iwd; do
    if systemctl list-unit-files "${svc}.service" >/dev/null 2>&1 && systemctl is-active --quiet "$svc"; then
        echo "Stopping and disabling $svc to prevent interference..."
        systemctl stop "$svc" || true
        systemctl disable "$svc" || true
        RESTORE_SERVICES="$RESTORE_SERVICES $svc"
    fi
done

GIT_HASH=$(git -c safe.directory="$PWD" rev-parse --short=8 HEAD 2>/dev/null) || {
    echo "ERROR: Unable to determine git commit hash"
    exit 1
}
RUNNING_KVER=$(uname -r)
RTW88_BUILD_MODULES="rtw_core rtw_8723x rtw_8723b rtw_sdio rtw_8723bs"
RTW88_UNLOAD_MODULES="rtw_8723bs rtw_sdio rtw_8723b rtw_8723x rtw_core"

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

unload_rtw88_stack() {
    local mod

    for mod in $RTW88_UNLOAD_MODULES; do
        echo "+ modprobe -r $mod"
        modprobe -r "$mod" || true
    done
}

load_built_rtw88_stack() {
    local mod
    local ko

    echo "+ modprobe mac80211"
    modprobe mac80211

    for mod in $RTW88_BUILD_MODULES; do
        ko="./$mod.ko"
        if [ ! -f "$ko" ]; then
            echo "ERROR: built module missing: $ko"
            return 1
        fi

        echo "+ insmod $ko"
        insmod "$ko"
    done
}

dump_module_provenance() {
    local out="$1"
    local title="$2"
    local mod
    local path

    {
        echo "=== $title ==="
        echo "Running kernel: $RUNNING_KVER"
        echo "Git commit: $GIT_HASH"
        echo "Build directory: $PWD"
        echo ""
        echo "Local built modules:"
        for mod in $RTW88_BUILD_MODULES; do
            if [ -f "$mod.ko" ]; then
                sha256sum "$mod.ko"
                modinfo "$mod.ko" | awk '/^(filename|version|vermagic|srcversion|depends):/ { print }'
            else
                echo "$mod.ko: missing"
            fi
            echo ""
        done

        echo "modprobe-resolved modules:"
        for mod in $RTW88_BUILD_MODULES; do
            path=$(modinfo -n "$mod" 2>/dev/null || true)
            if [ -n "$path" ] && [ -e "$path" ]; then
                echo "$mod: $path"
                sha256sum "$path"
                modinfo "$path" | awk '/^(filename|version|vermagic|srcversion|depends):/ { print }'
            else
                echo "$mod: NOT_FOUND"
            fi
            echo ""
        done

        echo "Loaded rtw modules:"
        awk '$1 ~ /^rtw_/ || $1 == "rtw_core" || $1 == "r8723bs" { print }' /proc/modules
        echo ""
        echo "Loaded rtw module sysfs:"
        for mod in $RTW88_BUILD_MODULES; do
            if [ -d "/sys/module/$mod" ]; then
                echo "$mod:"
                if [ -r "/sys/module/$mod/srcversion" ]; then
                    printf 'srcversion: '
                    cat "/sys/module/$mod/srcversion"
                fi
                if [ -r "/sys/module/$mod/refcnt" ]; then
                    printf 'refcnt: '
                    cat "/sys/module/$mod/refcnt"
                fi
                printf 'holders:'
                for path in "/sys/module/$mod"/holders/*; do
                    [ -e "$path" ] || continue
                    printf ' %s' "${path##*/}"
                done
                echo ""
            else
                echo "$mod: not loaded"
            fi
        done
        echo ""
    } >> "$out" 2>&1
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

wpa_conf_quote() {
    local value="$1"

    value=${value//\\/\\\\}
    value=${value//\"/\\\"}
    printf '%s' "$value"
}

write_wpa_config() {
    local out="$1"
    local ssid="$2"
    local psk="$3"

    {
        echo "ctrl_interface=/run/wpa_supplicant"
        echo "update_config=0"
        echo "network={"
        printf '\tssid="%s"\n' "$(wpa_conf_quote "$ssid")"
        if [[ "$psk" =~ ^[[:xdigit:]]{64}$ ]]; then
            printf '\tpsk=%s\n' "$psk"
        else
            printf '\tpsk="%s"\n' "$(wpa_conf_quote "$psk")"
        fi
        printf '\tkey_mgmt=WPA-PSK\n'
        echo "}"
    } > "$out"
}

run_dhcp_client() {
    local iface="$1"
    local out="$2"
    local rc=0

    {
        if command -v dhclient >/dev/null 2>&1; then
            echo "+ timeout 20 dhclient -v $iface"
            if timeout 20 dhclient -v "$iface"; then
                rc=0
            else
                rc=$?
            fi
            echo "dhclient exit code: $rc"
        elif command -v dhcpcd >/dev/null 2>&1; then
            echo "+ timeout 25 dhcpcd -4 -t 20 $iface"
            if timeout 25 dhcpcd -4 -t 20 "$iface"; then
                rc=0
            else
                rc=$?
            fi
            echo "dhcpcd exit code: $rc"
        elif command -v udhcpc >/dev/null 2>&1; then
            echo "+ timeout 20 udhcpc -i $iface -q -n"
            if timeout 20 udhcpc -i "$iface" -q -n; then
                rc=0
            else
                rc=$?
            fi
            echo "udhcpc exit code: $rc"
        else
            echo "No supported DHCP client found (tried dhclient, dhcpcd, udhcpc)"
        fi
    } > "$out" 2>&1
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

PREBUILD_UNLOAD_LOG="$OUTDIR/test-00-prebuild-unload.txt"
{
    unload_rtw88_stack
    echo "+ modprobe -r r8723bs"
    modprobe -r r8723bs || true
} > "$PREBUILD_UNLOAD_LOG" 2>&1
sleep 1

echo "Building and installing current driver modules..."
{
    echo "+ make clean"
    make clean
    echo "+ make KVER=$RUNNING_KVER"
    make -j"$(nproc)" KVER="$RUNNING_KVER"
    echo "+ make install KVER=$RUNNING_KVER"
    make install KVER="$RUNNING_KVER"
    echo "+ depmod -a $RUNNING_KVER"
    depmod -a "$RUNNING_KVER"
} > "$OUTDIR/test-00-build-install.log" 2>&1 || {
    echo "ERROR: build/install failed; see $OUTDIR/test-00-build-install.log"
    exit 1
}

: > "$OUTDIR/test-00-module-provenance.txt"
dump_module_provenance "$OUTDIR/test-00-module-provenance.txt" "after build/install"

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
{
    unload_rtw88_stack
    echo "+ modprobe -r r8723bs"
    modprobe -r r8723bs || true
} >> "$OUTDIR/test-00-setup.log" 2>&1
sleep 2

# 2. Record which managed interfaces exist BEFORE we load our driver.
EXISTING_IFACES=$(list_managed_ifaces || true)
EXISTING_IW_DEV=$(iw dev 2>&1 || true)

# 3. Load the new driver
load_built_rtw88_stack >> "$OUTDIR/test-00-setup.log" 2>&1
sleep 3
dump_module_provenance "$OUTDIR/test-00-module-provenance.txt" "after explicit built-module load"
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
# Test 4: Connection Attempt
# ============================================================================
echo "[4/6] Connection attempt..."

if [ -z "$AP_SSID" ]; then
    echo "No AP credentials provided - skipping connection test"
    echo "No SSID/password provided - skipping connection test" > "$OUTDIR/test-04-connect-cmd.txt"
    echo "No SSID/password provided - skipping connection test" > "$OUTDIR/test-04-connect-link.txt"
    echo "No SSID/password provided - skipping DHCP test" > "$OUTDIR/test-04-dhcp-output.txt"
    echo "No SSID/password provided - skipping ping test" > "$OUTDIR/test-04-ping-output.txt"
else
    CONNECT_CMD_LOG="$OUTDIR/test-04-connect-cmd.txt"
    WPA_LOG="$OUTDIR/test-04-wpa-supplicant.log"
    WPA_CONF=$(mktemp "${TMPDIR:-/tmp}/rtw88-wpa-conf.XXXXXX")
    WPA_PIDFILE=$(mktemp "${TMPDIR:-/tmp}/rtw88-wpa-pid.XXXXXX")
    rm -f "$WPA_PIDFILE"

    clear_dmesg

    if ! write_wpa_config "$WPA_CONF" "$AP_SSID" "$AP_PASSWORD"; then
        {
            echo "Failed to generate wpa_supplicant config"
            echo "SSID: $AP_SSID"
        } > "$CONNECT_CMD_LOG"
        echo "wpa_supplicant config generation failed - skipping link check" > "$OUTDIR/test-04-connect-link.txt"
        echo "wpa_supplicant config generation failed - skipping DHCP" > "$OUTDIR/test-04-dhcp-output.txt"
        echo "wpa_supplicant config generation failed - skipping ping" > "$OUTDIR/test-04-ping-output.txt"
        echo "wpa_supplicant config generation failed" > "$WPA_LOG"
        rm -f "$WPA_CONF" "$WPA_PIDFILE"
    else
        chmod 600 "$WPA_CONF"
        CONNECTED=0

        : > "$OUTDIR/test-04-connect-link.txt"
        : > "$WPA_LOG"

        echo "Attempting WPA/WPA2 connection to '$AP_SSID' for up to 20 seconds..."

        {
            echo "Connecting to SSID: $AP_SSID"
            echo "Interface: $IFACE"
            echo "Auth: WPA/WPA2-PSK via wpa_supplicant"
            echo "+ ip link set $IFACE up"
            ip link set "$IFACE" up || true
            echo "+ timeout 15 wpa_supplicant -B -i $IFACE -c <temp-conf> -P <pidfile> -f $WPA_LOG"
            if timeout 30 wpa_supplicant -B -i "$IFACE" -c "$WPA_CONF" -P "$WPA_PIDFILE" -f "$WPA_LOG"; then
                echo "wpa_supplicant start exit code: 0"
            else
                WPA_RC=$?
                echo "wpa_supplicant start exit code: $WPA_RC"
            fi

            i=0
            while [ "$i" -lt 20 ]; do
                LINK_SNAPSHOT=$(timeout 3 iw dev "$IFACE" link 2>&1 || true)
                {
                    echo "=== link poll $i ==="
                    echo "$LINK_SNAPSHOT"
                    echo ""
                } >> "$OUTDIR/test-04-connect-link.txt"

                if printf '%s\n' "$LINK_SNAPSHOT" | grep -q '^Connected to '; then
                    CONNECTED=1
                    break
                fi

                sleep 1
                i=$((i + 1))
            done

            echo "connected: $CONNECTED"
        } > "$CONNECT_CMD_LOG" 2>&1

        if [ "$CONNECTED" -eq 1 ]; then
            echo "Association succeeded; requesting DHCP..."
            run_dhcp_client "$IFACE" "$OUTDIR/test-04-dhcp-output.txt"
        else
            echo "Association failed; continuing diagnostics."
            echo "WPA association failed - skipping DHCP" > "$OUTDIR/test-04-dhcp-output.txt"
        fi

        PING_TARGET=$(ip route show dev "$IFACE" | awk '$1 == "default" { print $3; exit }')
        if [ -z "$PING_TARGET" ]; then
            PING_TARGET="192.168.1.1"
        fi

        timeout 10 ping -I "$IFACE" "$PING_TARGET" -c 3 > "$OUTDIR/test-04-ping-output.txt" 2>&1 || \
            echo "ping failed or timed out" >> "$OUTDIR/test-04-ping-output.txt"

        dmesg > "$OUTDIR/test-04-connect-log.txt"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "iw dev link output" "$OUTDIR/test-04-connect-link.txt"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "wpa_supplicant output" "$WPA_LOG"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "connect command output" "$CONNECT_CMD_LOG"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "DHCP output" "$OUTDIR/test-04-dhcp-output.txt"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "ping output" "$OUTDIR/test-04-ping-output.txt"

        {
            echo "+ iw dev $IFACE disconnect"
            iw dev "$IFACE" disconnect || true
            if [ -s "$WPA_PIDFILE" ]; then
                echo "+ kill wpa_supplicant pid $(cat "$WPA_PIDFILE")"
                kill "$(cat "$WPA_PIDFILE")" 2>/dev/null || true
            fi
        } >> "$CONNECT_CMD_LOG" 2>&1

        rm -f "$WPA_CONF" "$WPA_PIDFILE"
    fi

    if [ ! -s "$OUTDIR/test-04-connect-log.txt" ]; then
        dmesg > "$OUTDIR/test-04-connect-log.txt"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "iw dev link output" "$OUTDIR/test-04-connect-link.txt"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "wpa_supplicant output" "$WPA_LOG"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "connect command output" "$CONNECT_CMD_LOG"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "DHCP output" "$OUTDIR/test-04-dhcp-output.txt"
        append_if_nonempty "$OUTDIR/test-04-connect-log.txt" "ping output" "$OUTDIR/test-04-ping-output.txt"
    fi
fi

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
unload_rtw88_stack >> "$RTW88_UNLOAD_LOG" 2>&1
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

if [ -n "$RESTORE_SERVICES" ]; then
    echo ""
    for svc in $RESTORE_SERVICES; do
        echo "Restoring $svc..."
        systemctl enable "$svc" || true
        systemctl start "$svc" || true
    done
fi
