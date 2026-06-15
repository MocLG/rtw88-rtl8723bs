#!/bin/bash
# RTL8723BS Warm Takeover Test
# Usage: sudo ./diagnose.sh [SSID] [PASSWORD]
#   SSID PASSWORD - optional WPA/WPA2-PSK AP credentials for connection test
#
# This script tests the warm takeover approach:
#   1. Build vendor + rtw88 modules
#   2. Load vendor with keep_alive=1  (firmware init, chip stays alive on rmmod)
#   3. Unload vendor                 (chip stays alive)
#   4. Load rtw88 with warm_start=1  (skip firmware download / chip init)
#   5. Run scan + connection test

set -e

AP_SSID="${1:-}"
AP_PASSWORD="${2:-}"

if [ "$#" -gt 0 ] && [ "$#" -ne 2 ]; then
    echo "Usage: sudo ./diagnose.sh [SSID PASSWORD]"
    echo "Provide both SSID and WPA/WPA2 password, or no arguments to skip connection."
    exit 1
fi

if [ "$#" -eq 2 ] && { [ -z "$AP_SSID" ] || [ -z "$AP_PASSWORD" ]; }; then
    echo "Usage: sudo ./diagnose.sh [SSID] [PASSWORD]"
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
REPO_ROOT=$(pwd -P)
VENDOR_SRC="$REPO_ROOT/rtl8723bs-vendor"

restore_services() {
    local svc

    if [ -z "$RESTORE_SERVICES" ]; then
        return
    fi

    echo ""
    for svc in $RESTORE_SERVICES; do
        echo "Restoring $svc..."
        systemctl start "$svc" || true
    done
}

trap restore_services EXIT

for svc in NetworkManager wpa_supplicant iwd; do
    if systemctl list-unit-files "${svc}.service" >/dev/null 2>&1 && systemctl is-active --quiet "$svc"; then
        echo "Stopping $svc to prevent interference..."
        systemctl stop "$svc" || true
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

echo "=== RTL8723BS Warm Takeover Test ==="
echo "Git commit: $GIT_HASH"
echo "Output directory: $OUTDIR"
echo ""

clear_dmesg() {
    dmesg -C 2>/dev/null || true
}

unload_rtw88_stack() {
    for mod in $RTW88_UNLOAD_MODULES; do
        modprobe -r "$mod" 2>/dev/null || true
    done
}

unload_everything() {
    unload_rtw88_stack
    modprobe -r 8723bs 2>/dev/null || true
    modprobe -r r8723bs 2>/dev/null || true
}

load_rtw88_stack() {
    local warm="$1"
    local mod
    local ko
    local total
    local count

    modprobe mac80211

    total=0
    for mod in $RTW88_BUILD_MODULES; do
        total=$((total + 1))
    done

    count=0
    for mod in $RTW88_BUILD_MODULES; do
        ko="./$mod.ko"
        count=$((count + 1))
        if [ ! -f "$ko" ]; then
            echo "ERROR: built module missing: $ko"
            return 1
        fi
        if [ -n "$warm" ] && [ "$count" -eq "$total" ]; then
            insmod "$ko" warm_start=1
        else
            insmod "$ko"
        fi
    done
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
            timeout 20 dhclient -v "$iface" 2>&1 || rc=$?
            echo "dhclient exit code: $rc"
        elif command -v dhcpcd >/dev/null 2>&1; then
            echo "+ timeout 25 dhcpcd -4 -t 20 $iface"
            timeout 25 dhcpcd -4 -t 20 "$iface" 2>&1 || rc=$?
            echo "dhcpcd exit code: $rc"
        elif command -v udhcpc >/dev/null 2>&1; then
            echo "+ timeout 20 udhcpc -i $iface -q -n"
            timeout 20 udhcpc -i "$iface" -q -n 2>&1 || rc=$?
            echo "udhcpc exit code: $rc"
        else
            echo "No supported DHCP client found"
        fi
    } > "$out" 2>&1
}

capture_connection_state() {
    local iface="$1"
    local out="$2"

    {
        echo "+ iw dev $iface link"
        iw dev "$iface" link || true
        echo ""
        echo "+ iw dev $iface station dump"
        iw dev "$iface" station dump || true
        echo ""
        echo "+ ip addr show dev $iface"
        ip addr show dev "$iface" || true
        echo ""
        echo "+ ip -4 route show dev $iface"
        ip -4 route show dev "$iface" || true
    } > "$out" 2>&1
}

dhcp_peer_from_log() {
    awk '
        /DHCPOFFER .* from / { peer = $NF }
        /DHCPACK .* from / { peer = $NF }
        END { if (peer != "") print peer }
    ' "$1"
}

choose_ping_target() {
    local iface="$1"
    local dhcp_log="$2"
    local target

    target=$(ip -4 route show default dev "$iface" 2>/dev/null |
             awk '$1 == "default" { print $3; exit }')
    [ -z "$target" ] && target=$(ip -4 route show 2>/dev/null |
             awk -v iface="$iface" '$1 == "default" && $0 ~ (" dev " iface "( |$)") { print $3; exit }')
    [ -z "$target" ] && [ -s "$dhcp_log" ] && target=$(dhcp_peer_from_log "$dhcp_log")
    [ -z "$target" ] && target="192.168.1.1"

    printf '%s\n' "$target"
}

wpa_status_for_iface() {
    if ! command -v wpa_cli >/dev/null 2>&1; then
        echo "wpa_cli: unavailable"
        return
    fi
    timeout 3 wpa_cli -i "$1" status 2>&1 || true
}

run_wpa_connection_test() {
    local iface="$1"
    local prefix="$2"
    local label="$3"
    local connect_cmd_log="${prefix}-connect-cmd.txt"
    local connect_link_log="${prefix}-connect-link.txt"
    local connect_log="${prefix}-connect-log.txt"
    local state_log="${prefix}-state.txt"
    local dhcp_log="${prefix}-dhcp-output.txt"
    local ping_log="${prefix}-ping-output.txt"
    local wpa_log="${prefix}-wpa-supplicant.log"
    local wpa_conf wpa_pidfile
    local connected=0 associated=0 authorized=0
    local i link_snapshot station_snapshot wpa_status wpa_state ping_target wpa_rc

    if [ -z "$AP_SSID" ]; then
        echo "No SSID/password provided - skipping $label connection test" > "$connect_cmd_log"
        echo "No SSID/password provided" > "$connect_link_log"
        echo "No SSID/password provided" > "$connect_log"
        echo "No SSID/password provided" > "$state_log"
        echo "No SSID/password provided" > "$wpa_log"
        echo "No SSID/password provided" > "$dhcp_log"
        echo "No SSID/password provided" > "$ping_log"
        return
    fi

    wpa_conf=$(mktemp "${TMPDIR:-/tmp}/rtw88-wpa-conf.XXXXXX")
    wpa_pidfile=$(mktemp "${TMPDIR:-/tmp}/rtw88-wpa-pid.XXXXXX")
    rm -f "$wpa_pidfile"

    clear_dmesg

    if ! write_wpa_config "$wpa_conf" "$AP_SSID" "$AP_PASSWORD"; then
        echo "Failed to generate wpa_supplicant config" > "$connect_cmd_log"
        echo "wpa_supplicant config generation failed" > "$connect_link_log"
        echo "wpa_supplicant config generation failed" > "$connect_log"
        echo "wpa_supplicant config generation failed" > "$state_log"
        echo "wpa_supplicant config generation failed" > "$wpa_log"
        echo "wpa_supplicant config generation failed" > "$dhcp_log"
        echo "wpa_supplicant config generation failed" > "$ping_log"
        dmesg > "$connect_log"
        rm -f "$wpa_conf" "$wpa_pidfile"
        return
    fi

    chmod 600 "$wpa_conf"
    : > "$connect_link_log"
    : > "$wpa_log"

    echo "Attempting $label WPA/WPA2 connection to '$AP_SSID' for up to 30 seconds..."

    {
        echo "Connecting to SSID: $AP_SSID"
        echo "Interface: $iface"
        echo "Driver: $(driver_for_iface "$iface")"
        echo "Module: $(module_for_iface "$iface")"
        echo "Auth: WPA/WPA2-PSK via wpa_supplicant"
        echo "+ ip link set $iface up"
        ip link set "$iface" up || true
        echo "+ timeout 30 wpa_supplicant -B -t -dd -i $iface -c <conf> -P <pidfile> -f $wpa_log"
        timeout 30 wpa_supplicant -B -t -dd -i "$iface" -c "$wpa_conf" -P "$wpa_pidfile" -f "$wpa_log" || \
            wpa_rc=$?
        echo "wpa_supplicant start exit code: ${wpa_rc:-0}"

        i=0
        while [ "$i" -lt 30 ]; do
            link_snapshot=$(timeout 3 iw dev "$iface" link 2>&1 || true)
            station_snapshot=$(timeout 3 iw dev "$iface" station dump 2>&1 || true)
            wpa_status=$(wpa_status_for_iface "$iface")
            wpa_state=$(printf '%s\n' "$wpa_status" |
                        awk -F= '$1 == "wpa_state" { print $2; exit }')
            {
                echo "=== link poll $i ==="
                echo "$link_snapshot"
                echo ""
                echo "=== station poll $i ==="
                echo "$station_snapshot"
                echo ""
                echo "=== wpa_cli poll $i ==="
                echo "$wpa_status"
                echo ""
            } >> "$connect_link_log"

            if printf '%s\n' "$link_snapshot" | grep -q '^Connected to '; then
                associated=1
            fi

            if [ "$wpa_state" = "COMPLETED" ] ||
               printf '%s\n' "$station_snapshot" | grep -q '^[[:space:]]*authorized:[[:space:]]*yes$'; then
                authorized=1
                connected=1
                break
            fi

            sleep 1
            i=$((i + 1))
        done

        echo "associated: $associated"
        echo "authorized: $authorized"
        echo "connected: $connected"
    } > "$connect_cmd_log" 2>&1

    if [ "$connected" -eq 1 ]; then
        echo "$label WPA completed; requesting DHCP..."
        run_dhcp_client "$iface" "$dhcp_log"
    else
        echo "$label WPA did not complete; continuing diagnostics."
        echo "WPA did not complete" > "$dhcp_log"
    fi

    capture_connection_state "$iface" "$state_log"

    if [ "$connected" -eq 1 ]; then
        ping_target=$(choose_ping_target "$iface" "$dhcp_log")
        echo "Ping target: $ping_target" >> "$connect_cmd_log"
        timeout 10 ping -I "$iface" "$ping_target" -c 3 > "$ping_log" 2>&1 || \
            echo "ping failed or timed out" >> "$ping_log"
    else
        echo "WPA did not complete" > "$ping_log"
    fi

    {
        echo "+ iw dev $iface disconnect"
        iw dev "$iface" disconnect || true
        if [ -s "$wpa_pidfile" ]; then
            echo "+ kill wpa_supplicant pid $(cat "$wpa_pidfile")"
            kill "$(cat "$wpa_pidfile")" 2>/dev/null || true
        fi
    } >> "$connect_cmd_log" 2>&1

    dmesg > "$connect_log"
    append_if_nonempty "$connect_log" "iw dev link output" "$connect_link_log"
    append_if_nonempty "$connect_log" "connection state output" "$state_log"
    append_if_nonempty "$connect_log" "wpa_supplicant output" "$wpa_log"
    append_if_nonempty "$connect_log" "connect command output" "$connect_cmd_log"
    append_if_nonempty "$connect_log" "DHCP output" "$dhcp_log"
    append_if_nonempty "$connect_log" "ping output" "$ping_log"

    rm -f "$wpa_conf" "$wpa_pidfile"
}

list_managed_ifaces() {
    iw dev | awk '
        $1 == "Interface" { iface = $2; next }
        $1 == "type" && iface != "" {
            if ($2 == "managed") print iface
            iface = ""
        }
    '
}

iface_in_list() {
    printf '%s\n' "$2" | grep -Fxq "$1"
}

driver_for_iface() {
    local iface="$1"
    local path

    [ -z "$iface" ] && { echo "NOT_FOUND"; return; }
    path=$(readlink -f "/sys/class/net/$iface/device/driver" 2>/dev/null || true)
    if [ -n "$path" ] && [ -e "$path" ]; then
        echo "${path##*/}"
    else
        echo "NOT_FOUND"
    fi
}

module_for_iface() {
    local iface="$1"
    local path driver

    [ -z "$iface" ] && { echo "NOT_FOUND"; return; }
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
    echo "NOT_FOUND"
}

# ============================================================================
# Step 0: Build both modules
# ============================================================================
echo "[0/5] Building modules..."

unload_everything
sleep 1

# Build rtw88
echo "Building rtw88..."
make -j"$(nproc)" KVER="$RUNNING_KVER" > "$OUTDIR/test-00-build-rtw88.log" 2>&1 || {
    echo "ERROR: rtw88 build failed; see $OUTDIR/test-00-build-rtw88.log"
    exit 1
}

# Build vendor (force rebuild to get keep_alive support)
echo "Building vendor (with keep_alive support)..."
{
    make -C "/lib/modules/$RUNNING_KVER/build" M="$VENDOR_SRC" CONFIG_RTL8723BS=m clean
    make -j"$(nproc)" -C "/lib/modules/$RUNNING_KVER/build" M="$VENDOR_SRC" CONFIG_RTL8723BS=m modules
} > "$OUTDIR/test-00-build-vendor.log" 2>&1 || {
    echo "ERROR: vendor build failed; see $OUTDIR/test-00-build-vendor.log"
    exit 1
}

VENDOR_KO=$(find "$VENDOR_SRC" -name "8723bs.ko" | head -1)
if [ -z "$VENDOR_KO" ]; then
    echo "ERROR: vendor .ko not found after build"
    exit 1
fi
echo "Vendor .ko: $VENDOR_KO"

# Install vendor .ko for modprobe
MOD_EXTRA="/lib/modules/$RUNNING_KVER/extra"
mkdir -p "$MOD_EXTRA"
ln -sf "$VENDOR_KO" "$MOD_EXTRA/8723bs.ko"
depmod -a "$RUNNING_KVER" 2>/dev/null || true

echo "Build complete."
echo ""

# ============================================================================
# Step 1: Load vendor with keep_alive=1
# ============================================================================
echo "[1/5] Load vendor with keep_alive=1..."

clear_dmesg
BEFORE_VENDOR=$(list_managed_ifaces || true)

echo "+ modprobe 8723bs keep_alive=1"
if ! modprobe 8723bs keep_alive=1 2>&1; then
    echo "modprobe failed, trying insmod directly..."
    if ! insmod "$VENDOR_KO" keep_alive=1 2>&1; then
        echo "ERROR: vendor failed to load (both modprobe and insmod)"
        dmesg > "$OUTDIR/test-01-vendor-load.log"
        exit 1
    fi
fi
sleep 6
dmesg > "$OUTDIR/test-01-vendor-load.log"

# Find vendor interface
VENDOR_IFACE=""
AFTER_VENDOR=$(list_managed_ifaces || true)
for i in $AFTER_VENDOR; do
    if ! iface_in_list "$i" "$BEFORE_VENDOR"; then
        VENDOR_IFACE="$i"
        break
    fi
done
echo "Vendor interface: ${VENDOR_IFACE:-NOT_FOUND}"

if [ -n "$VENDOR_IFACE" ]; then
    ip link set "$VENDOR_IFACE" up > /dev/null 2>&1 || true
    sleep 1
    iw dev "$VENDOR_IFACE" info > "$OUTDIR/test-01-vendor-iw-info.log" 2>&1 || true
    dmesg >> "$OUTDIR/test-01-vendor-load.log"
fi

# ============================================================================
# Step 2: Unload vendor (chip stays alive via keep_alive)
# ============================================================================
echo "[2/5] Unload vendor (chip stays alive)..."

if [ -n "$VENDOR_IFACE" ]; then
    ip link set "$VENDOR_IFACE" down 2>/dev/null || true
fi

clear_dmesg
echo "+ modprobe -r 8723bs (keep_alive=1 protects the chip)"
if ! modprobe -r 8723bs 2>&1; then
    echo "WARNING: vendor unload had errors (may be expected with keep_alive)"
fi
sleep 2
dmesg > "$OUTDIR/test-02-vendor-unload.log"

# ============================================================================
# Step 3: Load rtw88 with warm_start=1
# ============================================================================
echo "[3/5] Load rtw88 with warm_start=1..."

clear_dmesg
BEFORE_RTW88=$(list_managed_ifaces || true)

if ! load_rtw88_stack warm 2>&1; then
    echo "ERROR: rtw88 failed to load"
    dmesg > "$OUTDIR/test-03-rtw88-load.log"
    exit 1
fi
sleep 3
dmesg > "$OUTDIR/test-03-rtw88-load.log"

# Enable dynamic debug
mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
if [ -w /sys/kernel/debug/dynamic_debug/control ]; then
    echo "module rtw_* +p" > /sys/kernel/debug/dynamic_debug/control 2>/dev/null || true
fi
echo 0xc0 > /sys/module/rtw_core/parameters/debug_mask 2>/dev/null || true

# Find rtw88 interface
IFACE=""
AFTER_RTW88=$(list_managed_ifaces || true)
for i in $AFTER_RTW88; do
    if ! iface_in_list "$i" "$BEFORE_RTW88"; then
        IFACE="$i"
        break
    fi
done

if [ -z "$IFACE" ]; then
    echo "ERROR: No rtw88 interface found"
    dmesg >> "$OUTDIR/test-03-rtw88-load.log"
    exit 1
fi
echo "Warm rtw88 interface: $IFACE"
echo "Driver: $(driver_for_iface "$IFACE")"

# Bring up
ip link set "$IFACE" up > "$OUTDIR/test-03-iface-up.txt" 2>&1 || \
    echo "ip link set $IFACE up failed" >> "$OUTDIR/test-03-iface-up.txt"
sleep 2

iw dev "$IFACE" info > "$OUTDIR/test-03-iw-info.log" 2>&1 || true
dmesg >> "$OUTDIR/test-03-rtw88-load.log"

# ============================================================================
# Step 4: Scan
# ============================================================================
echo "[4/5] Warm takeover scan..."

clear_dmesg
timeout 60 iw dev "$IFACE" scan > "$OUTDIR/test-04-scan-output.txt" 2>&1 || \
    echo "scan failed or timed out" >> "$OUTDIR/test-04-scan-output.txt"
dmesg > "$OUTDIR/test-04-scan-log.txt"

# ============================================================================
# Step 5: Connection test
# ============================================================================
echo "[5/5] Warm takeover connection test..."

run_wpa_connection_test "$IFACE" "$OUTDIR/test-05" "warm-takeover"

# ============================================================================
# Summary
# ============================================================================
echo ""
echo "=== Warm Takeover Test Complete ==="
echo "Files saved to: $OUTDIR"
echo ""
ls -la "$OUTDIR"

tar -czf "$TARFILE" "$OUTDIR"
echo ""
echo "Archive created: $TARFILE"
echo "Upload $TARFILE for analysis."
