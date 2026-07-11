#!/bin/bash
# RTL8723BS rtw88 Driver Diagnostic Script
# Usage: sudo ./diagnose.sh [SSID] [PASSWORD]
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

for cmd in git make depmod modprobe insmod modinfo iw ip dmesg timeout tar ping awk grep readlink systemctl mktemp sha256sum uname flock; do
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
DIAGNOSE_SCRIPT="$REPO_ROOT/diagnose.sh"
STAGING_SRC="$REPO_ROOT/rtl8723bs-vendor"
STAGING_HOLD_DIR=""
STAGING_HOLD_PATH="${REPO_ROOT%/*}/.${REPO_ROOT##*/}.staging-hold"
DIAGNOSE_LOCK="${REPO_ROOT%/*}/.${REPO_ROOT##*/}.diagnose.lock"

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

restore_staging_tree() {
    local hold_dir
    local saved_tree

    if [ -z "$STAGING_HOLD_DIR" ]; then
        return
    fi

    hold_dir="$STAGING_HOLD_DIR"
    saved_tree="$hold_dir/rtl8723bs-vendor"
    if [ -e "$STAGING_SRC" ] || [ -L "$STAGING_SRC" ]; then
        if [ ! -e "$saved_tree" ]; then
            if ! rmdir "$hold_dir" 2>/dev/null; then
                echo "ERROR: Unexpected files in staging hold directory: $hold_dir" >&2
                return 1
            fi
            STAGING_HOLD_DIR=""
            return
        fi
        echo "ERROR: Cannot restore staging tree; destination exists: $STAGING_SRC" >&2
        return 1
    fi
    if [ ! -d "$saved_tree" ]; then
        echo "ERROR: Cannot restore staging tree; saved tree missing: $saved_tree" >&2
        return 1
    fi

    echo "+ restore $STAGING_SRC"
    if ! mv "$saved_tree" "$STAGING_SRC"; then
        echo "ERROR: Failed to restore staging tree from $saved_tree" >&2
        return 1
    fi
    if ! rmdir "$hold_dir"; then
        echo "ERROR: Staging tree restored, but hold directory is not empty: $hold_dir" >&2
        return 1
    fi
    STAGING_HOLD_DIR=""
}

recover_stale_staging_tree() {
    local saved_tree="$STAGING_HOLD_PATH/rtl8723bs-vendor"

    if [ -L "$STAGING_HOLD_PATH" ]; then
        echo "ERROR: Staging hold path must not be a symlink: $STAGING_HOLD_PATH" >&2
        return 1
    fi
    if [ ! -e "$STAGING_HOLD_PATH" ]; then
        return
    fi
    if [ ! -d "$STAGING_HOLD_PATH" ]; then
        echo "ERROR: Staging hold path is not a directory: $STAGING_HOLD_PATH" >&2
        return 1
    fi
    if { [ -e "$STAGING_SRC" ] || [ -L "$STAGING_SRC" ]; } &&
       [ ! -e "$saved_tree" ]; then
        rmdir "$STAGING_HOLD_PATH" 2>/dev/null || {
            echo "ERROR: Unexpected files in staging hold directory: $STAGING_HOLD_PATH" >&2
            return 1
        }
        return
    fi
    if [ -e "$STAGING_SRC" ] || [ -L "$STAGING_SRC" ]; then
        echo "ERROR: Both staging source and saved staging tree exist; refusing to overwrite either:" >&2
        echo "  $STAGING_SRC" >&2
        echo "  $saved_tree" >&2
        return 1
    fi
    if [ ! -d "$saved_tree" ]; then
        echo "ERROR: Saved staging tree missing from hold directory: $saved_tree" >&2
        return 1
    fi

    echo "Recovering staging tree left by an interrupted diagnostic run..."
    STAGING_HOLD_DIR="$STAGING_HOLD_PATH"
    restore_staging_tree
}

preserve_staging_tree_for_clean() {
    if [ ! -f "$STAGING_SRC/8723bs.ko" ]; then
        return
    fi

    if [ -e "$STAGING_HOLD_PATH" ] || [ -L "$STAGING_HOLD_PATH" ]; then
        echo "ERROR: Staging hold path already exists: $STAGING_HOLD_PATH" >&2
        return 1
    fi
    if ! mkdir "$STAGING_HOLD_PATH"; then
        echo "ERROR: Failed to create staging hold directory: $STAGING_HOLD_PATH" >&2
        return 1
    fi

    STAGING_HOLD_DIR="$STAGING_HOLD_PATH"
    echo "+ preserve $STAGING_SRC at $STAGING_HOLD_DIR"
    if ! mv "$STAGING_SRC" "$STAGING_HOLD_DIR/"; then
        echo "ERROR: Failed to preserve staging tree before clean" >&2
        rmdir "$STAGING_HOLD_DIR" || true
        STAGING_HOLD_DIR=""
        return 1
    fi
}

build_and_install_rtw88() {
    local rc

    preserve_staging_tree_for_clean || return 1

    echo "+ make clean"
    make clean || {
        rc=$?
        echo "ERROR: make clean failed"
        restore_staging_tree || true
        return "$rc"
    }
    restore_staging_tree || return 1

    echo "+ make KVER=$RUNNING_KVER"
    make -j"$(nproc)" KVER="$RUNNING_KVER" || return 1
    echo "+ make install KVER=$RUNNING_KVER"
    make install KVER="$RUNNING_KVER" || return 1
    echo "+ depmod -a $RUNNING_KVER"
    depmod -a "$RUNNING_KVER" || return 1
}

cleanup() {
    restore_staging_tree || true
    restore_services
}

run_under_diagnose_lock() {
    local rc

    if [ "${RTW88_DIAGNOSE_LOCKED_FOR:-}" = "$DIAGNOSE_LOCK" ]; then
        return
    fi
    if [ -L "$DIAGNOSE_LOCK" ]; then
        echo "ERROR: Diagnostic lock must not be a symlink: $DIAGNOSE_LOCK"
        return 1
    fi

    export RTW88_DIAGNOSE_LOCKED_FOR="$DIAGNOSE_LOCK"
    if flock -n -o -E 75 "$DIAGNOSE_LOCK" /bin/bash "$DIAGNOSE_SCRIPT" "$@"; then
        exit 0
    fi

    rc=$?
    if [ "$rc" -eq 75 ]; then
        echo "ERROR: Another diagnose.sh run is already active for $REPO_ROOT"
    fi
    exit "$rc"
}

trap cleanup EXIT

run_under_diagnose_lock "$@" || exit 1
recover_stale_staging_tree || exit 1

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
        echo ""
        echo "+ ip -4 route show"
        ip -4 route show || true
    } > "$out" 2>&1
}

dhcp_peer_from_log() {
    local dhcp_log="$1"

    awk '
        /DHCPOFFER .* from / { peer = $NF }
        /DHCPACK .* from / { peer = $NF }
        END { if (peer != "") print peer }
    ' "$dhcp_log"
}

choose_ping_target() {
    local iface="$1"
    local dhcp_log="$2"
    local target

    target=$(ip -4 route show default dev "$iface" 2>/dev/null |
             awk '$1 == "default" { print $3; exit }')
    if [ -z "$target" ]; then
        target=$(ip -4 route show 2>/dev/null |
                 awk -v iface="$iface" '$1 == "default" && $0 ~ (" dev " iface "( |$)") { print $3; exit }')
    fi
    if [ -z "$target" ] && [ -s "$dhcp_log" ]; then
        target=$(dhcp_peer_from_log "$dhcp_log")
    fi
    if [ -z "$target" ]; then
        target="192.168.1.1"
    fi

    printf '%s\n' "$target"
}

wpa_status_for_iface() {
    local iface="$1"

    if ! command -v wpa_cli >/dev/null 2>&1; then
        echo "wpa_cli: unavailable"
        return
    fi

    timeout 3 wpa_cli -i "$iface" status 2>&1 || true
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
    local wpa_conf
    local wpa_pidfile
    local connected=0
    local associated=0
    local authorized=0
    local i
    local link_snapshot
    local station_snapshot
    local wpa_status
    local wpa_state
    local ping_target
    local wpa_rc

    if [ -z "$AP_SSID" ]; then
        echo "No AP credentials provided - skipping $label connection test"
        echo "No SSID/password provided - skipping $label connection test" > "$connect_cmd_log"
        echo "No SSID/password provided - skipping $label connection test" > "$connect_link_log"
        echo "No SSID/password provided - skipping $label connection test" > "$connect_log"
        echo "No SSID/password provided - skipping $label state dump" > "$state_log"
        echo "No SSID/password provided - skipping $label WPA test" > "$wpa_log"
        echo "No SSID/password provided - skipping $label DHCP test" > "$dhcp_log"
        echo "No SSID/password provided - skipping $label ping test" > "$ping_log"
        return
    fi

    wpa_conf=$(mktemp "${TMPDIR:-/tmp}/rtw88-wpa-conf.XXXXXX")
    wpa_pidfile=$(mktemp "${TMPDIR:-/tmp}/rtw88-wpa-pid.XXXXXX")
    rm -f "$wpa_pidfile"

    clear_dmesg

    if ! write_wpa_config "$wpa_conf" "$AP_SSID" "$AP_PASSWORD"; then
        {
            echo "Failed to generate wpa_supplicant config"
            echo "SSID: $AP_SSID"
        } > "$connect_cmd_log"
        echo "wpa_supplicant config generation failed - skipping link check" > "$connect_link_log"
        echo "wpa_supplicant config generation failed - skipping state dump" > "$state_log"
        echo "wpa_supplicant config generation failed - skipping DHCP" > "$dhcp_log"
        echo "wpa_supplicant config generation failed - skipping ping" > "$ping_log"
        echo "wpa_supplicant config generation failed" > "$wpa_log"
        dmesg > "$connect_log"
        append_if_nonempty "$connect_log" "connect command output" "$connect_cmd_log"
        append_if_nonempty "$connect_log" "wpa_supplicant output" "$wpa_log"
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
        echo "+ timeout 30 wpa_supplicant -B -t -dd -i $iface -c <temp-conf> -P <pidfile> -f $wpa_log"
        if timeout 30 wpa_supplicant -B -t -dd -i "$iface" -c "$wpa_conf" -P "$wpa_pidfile" -f "$wpa_log"; then
            echo "wpa_supplicant start exit code: 0"
        else
            wpa_rc=$?
            echo "wpa_supplicant start exit code: $wpa_rc"
        fi

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
        echo "WPA did not complete - skipping DHCP" > "$dhcp_log"
    fi

    capture_connection_state "$iface" "$state_log"

    if [ "$connected" -eq 1 ]; then
        ping_target=$(choose_ping_target "$iface" "$dhcp_log")
        echo "Ping target: $ping_target" >> "$connect_cmd_log"

        timeout 10 ping -I "$iface" "$ping_target" -c 3 > "$ping_log" 2>&1 || \
            echo "ping failed or timed out" >> "$ping_log"
    else
        echo "WPA did not complete - skipping ping" > "$ping_log"
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
        candidate="8723bs"
        if [ -d "/sys/module/$candidate" ]; then
            echo "$candidate"
            return
        fi
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

    for dir in /proc/net/r8723bs /proc/net/rtl8723bs /proc/net/rtl8723b /proc/net/8723bs /proc/net/*8723*; do
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
        echo "Vendor interface: ${iface:-NOT_FOUND}"
        echo "Vendor netdev driver: $(driver_for_iface "$iface")"
        echo "Vendor netdev module: $(module_for_iface "$iface")"
        echo ""
        echo "Module directory /sys/module/8723bs:"
        if [ -d /sys/module/8723bs ]; then
            ls -la /sys/module/8723bs
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
        echo "Candidate vendor /proc/net directories:"
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
            echo "Vendor procfs registers not found"
            echo "Note: vendor module is 8723bs; SDIO driver name may be rtl8723bs."
            echo "This vendor tree may not expose /proc/net register dumps on this kernel."
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
    echo "+ modprobe -r 8723bs"
    modprobe -r 8723bs || true
} > "$PREBUILD_UNLOAD_LOG" 2>&1
sleep 1

echo "Building and installing current driver modules..."
if ! build_and_install_rtw88 > "$OUTDIR/test-00-build-install.log" 2>&1; then
    echo "ERROR: build/install failed; see $OUTDIR/test-00-build-install.log"
    exit 1
fi

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
    echo "+ modprobe -r 8723bs"
    modprobe -r 8723bs || true
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

run_wpa_connection_test "$IFACE" "$OUTDIR/test-04" "rtw88"

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
sleep 5
echo "[6/6] Check vendor reference driver..."

clear_dmesg

# 1. Clean up IP and unload rtw88
ip addr flush dev "$IFACE" 2>/dev/null || true
RTW88_UNLOAD_LOG="$OUTDIR/test-06-rtw88-unload.txt"
ip link set "$IFACE" down >> "$RTW88_UNLOAD_LOG" 2>&1 || \
    echo "ip link set $IFACE down failed" >> "$RTW88_UNLOAD_LOG"
unload_rtw88_stack >> "$RTW88_UNLOAD_LOG" 2>&1
sleep 2

# 2. Build vendor reference driver (skip only if .ko exists and is newer
#    than every source file — a stale .ko silently drops trace changes)
STAGING_BUILD_LOG="$OUTDIR/test-06-staging-build.log"
STAGING_KO=$(find "$STAGING_SRC" -name "8723bs.ko" | head -1)
STAGING_NEWER_SRC=""
if [ -n "$STAGING_KO" ]; then
    STAGING_NEWER_SRC=$(find "$STAGING_SRC" \( -name '*.c' -o -name '*.h' \) -newer "$STAGING_KO" -print -quit)
fi

if [ -n "$STAGING_KO" ] && [ -z "$STAGING_NEWER_SRC" ]; then
    echo "Vendor driver already built: $STAGING_KO — skipping build" > "$STAGING_BUILD_LOG"
    echo "Run: make -C /usr/src/linux-headers-7.0.11-zabbly+ M=\$(pwd)/rtl8723bs-vendor CONFIG_RTL8723BS=m clean" >> "$STAGING_BUILD_LOG"
    echo "  to force a rebuild after editing vendor source." >> "$STAGING_BUILD_LOG"
else
    {
        if [ -n "$STAGING_NEWER_SRC" ]; then
            echo "Vendor source newer than .ko ($STAGING_NEWER_SRC) — rebuilding"
        fi
        echo "+ make in vendor tree"
        make -j"$(nproc)" -C "/lib/modules/$RUNNING_KVER/build" M="$STAGING_SRC" CONFIG_RTL8723BS=m modules
    } > "$STAGING_BUILD_LOG" 2>&1 || {
        echo "ERROR: vendor build failed; see $STAGING_BUILD_LOG"
        exit 1
    }
    STAGING_KO=$(find "$STAGING_SRC" -name "8723bs.ko" | head -1)
fi

if [ -z "$STAGING_KO" ]; then
    echo "ERROR: vendor .ko not found" | tee -a "$OUTDIR/test-06-staging-load.txt"
    STAGING_IFACE=""
else
    # 3. Install locally built .ko so modprobe finds it
    MOD_EXTRA="/lib/modules/$RUNNING_KVER/extra"
    mkdir -p "$MOD_EXTRA"
    STAGING_MOD_DEST="$MOD_EXTRA/8723bs.ko"
    ln -sf "$STAGING_KO" "$STAGING_MOD_DEST"
    depmod -a "$RUNNING_KVER"

    # 4. Record existing interfaces, then unload prior instance and modprobe
    BEFORE_STAGING=$(list_managed_ifaces || true)
    BEFORE_STAGING_IW_DEV=$(iw dev 2>&1 || true)
    modprobe -r r8723bs 2>/dev/null || true
    modprobe -r 8723bs 2>/dev/null || true
    sleep 1
    STAGING_LOAD_LOG="$OUTDIR/test-06-staging-load.txt"
    echo "+ modprobe 8723bs (local vendor build with debug)" >> "$STAGING_LOAD_LOG"
    modprobe 8723bs >> "$STAGING_LOAD_LOG" 2>&1 || \
        echo "Vendor driver 8723bs failed to load" >> "$STAGING_LOAD_LOG"
fi
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

    run_wpa_connection_test "$STAGING_IFACE" "$OUTDIR/test-06-staging" "staging"

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
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-connect-cmd.txt"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-connect-link.txt"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-connect-log.txt"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-state.txt"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-wpa-supplicant.log"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-dhcp-output.txt"
    echo "Staging interface not found" > "$OUTDIR/test-06-staging-ping-output.txt"
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
