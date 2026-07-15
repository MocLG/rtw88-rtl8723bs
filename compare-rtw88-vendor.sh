#!/bin/bash
# Compare the in-tree RTL8723BS rtw88 driver with the Realtek vendor driver.
# Builds both against the same kernel, loads them directly (no installation),
# and runs repeated WiFi-bound iperf3 TCP upload tests.

set -euo pipefail

REPO_DIR=$(cd "$(dirname "$0")" && pwd)
VENDOR_DIR="$REPO_DIR/rtl8723bs-vendor"
SSID="luka"
PSK="linux123"
SERVER="192.168.88.4"
DURATION=30
INTERVAL=0.2
RUNS=3
OOKLA_RUNS=1
RUN_OOKLA=1
KVER=$(uname -r)
KSRC=""
RTW_TREE=""
BUILD_ONLY=0
COOLDOWN=5
OUT=""
IFACE=""
WPA_PID=""
WPA_CONF=""
RESTORING=0
TESTING_STARTED=0
ROUTE_TABLE=200

usage() {
    cat <<EOF
usage: $0 [options]

Options:
  --ssid SSID             WiFi SSID (default: $SSID)
  --psk PSK               WiFi passphrase (default: project test PSK)
  --server IP             Wired iperf3 server (default: $SERVER)
  --duration SEC          Seconds per run (default: $DURATION)
  --interval SEC          iperf reporting interval (default: $INTERVAL)
  --runs N                iperf runs per driver (default: $RUNS)
  --ookla-runs N          Ookla Speedtest runs per driver (default: $OOKLA_RUNS)
  --no-ookla              Skip Ookla Speedtest
  --kernel-release REL    Kernel release to build for (default: uname -r)
  --kernel-src DIR        Kernel build/source directory
  --rtw-tree DIR          In-tree rtw kernel source (contains drivers/net/...)
  --output DIR            Result directory
  --build-only            Build both drivers, do not load or test
  -h, --help              Show this help

The normal test requires root. It never installs modules or reboots. The
Ethernet's main-table default route is left intact. Source-based policy routing
sends traffic bound to the WiFi address through WiFi, and a /32 server route
makes the LAN iperf target unambiguous. rtw88 is restored on exit.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
    --ssid) SSID=$2; shift 2 ;;
    --psk) PSK=$2; shift 2 ;;
    --server) SERVER=$2; shift 2 ;;
    --duration) DURATION=$2; shift 2 ;;
    --interval) INTERVAL=$2; shift 2 ;;
    --runs) RUNS=$2; shift 2 ;;
    --ookla-runs) OOKLA_RUNS=$2; shift 2 ;;
    --no-ookla) RUN_OOKLA=0; shift ;;
    --kernel-release) KVER=$2; shift 2 ;;
    --kernel-src) KSRC=$2; shift 2 ;;
    --rtw-tree) RTW_TREE=$2; shift 2 ;;
    --output) OUT=$2; shift 2 ;;
    --build-only) BUILD_ONLY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [ -z "$KSRC" ]; then
    KSRC=$(readlink -f "/lib/modules/$KVER/build" 2>/dev/null || true)
fi
if [ -z "$RTW_TREE" ]; then
    for candidate in "$REPO_DIR/../rtw" /root/hdd/rtw /mnt/hdd/rtw; do
        if [ -f "$candidate/drivers/net/wireless/realtek/rtw88/Makefile" ]; then
            RTW_TREE=$(readlink -f "$candidate")
            break
        fi
    done
fi

[ -d "$KSRC" ] || { echo "kernel source not found: $KSRC" >&2; exit 1; }
[ -n "$RTW_TREE" ] && [ -f "$RTW_TREE/drivers/net/wireless/realtek/rtw88/Makefile" ] || {
    echo "rtw kernel tree not found; pass --rtw-tree DIR" >&2
    exit 1
}
[ -d "$VENDOR_DIR" ] || { echo "vendor source not found: $VENDOR_DIR" >&2; exit 1; }

if [ -z "$OUT" ]; then
    OUT="$REPO_DIR/compare-$(date +%Y%m%d-%H%M%S)"
fi
mkdir -p "$OUT"
OUT=$(readlink -f "$OUT")
LOG="$OUT/run.log"
WPA_CONF="$OUT/wpa.conf"
WPA_PID="$OUT/wpa.pid"
chmod 700 "$OUT"

log() { printf '[%(%H:%M:%S)T] %s\n' -1 "$*" | tee -a "$LOG"; }
run() {
    printf '+ ' >> "$LOG"
    printf '%q ' "$@" >> "$LOG"
    printf '\n' >> "$LOG"
    "$@"
}
need() { command -v "$1" >/dev/null || { echo "missing command: $1" >&2; exit 1; }; }

for cmd in make modinfo awk sort grep; do
    need "$cmd"
done
if [ "$BUILD_ONLY" -eq 0 ]; then
    for cmd in ip iw iperf3 wpa_supplicant wpa_passphrase dhclient insmod rmmod modprobe; do
        need "$cmd"
    done
    if [ "$RUN_OOKLA" -eq 1 ]; then
        need speedtest
        if ! speedtest --help 2>&1 | grep -q -- '--interface'; then
            echo "speedtest is not the Ookla CLI with --interface support" >&2
            exit 1
        fi
    fi
    if ! iperf3 --help 2>&1 | grep -q -- '--bind-dev'; then
        echo "iperf3 lacks --bind-dev; refusing a potentially Ethernet-routed test" >&2
        exit 1
    fi
fi

RTW_MODDIR="$RTW_TREE/drivers/net/wireless/realtek/rtw88"
RTW_MODULES=(
    "$RTW_MODDIR/rtw88_core.ko"
    "$RTW_MODDIR/rtw88_8723x.ko"
    "$RTW_MODDIR/rtw88_8723b.ko"
    "$RTW_MODDIR/rtw88_sdio.ko"
    "$RTW_MODDIR/rtw88_8723bs.ko"
)
VENDOR_KO="$VENDOR_DIR/8723bs.ko"

build_drivers() {
    log "building rtw88 from $RTW_TREE for $KVER"
    run make -C "$KSRC" M="$RTW_MODDIR" clean \
        >"$OUT/build-rtw88-clean.log" 2>&1 || true
    run make -j"$(nproc)" -C "$KSRC" M="$RTW_MODDIR" \
        CONFIG_RTW88_CORE=m CONFIG_RTW88_SDIO=m CONFIG_RTW88_8723X=m \
        CONFIG_RTW88_8723B=m CONFIG_RTW88_8723BS=m modules \
        >"$OUT/build-rtw88.log" 2>&1

    local module
    for module in "${RTW_MODULES[@]}"; do
        [ -f "$module" ] || { log "missing rtw88 module: $module"; return 1; }
    done

    log "building vendor 8723bs from $VENDOR_DIR for $KVER"
    run make -C "$VENDOR_DIR" clean >"$OUT/build-vendor-clean.log" 2>&1 || true
    run make -j"$(nproc)" -C "$VENDOR_DIR" KVER="$KVER" KSRC="$KSRC" \
        CONFIG_WERROR= >"$OUT/build-vendor.log" 2>&1
    [ -f "$VENDOR_KO" ] || { log "vendor module was not produced"; return 1; }

    {
        echo "kernel_release=$KVER"
        echo "kernel_src=$KSRC"
        echo "rtw_tree=$RTW_TREE"
        echo "rtw88_built_vermagic=$(modinfo -F vermagic "${RTW_MODULES[0]}")"
        echo "rtw88_installed=$(modinfo -n rtw88_8723bs 2>/dev/null || true)"
        echo "rtw88_installed_vermagic=$(modinfo -F vermagic rtw88_8723bs 2>/dev/null || true)"
        echo "vendor_vermagic=$(modinfo -F vermagic "$VENDOR_KO")"
        echo "vendor_version=$(modinfo -F version "$VENDOR_KO")"
    } > "$OUT/build-info.txt"

    if ! grep -q "^rtw88_built_vermagic=$KVER " "$OUT/build-info.txt" ||
       ! grep -q "^vendor_vermagic=$KVER " "$OUT/build-info.txt"; then
        log "built module vermagic does not match $KVER; inspect build-info.txt"
        return 1
    fi
    if [ "$BUILD_ONLY" -eq 0 ] &&
       ! grep -q "^rtw88_installed_vermagic=$KVER " "$OUT/build-info.txt"; then
        log "installed rtw88_8723bs does not match $KVER; inspect build-info.txt"
        return 1
    fi
}

unload_wireless() {
    # A prior comparison intentionally leaves rtw88 connected. Discover that
    # interface before stopping userspace; IFACE is empty at script startup,
    # so otherwise the old wpa_supplicant survives and the new instance cannot
    # take control of wlan0.
    detect_iface 2>/dev/null || true
    stop_userspace
    local m
    for m in rtw88_8723bs rtw88_8723b rtw88_8723x rtw88_sdio rtw88_core \
             rtw_8723bs rtw_8723b rtw_8723x rtw_sdio rtw_core 8723bs; do
        rmmod "$m" 2>/dev/null || true
    done
    sleep 2
}

clear_wifi_routes() {
    if [ -n "${WIP:-}" ]; then
        while ip rule del from "$WIP" table "$ROUTE_TABLE" 2>/dev/null; do :; done
    fi
    ip route flush table "$ROUTE_TABLE" 2>/dev/null || true
    ip route del "$SERVER/32" 2>/dev/null || true
    ip route flush cache 2>/dev/null || true
}

stop_userspace() {
    clear_wifi_routes
    if [ -s "$WPA_PID" ]; then
        kill "$(cat "$WPA_PID")" 2>/dev/null || true
        rm -f "$WPA_PID"
    fi
    if [ -n "$IFACE" ]; then
        pkill -f "wpa_supplicant.*-i[[:space:]]*$IFACE" 2>/dev/null || true
        pkill -f "wpa_supplicant.*-i$IFACE" 2>/dev/null || true
    fi
}

load_rtw88() {
    unload_wireless
    run modprobe rtw88_8723bs
    sleep 3
}

load_vendor() {
    unload_wireless
    run modprobe cfg80211
    run insmod "$VENDOR_KO"
    sleep 3
}

detect_iface() {
    IFACE=$(iw dev 2>/dev/null | awk '$1 == "Interface" {print $2; exit}')
    [ -n "$IFACE" ] || { log "no managed wireless interface found"; return 1; }
}

is_connected() {
    iw dev "$IFACE" link 2>/dev/null | grep -q '^Connected to'
}

connect_wifi() {
    detect_iface
    log "connecting $IFACE to SSID $SSID"
    ip link set "$IFACE" up
    {
        echo 'ctrl_interface=/run/wpa_supplicant'
        wpa_passphrase "$SSID" "$PSK"
    } > "$WPA_CONF"
    chmod 600 "$WPA_CONF"
    if ! wpa_supplicant -B -P "$WPA_PID" -i "$IFACE" -c "$WPA_CONF" \
         -f "$OUT/wpa-${CURRENT_DRIVER}.log"; then
        log "$CURRENT_DRIVER wpa_supplicant failed to start"
        tail -30 "$OUT/wpa-${CURRENT_DRIVER}.log" | tee -a "$LOG"
        return 1
    fi

    local i
    for i in $(seq 1 40); do
        is_connected && break
        sleep 1
    done
    is_connected || { log "$CURRENT_DRIVER failed to associate"; return 1; }

    timeout 30 dhclient -1 -v -pf "$OUT/dhclient-${CURRENT_DRIVER}.pid" \
        -lf "$OUT/dhclient-${CURRENT_DRIVER}.leases" "$IFACE" \
        >"$OUT/dhclient-${CURRENT_DRIVER}.log" 2>&1 || true
    WIP=$(ip -4 -o addr show dev "$IFACE" | awk '{split($4,a,"/"); print a[1]; exit}')
    [ -n "$WIP" ] || { log "$IFACE did not receive IPv4"; return 1; }

    local gw subnet route internet_route

    # Save the DHCP gateway before removing its main-table default route.
    gw=$(ip route show default dev "$IFACE" | awk '{print $3; exit}')
    if [ -z "$gw" ]; then
        gw=$(awk '/option routers/ {gsub(/[;,]/, "", $3); r=$3} END {print r}' \
             "$OUT/dhclient-${CURRENT_DRIVER}.leases")
    fi
    [ -n "$gw" ] || { log "could not determine WiFi gateway"; return 1; }
    subnet=$(ip -4 -o route show dev "$IFACE" scope link | awk '{print $1; exit}')
    [ -n "$subnet" ] || subnet="${WIP%.*}.0/24"

    # Keep Ethernet's main default authoritative. Traffic explicitly bound to
    # the WiFi source address uses a private table with a WiFi default, which
    # is required by Ookla before it knows any destination addresses.
    ip route del default dev "$IFACE" 2>/dev/null || true
    while ip rule del from "$WIP" table "$ROUTE_TABLE" 2>/dev/null; do :; done
    ip route flush table "$ROUTE_TABLE" 2>/dev/null || true
    ip route add "$subnet" dev "$IFACE" src "$WIP" table "$ROUTE_TABLE"
    ip route add default via "$gw" dev "$IFACE" table "$ROUTE_TABLE"
    ip rule add from "$WIP" table "$ROUTE_TABLE"

    # Pin the local iperf server even if applications do not select the WiFi
    # source before route lookup (both NICs are on the same LAN on xtouch).
    ip route replace "$SERVER/32" dev "$IFACE" src "$WIP"
    ip route flush cache

    route=$(ip route get "$SERVER" from "$WIP")
    internet_route=$(ip route get 1.1.1.1 from "$WIP")
    {
        echo "server: $route"
        echo "internet: $internet_route"
        echo "table $ROUTE_TABLE:"
        ip route show table "$ROUTE_TABLE"
    } > "$OUT/route-${CURRENT_DRIVER}.txt"
    printf '%s\n' "$route" | grep -q "dev $IFACE" || {
        log "route to $SERVER is not on $IFACE: $route"
        return 1
    }
    printf '%s\n' "$internet_route" | grep -q "dev $IFACE" || {
        log "WiFi-bound Internet route is not on $IFACE: $internet_route"
        return 1
    }
    iw dev "$IFACE" link > "$OUT/link-${CURRENT_DRIVER}.txt"
}

station_snapshot() {
    iw dev "$IFACE" station dump 2>/dev/null || true
}

parse_iperf() {
    local file=$1
    awk '
    /sender$/ {
        for (i = 1; i <= NF; i++) {
            if ($i ~ /bits\/sec$/) {
                rate = $(i - 1); unit = $i; retr = $(i + 1)
                if (unit ~ /^G/) rate *= 1000
                else if (unit ~ /^K/) rate /= 1000
                printf "mbps=%.3f retr=%d\n", rate, retr
            }
        }
    }
    ' "$file" | tail -1
}

count_intervals() {
    local file=$1
    awk '
    /sec/ && /bits\/sec/ && $0 !~ /sender|receiver/ {
        for (i = 1; i <= NF; i++) {
            if ($i ~ /bits\/sec$/) {
                rate = $(i - 1); unit = $i
                mbps = rate
                if (unit ~ /^G/) mbps *= 1000
                else if (unit ~ /^K/) mbps /= 1000
                if (mbps == 0) zero++
                if (mbps < 1) sub1++
            }
        }
    }
    END {printf "zero=%d sub1=%d\n", zero + 0, sub1 + 0}
    ' "$file"
}

parse_ookla() {
    local file=$1
    awk -F: '
    /^[[:space:]]*Download:/ {
        value = $2; sub(/^[[:space:]]*/, "", value); split(value, a, /[[:space:]]+/)
        download = a[1]
    }
    /^[[:space:]]*Upload:/ {
        value = $2; sub(/^[[:space:]]*/, "", value); split(value, a, /[[:space:]]+/)
        upload = a[1]
    }
    /^[[:space:]]*Idle Latency:/ {
        value = $2; sub(/^[[:space:]]*/, "", value); split(value, a, /[[:space:]]+/)
        ping = a[1]
    }
    END {printf "download_mbps=%s upload_mbps=%s idle_ms=%s\n", download, upload, ping}
    ' "$file"
}

run_ookla_tests() {
    local driver=$1 dir="$OUT/$1" run_id file before after delta eth_before eth_after eth_delta

    [ "$RUN_OOKLA" -eq 1 ] || return 0
    for run_id in $(seq 1 "$OOKLA_RUNS"); do
        file="$dir/speedtest-$run_id.log"
        before=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
        if [ -e /sys/class/net/eth0/statistics/tx_bytes ]; then
            eth_before=$(cat /sys/class/net/eth0/statistics/tx_bytes)
        else
            eth_before=0
        fi
        log "$driver Ookla Speedtest run $run_id/$OOKLA_RUNS on $IFACE"
        if ! speedtest --accept-license --accept-gdpr --interface "$IFACE" \
             > "$file" 2>&1; then
            log "$driver Ookla run $run_id failed; see $file"
        fi
        after=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
        if [ -e /sys/class/net/eth0/statistics/tx_bytes ]; then
            eth_after=$(cat /sys/class/net/eth0/statistics/tx_bytes)
        else
            eth_after=$eth_before
        fi
        delta=$((after - before))
        eth_delta=$((eth_after - eth_before))
        {
            parse_ookla "$file"
            echo "wlan_tx_bytes=$delta eth_tx_bytes=$eth_delta"
        } > "$dir/speedtest-$run_id.metrics"
        if [ "$delta" -lt 1048576 ]; then
            log "$driver Ookla run $run_id did not carry enough WiFi bytes ($delta)"
            return 1
        fi
        sleep "$COOLDOWN"
    done
}

run_driver_tests() {
    local driver=$1 dir="$OUT/$1" run_id file before after delta eth_before eth_after eth_delta
    mkdir -p "$dir"
    dmesg > "$dir/dmesg-before.log"
    station_snapshot > "$dir/station-before.log"

    for run_id in $(seq 1 "$RUNS"); do
        file="$dir/iperf-$run_id.log"
        before=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
        if [ -e /sys/class/net/eth0/statistics/tx_bytes ]; then
            eth_before=$(cat /sys/class/net/eth0/statistics/tx_bytes)
        else
            eth_before=0
        fi
        log "$driver iperf run $run_id/$RUNS on $IFACE"
        if ! iperf3 --bind-dev "$IFACE" -c "$SERVER" -t "$DURATION" \
             -i "$INTERVAL" > "$file" 2>&1; then
            log "$driver run $run_id failed; see $file"
        fi
        after=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
        if [ -e /sys/class/net/eth0/statistics/tx_bytes ]; then
            eth_after=$(cat /sys/class/net/eth0/statistics/tx_bytes)
        else
            eth_after=$eth_before
        fi
        delta=$((after - before))
        eth_delta=$((eth_after - eth_before))
        {
            parse_iperf "$file"
            count_intervals "$file"
            echo "wlan_tx_bytes=$delta eth_tx_bytes=$eth_delta"
        } > "$dir/run-$run_id.metrics"
        if [ "$delta" -lt 1048576 ]; then
            log "$driver run $run_id did not carry enough WiFi bytes ($delta); aborting"
            return 1
        fi
        station_snapshot > "$dir/station-after-$run_id.log"
        sleep "$COOLDOWN"
    done
    run_ookla_tests "$driver"
    dmesg > "$dir/dmesg-after.log"
}

summarize_driver() {
    local driver=$1 dir="$OUT/$1" rates retrs zeros sub1 median ookla
    rates=$(awk -F'[ =]' '/mbps=/{print $2}' "$dir"/run-*.metrics 2>/dev/null | sort -n)
    retrs=$(awk -F'[ =]' '/mbps=/{s += $4} END {print s + 0}' "$dir"/run-*.metrics 2>/dev/null)
    zeros=$(awk -F'[ =]' '/zero=/{s += $2} END {print s + 0}' "$dir"/run-*.metrics)
    sub1=$(awk -F'[ =]' '/zero=/{s += $4} END {print s + 0}' "$dir"/run-*.metrics)
    median=$(printf '%s\n' "$rates" | awk '{a[NR]=$1} END {if (NR) print a[int((NR+1)/2)]; else print "n/a"}')
    printf '%s median_mbps=%s total_retr=%s zero_intervals=%s sub1_intervals=%s runs=[' \
        "$driver" "$median" "$retrs" "$zeros" "$sub1"
    printf '%s' "$rates" | paste -sd,
    printf ']'
    if [ "$RUN_OOKLA" -eq 1 ]; then
        ookla=$(awk '/download_mbps=/{print}' "$dir"/speedtest-*.metrics 2>/dev/null | paste -sd';')
        printf ' ookla=[%s]' "$ookla"
    fi
    printf '\n'
}

restore_rtw88() {
    [ "$BUILD_ONLY" -eq 0 ] || return 0
    [ "$TESTING_STARTED" -eq 1 ] || return 0
    [ "$RESTORING" -eq 0 ] || return 0
    RESTORING=1
    log "restoring rtw88"
    CURRENT_DRIVER=restore
    if load_rtw88 && connect_wifi; then
        log "rtw88 restored on $IFACE ($WIP)"
    else
        log "WARNING: automatic rtw88 restore failed"
    fi
    RESTORING=0
}

cleanup() {
    local rc=$?
    set +e
    restore_rtw88
    rm -f "$WPA_CONF" "$WPA_PID"
    exit "$rc"
}
trap cleanup EXIT INT TERM

log "output directory: $OUT"
log "kernel: $KVER, KSRC: $KSRC"
log "rtw tree: $RTW_TREE"
build_drivers

if [ "$BUILD_ONLY" -eq 1 ]; then
    log "build-only complete"
    exit 0
fi
[ "$(id -u)" -eq 0 ] || { echo "run as root (or use --build-only)" >&2; exit 1; }
TESTING_STARTED=1

CURRENT_DRIVER=rtw88
log "loading rtw88"
load_rtw88
connect_wifi
run_driver_tests rtw88
stop_userspace

CURRENT_DRIVER=vendor
log "loading vendor 8723bs"
load_vendor
connect_wifi
run_driver_tests vendor
stop_userspace

{
    echo "RTL8723BS rtw88 vs vendor comparison"
    echo "kernel=$KVER server=$SERVER duration=$DURATION interval=$INTERVAL runs=$RUNS ookla_runs=$OOKLA_RUNS"
    summarize_driver rtw88
    summarize_driver vendor
} | tee "$OUT/SUMMARY.txt"

restore_rtw88
TESTING_STARTED=0
rm -f "$WPA_CONF" "$WPA_PID"
trap - EXIT INT TERM
log "comparison complete: $OUT/SUMMARY.txt"
