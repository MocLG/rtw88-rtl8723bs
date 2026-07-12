#!/bin/bash
# Stability / throughput test for the rtw88 RTL8723BS SDIO port.
#
# Usage (as root, from the repo directory on the test machine):
#   ./stability-test.sh <SSID> <PSK> [IPERF3_SERVER_IP]
#
# Tunables via environment:
#   SOAK_MINUTES=20    ping/monitor soak duration (OVERNIGHT=1 sets 480)
#   RECONNECTS=15      wpa disconnect/reconnect cycles
#   LINKCYCLES=5       ip link down/up cycles
#   RELOADS=3          full module unload/reload cycles (0 to skip)
#   SCANS=5            scans while traffic is flowing
#   OVERNIGHT=0        1 = long soak preset
#
# Everything is logged into stability-<date>-<githash>/ next to this
# script, with a SUMMARY.txt verdict at the end.  The connection is
# left up when the script finishes.

set -u

SSID="${1:-}"
PSK="${2:-}"
IPERF_SERVER="${3:-${IPERF_SERVER:-}}"

if [ -z "$SSID" ] || [ -z "$PSK" ]; then
    echo "usage: $0 <ssid> <psk> [iperf3-server-ip]" >&2
    exit 1
fi
if [ "$(id -u)" != 0 ]; then
    echo "run as root" >&2
    exit 1
fi

SOAK_MINUTES="${SOAK_MINUTES:-20}"
[ "${OVERNIGHT:-0}" = "1" ] && SOAK_MINUTES=480
RECONNECTS="${RECONNECTS:-15}"
LINKCYCLES="${LINKCYCLES:-5}"
RELOADS="${RELOADS:-3}"
SCANS="${SCANS:-5}"

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
GITHASH="$(git -C "$REPO_DIR" rev-parse --short=8 HEAD 2>/dev/null || echo nogit)"
OUT="$REPO_DIR/stability-$(date +%Y%m%d-%H%M%S)-$GITHASH"
mkdir -p "$OUT"

WPA_CTRL="/run/wpa_supplicant"
WPA_CONF="$OUT/wpa.conf"
WPA_LOG="$OUT/wpa.log"
DMESG_LOG="$OUT/dmesg.log"
PING_LOG="$OUT/ping-soak.log"
STA_LOG="$OUT/station-samples.csv"
SUMMARY="$OUT/SUMMARY.txt"

MON_PIDS=()
IFACE=""
GW=""

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/run.log"; }
kmsg() { echo "STABILITY_TEST: $*" > /dev/kmsg 2>/dev/null || true; }

cleanup() {
    local pid
    for pid in "${MON_PIDS[@]:-}"; do
        [ -n "$pid" ] && kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

require() {
    local missing=0 tool
    for tool in "$@"; do
        command -v "$tool" >/dev/null || { echo "missing tool: $tool" >&2; missing=1; }
    done
    return $missing
}

detect_iface() {
    IFACE="$(iw dev 2>/dev/null | awk '$1 == "Interface" { print $2; exit }')"
}

ensure_driver() {
    # Make sure the vendor reference driver is not holding the device.
    rmmod 8723bs 2>/dev/null

    detect_iface
    if [ -n "$IFACE" ]; then
        return 0
    fi

    log "no wireless interface - loading rtw88 modules"
    modprobe mac80211 2>/dev/null
    if [ -f "$REPO_DIR/rtw_core.ko" ]; then
        insmod "$REPO_DIR/rtw_core.ko" 2>/dev/null
        insmod "$REPO_DIR/rtw_sdio.ko" 2>/dev/null
        insmod "$REPO_DIR/rtw_8723bs.ko" 2>/dev/null
    else
        modprobe rtw_8723bs 2>/dev/null
    fi
    sleep 3
    detect_iface
    [ -n "$IFACE" ]
}

is_connected() {
    iw dev "$IFACE" link 2>/dev/null | grep -q "^Connected to"
}

wait_connected() {
    local timeout="${1:-25}" i=0
    while [ "$i" -lt "$timeout" ]; do
        is_connected && return 0
        sleep 1
        i=$((i + 1))
    done
    return 1
}

connect() {
    pkill -f "wpa_supplicant.*$IFACE" 2>/dev/null
    pkill dhclient 2>/dev/null
    sleep 1

    cat > "$WPA_CONF" <<EOF
ctrl_interface=$WPA_CTRL
network={
    ssid="$SSID"
    psk="$PSK"
}
EOF
    ip link set "$IFACE" up
    wpa_supplicant -B -t -i "$IFACE" -c "$WPA_CONF" -f "$WPA_LOG" || return 1

    if ! wait_connected 30; then
        log "FATAL: no association within 30s"
        return 1
    fi
    log "associated: $(iw dev "$IFACE" link | head -1)"

    timeout 25 dhclient -1 -v "$IFACE" >> "$OUT/dhclient.log" 2>&1
    GW="$(ip route show default dev "$IFACE" | awk '{ print $3; exit }')"
    if [ -z "$GW" ]; then
        log "FATAL: no default route after DHCP"
        return 1
    fi
    log "connected, gateway $GW, addr $(ip -4 -o addr show dev "$IFACE" | awk '{ print $4 }')"
}

start_monitors() {
    dmesg --follow >> "$DMESG_LOG" 2>&1 &
    MON_PIDS+=($!)

    (
        echo "epoch,signal_dbm,tx_bitrate,rx_bitrate,tx_retries,tx_failed,beacon_loss"
        while true; do
            iw dev "$IFACE" station dump 2>/dev/null | awk -v t="$(date +%s)" '
                /signal:/        { sig = $2 }
                /tx bitrate:/    { txr = $3 " " $4 }
                /rx bitrate:/    { rxr = $3 " " $4 }
                /tx retries:/    { ret = $3 }
                /tx failed:/     { fail = $3 }
                /beacon loss:/   { bl = $3 }
                END { if (sig != "") printf "%s,%s,%s,%s,%s,%s,%s\n", t, sig, txr, rxr, ret, fail, bl }
            '
            sleep 10
        done
    ) >> "$STA_LOG" &
    MON_PIDS+=($!)
}

phase_soak() {
    log "phase 1: ping soak for $SOAK_MINUTES min against $GW"
    kmsg "soak start"
    ping -i 0.2 -w "$((SOAK_MINUTES * 60))" "$GW" > "$PING_LOG" 2>&1
    kmsg "soak end"
    tail -2 "$PING_LOG" | tee -a "$OUT/run.log"
}

phase_reconnect() {
    log "phase 2: $RECONNECTS wpa disconnect/reconnect cycles"
    kmsg "reconnect stress start"
    local i ok=0 t0 t1
    for i in $(seq 1 "$RECONNECTS"); do
        wpa_cli -p "$WPA_CTRL" -i "$IFACE" disconnect > /dev/null 2>&1
        sleep 2
        t0=$(date +%s)
        wpa_cli -p "$WPA_CTRL" -i "$IFACE" reconnect > /dev/null 2>&1
        if wait_connected 25; then
            t1=$(date +%s)
            ok=$((ok + 1))
            log "  reconnect $i: OK in $((t1 - t0))s"
        else
            log "  reconnect $i: FAILED"
        fi
    done
    echo "$ok" > "$OUT/reconnect-ok.count"
    log "reconnect stress: $ok/$RECONNECTS OK"
    kmsg "reconnect stress end"
}

phase_linkcycle() {
    log "phase 3: $LINKCYCLES ip link down/up cycles"
    kmsg "linkcycle start"
    local i ok=0
    for i in $(seq 1 "$LINKCYCLES"); do
        ip link set "$IFACE" down
        sleep 3
        ip link set "$IFACE" up
        if wait_connected 30; then
            ok=$((ok + 1))
            log "  linkcycle $i: OK"
        else
            log "  linkcycle $i: FAILED"
        fi
    done
    echo "$ok" > "$OUT/linkcycle-ok.count"
    kmsg "linkcycle end"
}

phase_reload() {
    [ "$RELOADS" = 0 ] && return 0
    if [ ! -f "$REPO_DIR/rtw_core.ko" ]; then
        log "phase 4: skipped (no in-tree .ko files found)"
        return 0
    fi
    log "phase 4: $RELOADS module unload/reload cycles"
    kmsg "reload stress start"
    local i ok=0
    for i in $(seq 1 "$RELOADS"); do
        pkill -f "wpa_supplicant.*$IFACE" 2>/dev/null
        pkill dhclient 2>/dev/null
        sleep 1
        ip link set "$IFACE" down 2>/dev/null
        if ! timeout 30 rmmod rtw_8723bs; then
            log "  reload $i: FAILED (rtw_8723bs unload hang)"
            continue
        fi
        rmmod rtw_sdio rtw_core 2>/dev/null
        sleep 2
        insmod "$REPO_DIR/rtw_core.ko" && \
        insmod "$REPO_DIR/rtw_sdio.ko" && \
        insmod "$REPO_DIR/rtw_8723bs.ko"
        sleep 4
        detect_iface
        if [ -n "$IFACE" ] && connect; then
            ok=$((ok + 1))
            log "  reload $i: OK"
        else
            log "  reload $i: FAILED (no reconnect after reload)"
            ensure_driver && connect
        fi
    done
    echo "$ok" > "$OUT/reload-ok.count"
    kmsg "reload stress end"
}

phase_scan_under_traffic() {
    log "phase 5: $SCANS scans while traffic is flowing"
    kmsg "scan-under-traffic start"
    ping -i 0.2 "$GW" > "$OUT/ping-during-scan.log" 2>&1 &
    local ping_pid=$! i ok=0
    for i in $(seq 1 "$SCANS"); do
        iw dev "$IFACE" scan > /dev/null 2>&1
        sleep 3
        if is_connected; then
            ok=$((ok + 1))
            log "  scan $i: link survived"
        else
            log "  scan $i: LINK LOST"
            wait_connected 30 || { connect || true; }
        fi
    done
    kill "$ping_pid" 2>/dev/null
    echo "$ok" > "$OUT/scan-ok.count"
    kmsg "scan-under-traffic end"
}

phase_throughput() {
    if [ -z "$IPERF_SERVER" ]; then
        log "phase 6: skipped (no iperf3 server given)"
        return 0
    fi
    if ! command -v iperf3 > /dev/null; then
        log "phase 6: skipped (iperf3 not installed)"
        return 0
    fi
    log "phase 6: iperf3 against $IPERF_SERVER"
    kmsg "throughput start"
    iperf3 -c "$IPERF_SERVER" -t 30       > "$OUT/iperf-tcp-up.log"   2>&1
    iperf3 -c "$IPERF_SERVER" -t 30 -R    > "$OUT/iperf-tcp-down.log" 2>&1
    iperf3 -c "$IPERF_SERVER" -u -b 40M -t 20    > "$OUT/iperf-udp-up.log"   2>&1
    iperf3 -c "$IPERF_SERVER" -u -b 40M -t 20 -R > "$OUT/iperf-udp-down.log" 2>&1
    kmsg "throughput end"
    grep -h "receiver\|sender" "$OUT"/iperf-*.log | tail -8 | tee -a "$OUT/run.log"
}

summarize() {
    {
        echo "=== rtw88 8723bs stability test summary ==="
        echo "date: $(date)  commit: $GITHASH  iface: $IFACE  ssid: $SSID"
        echo
        echo "--- ping soak ---"
        grep -E "packets transmitted|rtt" "$PING_LOG" 2>/dev/null
        echo
        echo "--- stress results ---"
        echo "reconnects OK:  $(cat "$OUT/reconnect-ok.count" 2>/dev/null || echo skipped)/$RECONNECTS"
        echo "linkcycles OK:  $(cat "$OUT/linkcycle-ok.count" 2>/dev/null || echo skipped)/$LINKCYCLES"
        echo "reloads OK:     $(cat "$OUT/reload-ok.count" 2>/dev/null || echo skipped)/$RELOADS"
        echo "scans OK:       $(cat "$OUT/scan-ok.count" 2>/dev/null || echo skipped)/$SCANS"
        echo
        echo "--- link quality (last sample) ---"
        tail -1 "$STA_LOG" 2>/dev/null
        echo "max tx bitrate seen: $(awk -F, 'NR>1 { print $3 }' "$STA_LOG" 2>/dev/null | sort -n | tail -1)"
        echo
        echo "--- red flags in kernel log ---"
        local flags
        flags=$(grep -ciE "deauthenticated|disassociated|beacon loss" "$DMESG_LOG" 2>/dev/null || echo 0)
        echo "deauth/disassoc/beacon-loss events: $flags"
        echo "rqpn_relatch (page pool regressed):  $(grep -c rqpn_relatch "$DMESG_LOG" 2>/dev/null || echo 0)"
        echo "txfovw=1 at start (stale overflow):  $(grep -c "txfovw=1" "$DMESG_LOG" 2>/dev/null || echo 0)"
        echo "sdio/dma errors:                     $(grep -ciE "sdio.*(err|fail)|txdma_status 0x[1-9a-f]" "$DMESG_LOG" 2>/dev/null || echo 0)"
        echo "kernel warnings/oops:                $(grep -ciE "WARNING:|BUG:|Oops" "$DMESG_LOG" 2>/dev/null || echo 0)"
        echo
        echo "--- wpa events ---"
        echo "rekeys completed: $(grep -c "Key negotiation completed" "$WPA_LOG" 2>/dev/null || echo 0)"
        echo "disconnect events: $(grep -c "CTRL-EVENT-DISCONNECTED" "$WPA_LOG" 2>/dev/null || echo 0)"
        echo
        echo "full logs in: $OUT"
    } | tee "$SUMMARY"
}

# --- main -----------------------------------------------------------------

require iw wpa_supplicant wpa_cli dhclient ping awk || exit 1

log "output directory: $OUT"
kmsg "run start commit=$GITHASH"

ensure_driver || { log "FATAL: no wireless interface"; exit 1; }
log "interface: $IFACE"

connect || exit 1
start_monitors

phase_soak
phase_reconnect
phase_linkcycle
phase_reload
phase_scan_under_traffic

# make sure we are connected again before throughput and final verdict
is_connected || connect || true

phase_throughput

kmsg "run end"
summarize
log "done - connection left up"
