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

load_rtw88() {
    local m
    modprobe mac80211 2>/dev/null
    for m in rtw_core rtw_sdio rtw_8723x rtw_8723b rtw_8723bs; do
        lsmod | grep -q "^$m " && continue
        if [ -f "$REPO_DIR/$m.ko" ]; then
            insmod "$REPO_DIR/$m.ko" 2>> "$OUT/run.log" || \
                log "insmod $m.ko failed (see run.log)"
        else
            modprobe "$m" 2>> "$OUT/run.log" || \
                log "modprobe $m failed (no $m.ko in $REPO_DIR either - run make first)"
        fi
    done
}

unload_rtw88() {
    local m
    for m in rtw_8723bs rtw_8723b rtw_8723x rtw_sdio rtw_core; do
        rmmod "$m" 2>/dev/null
    done
}

ensure_driver() {
    # Make sure the vendor reference driver is not holding the device.
    rmmod 8723bs 2>/dev/null

    detect_iface
    if [ -n "$IFACE" ]; then
        return 0
    fi

    # No interface but rtw88 modules present: either a partial stack
    # left by an interrupted run, or a stale system-installed build that
    # auto-loaded at boot without binding the device.  Either way, tear
    # it down and load the in-tree set.
    if lsmod | grep -q "^rtw_"; then
        log "rtw88 stack loaded without an interface - reloading in-tree set"
        unload_rtw88
        sleep 1
    fi

    log "no wireless interface - loading rtw88 modules"
    load_rtw88
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
        # The test machine's wired NIC can sit on the same subnet, in
        # which case DHCP does not add a second default route.  Fall
        # back to the DHCP-provided router, then to .1 of the subnet.
        GW="$(awk '/option routers/ { gsub(/[;,]/, "", $3); r = $3 } END { print r }' \
              /var/lib/dhcp/dhclient.leases 2>/dev/null)"
    fi
    if [ -z "$GW" ]; then
        GW="$(ip -4 -o addr show dev "$IFACE" | \
              awk '{ split($4, a, /[./]/); printf "%s.%s.%s.1", a[1], a[2], a[3]; exit }')"
    fi
    if ! ping -c1 -W3 -I "$IFACE" "$GW" > /dev/null 2>&1; then
        log "FATAL: gateway $GW not reachable over $IFACE"
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
    ping -I "$IFACE" -i 0.2 -w "$((SOAK_MINUTES * 60))" "$GW" > "$PING_LOG" 2>&1
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
        unload_rtw88
        sleep 2
        load_rtw88
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
    ping -I "$IFACE" -i 0.2 "$GW" > "$OUT/ping-during-scan.log" 2>&1 &
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
    log "phase 6: iperf3 against $IPERF_SERVER (bound to $IFACE)"
    kmsg "throughput start"
    local bind="--bind-dev $IFACE"
    if ! iperf3 --help 2>&1 | grep -q bind-dev; then
        bind="-B $(ip -4 -o addr show dev "$IFACE" | awk '{ split($4, a, "/"); print a[1]; exit }')"
    fi
    # snapshot byte counters so we can prove the traffic actually rode
    # the wireless interface (a wired NIC on the same subnet can answer
    # ARP for the wlan address and silently carry the test traffic)
    local wired wlan_rx0 wlan_tx0 wired_rx0 wired_tx0
    wired=$(ip -o link show up | awk -F': ' '$2 ~ /^(en|eth)/ { print $2; exit }')
    wlan_rx0=$(cat "/sys/class/net/$IFACE/statistics/rx_bytes")
    wlan_tx0=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
    if [ -n "$wired" ]; then
        wired_rx0=$(cat "/sys/class/net/$wired/statistics/rx_bytes")
        wired_tx0=$(cat "/sys/class/net/$wired/statistics/tx_bytes")
    fi
    iperf3 $bind -c "$IPERF_SERVER" -t 30       > "$OUT/iperf-tcp-up.log"   2>&1
    iperf3 $bind -c "$IPERF_SERVER" -t 30 -R    > "$OUT/iperf-tcp-down.log" 2>&1
    iperf3 $bind -c "$IPERF_SERVER" -u -b 40M -t 20    > "$OUT/iperf-udp-up.log"   2>&1
    iperf3 $bind -c "$IPERF_SERVER" -u -b 40M -t 20 -R > "$OUT/iperf-udp-down.log" 2>&1
    kmsg "throughput end"
    local wlan_rx wlan_tx
    wlan_rx=$(( $(cat "/sys/class/net/$IFACE/statistics/rx_bytes") - wlan_rx0 ))
    wlan_tx=$(( $(cat "/sys/class/net/$IFACE/statistics/tx_bytes") - wlan_tx0 ))
    log "throughput bytes on $IFACE: rx=$wlan_rx tx=$wlan_tx"
    if [ -n "$wired" ]; then
        local wired_rx wired_tx
        wired_rx=$(( $(cat "/sys/class/net/$wired/statistics/rx_bytes") - wired_rx0 ))
        wired_tx=$(( $(cat "/sys/class/net/$wired/statistics/tx_bytes") - wired_tx0 ))
        log "throughput bytes on $wired: rx=$wired_rx tx=$wired_tx"
        if [ "$wired_rx" -gt "$wlan_rx" ] || [ "$wired_tx" -gt "$wlan_tx" ]; then
            log "WARNING: wired NIC $wired carried more bytes than $IFACE - iperf results are NOT wireless numbers"
        fi
    fi
    grep -h "receiver\|sender" "$OUT"/iperf-*.log | tail -8 | tee -a "$OUT/run.log"
}

# count ERE matches in a file; always prints exactly one number
# (grep -c prints 0 itself on no match, so no || echo fallback that
# would emit a second line)
count() {
    local n
    n=$(grep -ciE "$1" "$2" 2>/dev/null)
    echo "${n:-0}"
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
        echo "deauth/disassoc/beacon-loss events: $(count "deauthenticated|disassociated|beacon loss" "$DMESG_LOG")"
        echo "rqpn_relatch (page pool regressed):  $(count "rqpn_relatch" "$DMESG_LOG")"
        echo "txfovw=1 at start (stale overflow):  $(count "txfovw=1" "$DMESG_LOG")"
        echo "sdio/dma errors:                     $(count "sdio.*(err|fail)|txdma_status 0x[1-9a-f]" "$DMESG_LOG")"
        # \bBUG so our own *_DEBUG trace lines do not count as kernel bugs
        echo "kernel warnings/oops:                $(count "WARNING:|\\bBUG:|Oops|Call Trace" "$DMESG_LOG")"
        echo
        echo "--- wpa events ---"
        echo "rekeys completed: $(count "Key negotiation completed" "$WPA_LOG")"
        echo "disconnect events: $(count "CTRL-EVENT-DISCONNECTED" "$WPA_LOG")"
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

# The wired NIC of the test machine may share the subnet with wlan0, so
# peers can resolve the wireless address to the wired MAC (ARP flux) and
# replies bypass the radio.  Strict-ARP sysctls would prevent that but
# also break access paths that depend on the flux (e.g. SSH reaching the
# wlan address over ethernet), so instead all test traffic is bound to
# $IFACE and the throughput phase compares interface byte counters to
# prove the bytes actually rode the radio.

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
