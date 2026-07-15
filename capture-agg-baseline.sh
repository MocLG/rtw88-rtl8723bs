#!/bin/bash
# Capture the current RTL8723BS rtw88 BlockAck/aggregation baseline while
# driving a WiFi-bound TCP upload. Read-only apart from the test traffic and
# output directory; it does not build, reload or reconfigure the driver.

set -euo pipefail

SERVER="${SERVER:-192.168.88.4}"
DURATION="${DURATION:-30}"
INTERVAL="${INTERVAL:-0.2}"
IFACE="${IFACE:-}"
OUT="${OUT:-}"

usage() {
    cat <<EOF
usage: $0 [--interface IFACE] [--server IP] [--duration SEC]
          [--interval SEC] [--output DIR]

Defaults: server=$SERVER duration=$DURATION interval=$INTERVAL
The interface is auto-detected when omitted.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
    --interface) IFACE=$2; shift 2 ;;
    --server) SERVER=$2; shift 2 ;;
    --duration) DURATION=$2; shift 2 ;;
    --interval) INTERVAL=$2; shift 2 ;;
    --output) OUT=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

need() { command -v "$1" >/dev/null || { echo "missing command: $1" >&2; exit 1; }; }
for cmd in ip iw iperf3 awk grep sort date; do need "$cmd"; done

if [ -z "$IFACE" ]; then
    IFACE=$(iw dev 2>/dev/null | awk '$1 == "Interface" {print $2; exit}')
fi
[ -n "$IFACE" ] || { echo "no managed WiFi interface found" >&2; exit 1; }
iw dev "$IFACE" link | grep -q '^Connected to' || {
    echo "$IFACE is not associated" >&2
    exit 1
}
if ! iperf3 --help 2>&1 | grep -q -- '--bind-dev'; then
    echo "iperf3 lacks --bind-dev; refusing a potentially Ethernet-routed test" >&2
    exit 1
fi

mountpoint -q /sys/kernel/debug || {
    [ "$(id -u)" -eq 0 ] || {
        echo "debugfs is not mounted; rerun as root or mount it first" >&2
        exit 1
    }
    mount -t debugfs none /sys/kernel/debug
}

mapfile -t AGG_FILES < <(
    find /sys/kernel/debug/ieee80211 -type f \
        -path "*/netdev:$IFACE/stations/*/agg_status" 2>/dev/null | sort
)
[ "${#AGG_FILES[@]}" -gt 0 ] || {
    echo "agg_status unavailable for $IFACE (need CONFIG_MAC80211_DEBUGFS)" >&2
    exit 1
}

if [ -z "$OUT" ]; then
    OUT="agg-baseline-$(date +%Y%m%d-%H%M%S)"
fi
mkdir -p "$OUT"
OUT=$(readlink -f "$OUT")

capture_agg() {
    local label=$1 f
    {
        for f in "${AGG_FILES[@]}"; do
            echo "== $f =="
            cat "$f"
            echo
        done
    } > "$OUT/agg-$label.txt"
}

capture_station() {
    local label=$1
    iw dev "$IFACE" station dump > "$OUT/station-$label.txt"
}

capture_link() {
    local label=$1
    iw dev "$IFACE" link > "$OUT/link-$label.txt"
}

WIP=$(ip -4 -o addr show dev "$IFACE" | awk '{split($4,a,"/"); print a[1]; exit}')
[ -n "$WIP" ] || { echo "$IFACE has no IPv4 address" >&2; exit 1; }
ROUTE=$(ip route get "$SERVER" from "$WIP")
printf '%s\n' "$ROUTE" > "$OUT/route.txt"
printf '%s\n' "$ROUTE" | grep -q "dev $IFACE" || {
    echo "route to $SERVER is not on $IFACE: $ROUTE" >&2
    exit 1
}

TX0=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
RX0=$(cat "/sys/class/net/$IFACE/statistics/rx_bytes")

capture_link before
capture_station before
capture_agg before

dmesg > "$OUT/dmesg-before.txt" 2>/dev/null || true
printf 'Running %ss TCP upload to %s on %s...\n' "$DURATION" "$SERVER" "$IFACE"
iperf3 --bind-dev "$IFACE" -c "$SERVER" -t "$DURATION" -i "$INTERVAL" \
    | tee "$OUT/iperf.log"

# Capture immediately while the BA session is still warm.
capture_agg after
capture_station after
capture_link after
dmesg > "$OUT/dmesg-after.txt" 2>/dev/null || true

TX1=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
RX1=$(cat "/sys/class/net/$IFACE/statistics/rx_bytes")

{
    echo "RTL8723BS aggregation baseline"
    echo "interface=$IFACE server=$SERVER duration=$DURATION interval=$INTERVAL"
    echo "route=$ROUTE"
    echo "wifi_tx_bytes=$((TX1 - TX0)) wifi_rx_bytes=$((RX1 - RX0))"
    grep 'sender$' "$OUT/iperf.log" | tail -1
    echo
    echo "Before active BlockAck entries:"
    grep -E 'active|operational|TX.*[Yy]es|RX.*[Yy]es' "$OUT/agg-before.txt" || true
    echo
    echo "After active BlockAck entries:"
    grep -E 'active|operational|TX.*[Yy]es|RX.*[Yy]es' "$OUT/agg-after.txt" || true
} | tee "$OUT/SUMMARY.txt"

echo "Results: $OUT"
