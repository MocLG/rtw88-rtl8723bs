#!/bin/bash
# bt-coex-test.sh - exercise the RTL8723B WiFi/BT coexistence path.
#
# Runs WiFi traffic while a Bluetooth A2DP sink plays audio, and records
# what the coex mechanism did, whether the audio glitched, and how much
# WiFi throughput was lost to sharing the antenna.
#
# The RTL8723B is a combo WiFi+BT part on a shared antenna, so both the
# WiFi throughput drop and the audio smoothness are the things under test.
#
# Usage (root):
#   ./bt-coex-test.sh --server 192.168.88.4
#
# Preconditions:
#   - wlan0 associated (use ./wifi-route.sh up first)
#   - a Bluetooth speaker/headset already paired AND connected as an
#     A2DP sink (bluetoothctl connect <MAC>)
#
# NOT part of the tree - test helper, do not submit upstream.

set -u

SERVER="${SERVER:-192.168.88.4}"
DURATION="${DURATION:-60}"
IFACE="${IFACE:-}"
OUT="${OUT:-}"
AUDIO=""

usage() {
    cat <<EOF
usage: $0 [--interface IFACE] [--server IP] [--duration SEC] [--output DIR]
          [--audio FILE]

Defaults: server=$SERVER duration=$DURATION
The WiFi interface and the BT sink are auto-detected when omitted.
A test tone is generated when --audio is not given.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
    --interface) IFACE=$2; shift 2 ;;
    --server) SERVER=$2; shift 2 ;;
    --duration) DURATION=$2; shift 2 ;;
    --output) OUT=$2; shift 2 ;;
    --audio) AUDIO=$2; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

[ "$(id -u)" = 0 ] || { echo "run as root" >&2; exit 1; }
need() { command -v "$1" >/dev/null || { echo "missing command: $1" >&2; exit 1; }; }
for c in iw iperf3 bluetoothctl awk grep date; do need "$c"; done

say() { echo "[$(date +%H:%M:%S)] $*"; }

# ---------------------------------------------------------------- preflight

if [ -z "$IFACE" ]; then
    IFACE=$(iw dev 2>/dev/null | awk '$1 == "Interface" {print $2; exit}')
fi
[ -n "$IFACE" ] || { echo "no WiFi interface found" >&2; exit 1; }
iw dev "$IFACE" link | grep -q '^Connected to' || {
    echo "$IFACE is not associated (run ./wifi-route.sh up first)" >&2
    exit 1
}
iperf3 --help 2>&1 | grep -q -- '--bind-dev' || {
    echo "iperf3 lacks --bind-dev; refusing a possibly Ethernet-routed test" >&2
    exit 1
}

# a connected BT device is required; find the first one
BT_MAC=$(bluetoothctl devices Connected 2>/dev/null | awk '{print $2; exit}')
if [ -z "$BT_MAC" ]; then
    echo "no connected Bluetooth device." >&2
    echo "Pair and connect an A2DP speaker/headset first:" >&2
    echo "    bluetoothctl" >&2
    echo "    power on / scan on / pair <MAC> / trust <MAC> / connect <MAC>" >&2
    exit 1
fi
BT_NAME=$(bluetoothctl info "$BT_MAC" 2>/dev/null | awk -F': ' '/Name:/ {print $2; exit}')

# The audio stack is a per-user service. When running as root over ssh
# there is no login session, so point at root's runtime dir explicitly.
: "${XDG_RUNTIME_DIR:=/run/user/$(id -u)}"
export XDG_RUNTIME_DIR

# audio stack: prefer pipewire (gives us an underrun counter)
AUDIO_STACK="none"
if command -v pw-play >/dev/null && command -v pw-dump >/dev/null; then
    AUDIO_STACK="pipewire"
elif command -v paplay >/dev/null; then
    AUDIO_STACK="pulse"
fi
[ "$AUDIO_STACK" = "none" ] && {
    echo "no pipewire (pw-play) or pulseaudio (paplay) found" >&2
    exit 1
}

# Fail early and clearly if the server is not reachable, rather than
# producing a run with silent playback.
if ! pactl info >/dev/null 2>&1; then
    cat >&2 <<EOF
no audio server reachable (XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR).

Debian's pipewire units carry ConditionUser=!root, so as root start them
by hand first:

    export XDG_RUNTIME_DIR=/run/user/0
    pipewire & sleep 1
    wireplumber & sleep 1
    pipewire-pulse & sleep 2
    pactl info
EOF
    exit 1
fi

mountpoint -q /sys/kernel/debug || mount -t debugfs none /sys/kernel/debug 2>/dev/null
COEX=$(find /sys/kernel/debug/ieee80211 -name coex_info 2>/dev/null | head -1)

if [ -z "$OUT" ]; then OUT="btcoex-$(date +%Y%m%d-%H%M%S)"; fi
mkdir -p "$OUT"; OUT=$(readlink -f "$OUT")

# ------------------------------------------------------------- test audio

if [ -z "$AUDIO" ]; then
    AUDIO="$OUT/tone.wav"
    if command -v sox >/dev/null; then
        sox -n -r 44100 -c 2 "$AUDIO" synth "$DURATION" sine 440 sine 554 vol 0.3 \
            >/dev/null 2>&1
    elif command -v ffmpeg >/dev/null; then
        ffmpeg -f lavfi -i "sine=frequency=440:duration=$DURATION" \
            -ac 2 -ar 44100 "$AUDIO" >/dev/null 2>&1
    else
        AUDIO=$(find /usr/share/sounds -name '*.wav' -o -name '*.ogg' 2>/dev/null | head -1)
        [ -n "$AUDIO" ] || { echo "install sox or ffmpeg, or pass --audio FILE" >&2; exit 1; }
        say "no tone generator; looping $AUDIO"
    fi
fi

# -------------------------------------------------------------- helpers

underruns() {
    # pipewire exposes a per-node error/xrun counter
    if [ "$AUDIO_STACK" = "pipewire" ]; then
        pw-dump 2>/dev/null | awk '/"error"|"xrun"/ {n++} END {print n+0}'
    else
        echo "n/a"
    fi
}

snap_coex() {
    local label=$1
    [ -n "$COEX" ] && cat "$COEX" > "$OUT/coex-$label.txt" 2>/dev/null
    bluetoothctl info "$BT_MAC" > "$OUT/btinfo-$label.txt" 2>/dev/null
    iw dev "$IFACE" link > "$OUT/link-$label.txt" 2>/dev/null
    iw dev "$IFACE" station dump > "$OUT/station-$label.txt" 2>/dev/null
}

run_iperf() {  # $1 = label
    iperf3 --bind-dev "$IFACE" -c "$SERVER" -t "$DURATION" -i 1 -R \
        > "$OUT/iperf-$1.log" 2>&1
    awk '/receiver/ {print $7, $8}' "$OUT/iperf-$1.log" | tail -1
}

play_audio() {
    if [ "$AUDIO_STACK" = "pipewire" ]; then
        while :; do pw-play "$AUDIO" >/dev/null 2>&1 || break; done &
    else
        while :; do paplay "$AUDIO" >/dev/null 2>&1 || break; done &
    fi
    echo $!
}

# ------------------------------------------------------------------- run

say "interface : $IFACE -> $SERVER"
say "bluetooth : $BT_NAME [$BT_MAC]"
say "audio     : $AUDIO_STACK, $(basename "$AUDIO")"
say "coex_info : ${COEX:-<not available>}"
echo

dmesg -C 2>/dev/null || true
snap_coex before

# Phase 1: WiFi alone, BT connected but idle
say "phase 1/2: WiFi download for ${DURATION}s, Bluetooth idle"
BASE=$(run_iperf baseline)
say "  baseline: ${BASE:-no result}"
snap_coex wifi-only

# Phase 2: WiFi + BT audio
say "phase 2/2: WiFi download for ${DURATION}s while audio plays"
UR_BEFORE=$(underruns)
PLAY_PID=$(play_audio)
sleep 3   # let A2DP settle before loading the air

( while kill -0 "$PLAY_PID" 2>/dev/null; do
      [ -n "$COEX" ] && cat "$COEX" >> "$OUT/coex-samples.txt" 2>/dev/null
      echo "--- $(date +%H:%M:%S)" >> "$OUT/coex-samples.txt"
      sleep 5
  done ) &
SAMPLER=$!

WITH=$(run_iperf with-bt)

kill "$PLAY_PID" 2>/dev/null; pkill -P "$PLAY_PID" 2>/dev/null
kill "$SAMPLER" 2>/dev/null
wait 2>/dev/null
UR_AFTER=$(underruns)
say "  with BT audio: ${WITH:-no result}"

snap_coex after
dmesg > "$OUT/dmesg.txt" 2>/dev/null || true

# --------------------------------------------------------------- summary

BASE_V=$(echo "$BASE" | awk '{print $1}')
WITH_V=$(echo "$WITH" | awk '{print $1}')
DROP="n/a"
if [ -n "$BASE_V" ] && [ -n "$WITH_V" ]; then
    DROP=$(awk -v a="$BASE_V" -v b="$WITH_V" \
        'BEGIN {if (a > 0) printf "%.1f%%", (a-b)/a*100; else print "n/a"}')
fi

{
    echo "=== RTL8723B WiFi/BT coexistence test ==="
    echo "date      : $(date)"
    echo "interface : $IFACE  server: $SERVER  duration: ${DURATION}s x2"
    echo "bluetooth : $BT_NAME [$BT_MAC]"
    echo
    echo "--- WiFi throughput (download) ---"
    echo "BT idle        : ${BASE:-no result}"
    echo "BT audio active: ${WITH:-no result}"
    echo "throughput drop: $DROP"
    echo
    echo "--- audio ---"
    echo "stack          : $AUDIO_STACK"
    echo "xrun counter   : before=$UR_BEFORE after=$UR_AFTER"
    echo
    echo "--- coex ---"
    if [ -n "$COEX" ]; then
        echo "mechanism seen during the run:"
        grep -hE 'Mechanism|TDMA|Coex Table|BT Info|Policy' \
            "$OUT/coex-samples.txt" 2>/dev/null | sort -u | head -12
    else
        echo "coex_info not available (needs CONFIG_RTW88_DEBUGFS)"
    fi
    echo
    echo "--- kernel log ---"
    echo "coex messages    : $(grep -ci coex "$OUT/dmesg.txt" 2>/dev/null || echo 0)"
    echo "warnings/oops    : $(grep -cE 'WARNING|Oops|BUG:' "$OUT/dmesg.txt" 2>/dev/null || echo 0)"
    echo
    echo "NOTE: audio smoothness is partly subjective. Listen during phase 2"
    echo "and record whether playback stuttered, and roughly how often."
} | tee "$OUT/SUMMARY.txt"

echo
say "results: $OUT"
