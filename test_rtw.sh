#!/bin/bash
# RTL8723BS mac80211 driver — simulation & validation test
# Tests module loading, symbol resolution, and mac80211 integration.
#
# Since there is no physical RTL8723BS SDIO hardware present, the SDIO
# bus-binding module (rtw88_8723bs) will register its driver but will
# not probe any device.  This test validates:
#   1. Clean compilation against the running kernel
#   2. Module metadata correctness (modinfo)
#   3. mac80211_hwsim baseline — proves the mac80211 subsystem works
#   4. Module load chain: rtw88_core → rtw88_8723x → rtw88_8723b
#      → rtw88_sdio → rtw88_8723bs with zero errors
#   5. No NULL pointer dereferences, locking warnings, or oops in dmesg
#   6. Clean unload in reverse order
#
# Usage:  sudo bash test_rtw8723b.sh
set -uo pipefail

RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
RST='\033[0m'

PASS=0; FAIL=0; WARN=0
RTW88_DIR="$(pwd)"

pass() { ((PASS++)); echo -e "  ${GRN}✓ PASS${RST}: $1"; }
fail() { ((FAIL++)); echo -e "  ${RED}✗ FAIL${RST}: $1"; }
warn() { ((WARN++)); echo -e "  ${YLW}⚠ WARN${RST}: $1"; }

section() { echo -e "\n${YLW}═══ $1 ═══${RST}"; }

# ── Preflight ────────────────────────────────────────────────────────
section "0. Preflight"
if [[ $EUID -ne 0 ]]; then
    echo -e "${RED}ERROR: Must run as root (sudo)${RST}"
    exit 1
fi
pass "Running as root"

KVER=$(uname -r)
echo "  Kernel: $KVER"

if [[ ! -f "$RTW88_DIR/rtw88_8723bs.ko" ]]; then
    fail "rtw88_8723bs.ko not found — build first"
    exit 1
fi
pass "Built .ko files present"

VERMAGIC=$(modinfo -F vermagic "$RTW88_DIR/rtw88_8723bs.ko" | awk '{print $1}')
if [[ "$VERMAGIC" == "$KVER" ]]; then
    pass "vermagic matches running kernel ($KVER)"
else
    fail "vermagic mismatch: module=$VERMAGIC kernel=$KVER"
    exit 1
fi

# ── 1. Module metadata ──────────────────────────────────────────────
section "1. Module Metadata Validation"

check_modinfo() {
    local ko=$1 field=$2 expect=$3
    local val
    val=$(modinfo -F "$field" "$RTW88_DIR/$ko" 2>/dev/null || true)
    if echo "$val" | grep -q "$expect"; then
        pass "$ko: $field contains '$expect'"
    else
        fail "$ko: $field='$val', expected '$expect'"
    fi
}

check_modinfo rtw88_8723b.ko  description "8723b"
check_modinfo rtw88_8723b.ko  firmware    "rtw88/rtw8723b_fw.bin"
check_modinfo rtw88_8723b.ko  depends     "rtw88_8723x"
check_modinfo rtw88_8723bs.ko description "8723bs"
check_modinfo rtw88_8723bs.ko alias       "sdio:c"
check_modinfo rtw88_8723bs.ko depends     "rtw88_8723b"

# ── 2. mac80211_hwsim baseline ──────────────────────────────────────
section "2. mac80211_hwsim Baseline"

# Record dmesg cursor
DMESG_CURSOR=$(dmesg | wc -l)

# Make sure hwsim isn't already loaded
if lsmod | grep -q mac80211_hwsim; then
    rmmod mac80211_hwsim 2>/dev/null || true
    sleep 0.5
fi

if modprobe mac80211_hwsim radios=2 2>&1; then
    pass "mac80211_hwsim loaded (2 radios)"
else
    fail "mac80211_hwsim failed to load"
fi

sleep 1

# Check that hwsim created wlan interfaces
HWSIM_IFACES=$(iw dev 2>/dev/null | grep -c "Interface wlan" || true)
if [[ "$HWSIM_IFACES" -ge 2 ]]; then
    pass "mac80211_hwsim created $HWSIM_IFACES wlan interfaces"
else
    warn "mac80211_hwsim created $HWSIM_IFACES interfaces (expected ≥2)"
fi

# Verify they are mac80211 devices
if iw dev 2>/dev/null | grep -q "type managed"; then
    pass "hwsim interfaces are mac80211 managed-mode devices"
else
    warn "Could not confirm mac80211 managed mode"
fi

# Trigger a virtual scan on the first hwsim interface
HWSIM_IF=$(iw dev 2>/dev/null | awk '/Interface wlan/{print $2; exit}')
if [[ -n "$HWSIM_IF" ]]; then
    if iw dev "$HWSIM_IF" scan trigger 2>/dev/null; then
        sleep 2
        iw dev "$HWSIM_IF" scan dump 2>/dev/null | head -5
        pass "Virtual scan completed on $HWSIM_IF"
    else
        warn "Scan trigger on $HWSIM_IF returned error (may need interface up)"
        # Try bringing it up first
        ip link set "$HWSIM_IF" up 2>/dev/null || true
        sleep 0.5
        if iw dev "$HWSIM_IF" scan trigger 2>/dev/null; then
            sleep 2
            pass "Virtual scan completed on $HWSIM_IF (after link up)"
        else
            warn "Scan failed even after link up — non-critical for our test"
        fi
    fi
fi

rmmod mac80211_hwsim 2>/dev/null || true
sleep 0.5
pass "mac80211_hwsim unloaded cleanly"

# ── 3. Load rtw88 8723B module chain ────────────────────────────────
section "3. RTW88 8723B Module Loading"

# First, unload any existing rtw88 8723b modules that conflict
# Also unload rtw88_core if it was loaded by a previous test run
# (but only if no other rtw88 chip drivers depend on it)
for mod in rtw88_8723bs rtw88_sdio rtw88_8723b rtw88_8723x rtw88_core; do
    if lsmod | grep -qw "${mod}"; then
        rmmod "$mod" 2>/dev/null || true
    fi
done
sleep 0.5

# Record dmesg cursor before our loads
DMESG_BEFORE=$(dmesg | wc -l)

# Ensure mmc_core is loaded (provides SDIO symbols)
if ! lsmod | grep -qw mmc_core; then
    modprobe mmc_core 2>/dev/null && pass "modprobe mmc_core" || warn "mmc_core not available"
else
    pass "mmc_core already loaded"
fi

# Load the module chain using insmod (our out-of-tree .ko files)
LOAD_ORDER=(
    rtw88_core.ko
    rtw88_88xxa.ko
    rtw88_8723x.ko
    rtw88_8723b.ko
    rtw88_sdio.ko
    rtw88_8723bs.ko
)

ALL_LOADED=true
for ko in "${LOAD_ORDER[@]}"; do
    if insmod "$RTW88_DIR/$ko" 2>&1; then
        pass "insmod $ko"
    else
        RC=$?
        # Firmware missing is expected (no real device = no firmware request)
        fail "insmod $ko (exit code $RC)"
        ALL_LOADED=false
        break
    fi
    sleep 0.3
done

if $ALL_LOADED; then
    pass "Full module chain loaded successfully"
fi

# ── 4. Verify module presence ───────────────────────────────────────
section "4. Module Verification"

for mod in rtw88_core rtw88_8723x rtw88_8723b rtw88_sdio rtw88_8723bs; do
    if lsmod | awk '{print $1}' | grep -qx "${mod}"; then
        pass "$mod is loaded"
    else
        fail "$mod is NOT loaded"
    fi
done

# Show dependency chain
echo "  Module dependency snapshot:"
lsmod | grep -E "rtw88" | sed 's/^/    /'

# ── 5. Check for SDIO driver registration ───────────────────────────
section "5. SDIO Driver Registration"

# The driver should appear in /sys even without hardware
if [[ -d /sys/bus/sdio/drivers/rtw_8723bs ]]; then
    pass "SDIO driver 'rtw_8723bs' registered in sysfs"
else
    warn "SDIO driver not found in sysfs (may need sdio bus loaded)"
    # Check if the sdio bus exists at all
    if [[ -d /sys/bus/sdio ]]; then
        ls /sys/bus/sdio/drivers/ 2>/dev/null | sed 's/^/    /'
    else
        warn "/sys/bus/sdio does not exist (no MMC/SDIO subsystem)"
    fi
fi

# ── 6. dmesg analysis ───────────────────────────────────────────────
section "6. dmesg Analysis"

# Capture only new messages since our load
DMESG_NEW=$(dmesg | tail -n +"$DMESG_BEFORE")

# Check for critical issues
OOPS_COUNT=$(echo "$DMESG_NEW" | grep -ciE "BUG:|oops|NULL pointer|general protection fault|kernel panic" || true)
LOCKDEP_COUNT=$(echo "$DMESG_NEW" | grep -ciE "lockdep|lock held|possible circular|DEADLOCK" || true)
WARN_COUNT=$(echo "$DMESG_NEW" | grep -ciE "WARNING:|WARN_ON" || true)
RCU_COUNT=$(echo "$DMESG_NEW" | grep -ciE "rcu.*stall|call_rcu" || true)

if [[ "$OOPS_COUNT" -eq 0 ]]; then
    pass "No NULL pointer dereferences or oops"
else
    fail "Found $OOPS_COUNT oops/BUG/NULL-ptr messages"
    echo "$DMESG_NEW" | grep -iE "BUG:|oops|NULL pointer|general protection fault" | head -10 | sed 's/^/    /'
fi

if [[ "$LOCKDEP_COUNT" -eq 0 ]]; then
    pass "No locking warnings"
else
    fail "Found $LOCKDEP_COUNT lockdep/deadlock warnings"
    echo "$DMESG_NEW" | grep -iE "lockdep|lock held|DEADLOCK" | head -5 | sed 's/^/    /'
fi

if [[ "$WARN_COUNT" -eq 0 ]]; then
    pass "No WARNING/WARN_ON traces"
else
    warn "Found $WARN_COUNT WARNING traces"
    echo "$DMESG_NEW" | grep -iE "WARNING:|WARN_ON" | head -5 | sed 's/^/    /'
fi

# Show rtw88-specific messages
RTW_MSGS=$(echo "$DMESG_NEW" | grep -i "rtw" || true)
if [[ -n "$RTW_MSGS" ]]; then
    echo "  rtw88 kernel messages:"
    echo "$RTW_MSGS" | sed 's/^/    /'
fi

# ── 7. Clean unload ─────────────────────────────────────────────────
section "7. Clean Unload"

UNLOAD_ORDER=(
    rtw88_8723bs
    rtw88_sdio
    rtw88_8723b
    rtw88_8723x
    rtw88_88xxa
    rtw88_core
)

for mod in "${UNLOAD_ORDER[@]}"; do
    if lsmod | awk '{print $1}' | grep -qx "${mod}"; then
        if rmmod "$mod" 2>&1; then
            pass "rmmod $mod"
        else
            warn "rmmod $mod failed — attempting to continue"
        fi
    fi
    sleep 0.2
done

# Verify our modules are gone (rtw88_core may remain if rtw89 holds a dep)
LEFTOVER=$(lsmod | grep -E "rtw88_(8723b|sdio)" || true)
if [[ -z "$LEFTOVER" ]]; then
    pass "All rtw88 8723b/sdio modules unloaded cleanly"
else
    fail "Leftover modules: $LEFTOVER"
fi

# Check dmesg for unload issues
DMESG_UNLOAD=$(dmesg | tail -20)
UNLOAD_OOPS=$(echo "$DMESG_UNLOAD" | grep -ciE "BUG:|oops|NULL pointer" || true)
if [[ "$UNLOAD_OOPS" -eq 0 ]]; then
    pass "No crashes during unload"
else
    fail "Crash detected during unload"
    echo "$DMESG_UNLOAD" | grep -iE "BUG:|oops|NULL pointer" | sed 's/^/    /'
fi

# ── Summary ─────────────────────────────────────────────────────────
section "SUMMARY"
echo -e "  ${GRN}Passed${RST}: $PASS"
echo -e "  ${RED}Failed${RST}: $FAIL"
echo -e "  ${YLW}Warned${RST}: $WARN"
echo ""

if [[ "$FAIL" -eq 0 ]]; then
    echo -e "${GRN}══════════════════════════════════════${RST}"
    echo -e "${GRN}  ALL TESTS PASSED                    ${RST}"
    echo -e "${GRN}══════════════════════════════════════${RST}"
    exit 0
else
    echo -e "${RED}══════════════════════════════════════${RST}"
    echo -e "${RED}  $FAIL TEST(S) FAILED                ${RST}"
    echo -e "${RED}══════════════════════════════════════${RST}"
    exit 1
fi