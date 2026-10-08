#!/usr/bin/env bash
# G1 (docs/MPC_PROTOTYPE.md): can the car computer run the MPC, and how long does a step take?
# Run on the Jetson from a checkout of control-mpc-v0 (needs offline/ and tests/):
#   bash tools/jetson_g1_check.sh
# Uses the python3 the policy node uses (override with PY=/path/to/python). No ROS, no
# vehicle commands; it does not need the vehicle enabled. Every log goes to one evidence
# directory, ~/ai4r-evidence/g1-<time>/ (override with EVIDENCE=...).
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
PY="${PY:-python3}"
EVIDENCE="${EVIDENCE:-$HOME/ai4r-evidence/g1-$(date +%Y%m%d-%H%M%S)}"

step() { printf '\n========== %s ==========\n' "$*"; }
fail() { printf '\nFAILED: %s\nLogs: %s\n' "$*" "$EVIDENCE"; echo "result:  FAIL ($*)" >> "$EVIDENCE/summary.txt"; exit 1; }
mkdir -p "$EVIDENCE"
cd "$REPO" || exit 1
: > "$EVIDENCE/summary.txt"

step "0. Source and machine"
{
  echo "date:     $(date -Is)"
  echo "host:     $(hostname)"
  echo "source:   $(git rev-parse HEAD 2>/dev/null || echo 'not a git checkout') $(git rev-parse --abbrev-ref HEAD 2>/dev/null)"
  [ -n "$(git status --porcelain 2>/dev/null)" ] && echo "WARNING:  uncommitted changes in $REPO"
  echo "policy:   sha256 $(sha256sum scripts/policy_node.py | cut -d' ' -f1)"
  echo "kernel:   $(uname -srm)"
  [ -r /proc/device-tree/model ] && echo "model:    $(tr -d '\0' < /proc/device-tree/model)"
  [ -r /etc/nv_tegra_release ] && echo "l4t:      $(head -1 /etc/nv_tegra_release)"
  command -v nvpmodel >/dev/null && echo "nvpmodel: $(nvpmodel -q 2>/dev/null | tr '\n' ' ')"
  echo "cpus:     $(nproc)  load: $(cut -d' ' -f1-3 /proc/loadavg)"
  for f in /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq; do
    [ -r "$f" ] && echo "$(basename "$f"): $(cat "$f")"
  done
} | tee "$EVIDENCE/machine.txt"
echo "Other busy processes (top 8 by CPU):" | tee -a "$EVIDENCE/machine.txt"
ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -9 | tee -a "$EVIDENCE/machine.txt"

step "1. Python dependencies ($PY)"
env -u PYTHONPATH "$PY" -c "
import sys, numpy, scipy, osqp, yaml
print('python', sys.version.split()[0], sys.executable)
print('numpy', numpy.__version__, 'scipy', scipy.__version__, 'osqp', getattr(osqp, '__version__', '?'), 'yaml', yaml.__version__)
" 2>&1 | tee "$EVIDENCE/dependencies.txt"
[ "${PIPESTATUS[0]}" -eq 0 ] || fail "missing numpy/scipy/osqp/yaml for $PY (try: $PY -m pip install --user osqp; if it cannot be installed, stop: MPC cannot run on this machine)"

step "2. Offline suites (correctness on this machine)"
suites_rc=0
for suite in "-m unittest tests.test_control tests.test_mpc tests.test_planning tests.test_estimation" \
             "offline/mpc_prediction_model/test_vehicle_model.py"; do
  echo "--- $suite"
  # shellcheck disable=SC2086
  env -u PYTHONPATH "$PY" -B $suite 2>&1 | tee -a "$EVIDENCE/offline-tests.log"
  [ "${PIPESTATUS[0]}" -eq 0 ] || suites_rc=1
done
grep -E "^Ran |^OK|^FAILED|MPC step time" "$EVIDENCE/offline-tests.log"

step "3. MPC step timing, N = 10 / 8 / 5"
env -u PYTHONPATH "$PY" -B tools/mpc_platform_timing.py --out "$EVIDENCE" 2>&1 | tee "$EVIDENCE/mpc-timing.txt"
timing_rc=${PIPESTATUS[0]}

step "Summary"
{
  grep -E "^(source|model|nvpmodel):" "$EVIDENCE/machine.txt"
  sed -n 2p "$EVIDENCE/dependencies.txt"
  echo "offline: $([ $suites_rc -eq 0 ] && echo PASS || echo FAIL)"
  echo "timing:  $([ "$timing_rc" -eq 0 ] && echo PASS || echo FAIL)"
  sed -n '/^MPC step time on this machine/,/^(ms;/p' "$EVIDENCE/mpc-timing.txt"
  grep "^OSQP time_limit" "$EVIDENCE/mpc-timing.txt"
  echo "logs:    $EVIDENCE"
} | tee -a "$EVIDENCE/summary.txt"
[ $suites_rc -eq 0 ] && [ "$timing_rc" -eq 0 ]
