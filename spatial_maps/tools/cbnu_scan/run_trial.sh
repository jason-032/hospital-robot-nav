#!/bin/bash
# One headless trial of the CBNU scan-map site: launch, localisation preflight,
# then (unless SWEEP=0) a sweep of the 11 spaces with the ground-truth logger
# attached and gt_analyse.py run on the result.
# OVERLAY=<yaml> replaces config/nav2_params_cbnu_scan.yaml as the site parameter file; SWEEP_TIMEOUT=<wall s> cuts the sweep short.
# HEADLESS=false RVIZ=true opens the Gazebo window and RViz to watch the run.
# Usage: [SWEEP=0] [HEADLESS=false RVIZ=true] [OVERLAY=...] [SWEEP_TIMEOUT=...] [TRIALS_DIR=...] run_trial.sh <label>
S="$(cd "$(dirname "$0")" && pwd)"; PKG="$(cd "$S/../.." && pwd)"
TR="${TRIALS_DIR:-$HOME/sim_trials/$(date +%F)}"; LABEL=${1:?usage: run_trial.sh <label>}
mkdir -p "$TR"; LOG=$TR/$LABEL.launch.log; RES=$TR/$LABEL.result
[ -e "$RES" ] && { echo "[$LABEL] ABORT: $RES exists, pick a new label"; exit 1; }
: > "$RES"
say() { echo "$*" | tee -a "$RES"; }
source /opt/ros/jazzy/setup.bash; source "$HOME/ros2_ws/install/setup.bash"
export GZ_WORLD=cbnu_scan MAP_YAML=$PKG/maps/CBNU_scan.yaml
SX=$(awk '/^spawn_x:/{print $2}' "$PKG/config/cbnu_scan_site.yaml")
SY=$(awk '/^spawn_y:/{print $2}' "$PKG/config/cbnu_scan_site.yaml")

if pgrep -f "^gz sim" >/dev/null; then say "[$LABEL] ABORT: a simulation is already running"; exit 1; fi
say "[$LABEL] start=$(date '+%F %T') spawn=($SX, $SY)"
say "[$LABEL] commit=$(git -C "$PKG" rev-parse --short HEAD)$(git -C "$PKG" diff --quiet || echo '+uncommitted') params=${OVERLAY:-config/nav2_params_cbnu_scan.yaml}"
setsid ros2 launch spatial_maps spatial_maps_cbnu_scan.launch.py headless:=${HEADLESS:-true} rviz:=${RVIZ:-false} ${OVERLAY:+params_overlay:="$OVERLAY"} > "$LOG" 2>&1 &
LP=$!; T0=$(date +%s)
# The launch has its own process group, so an interrupt here would orphan it.
stop_sim() { kill -INT -- -$LP 2>/dev/null; for i in $(seq 1 40); do kill -0 $LP 2>/dev/null || break; sleep 1; done
             kill -KILL -- -$LP 2>/dev/null; pkill -KILL -f "^gz sim .*cbnu_scan.world" 2>/dev/null; }
trap 'say "[$LABEL] interrupted"; stop_sim; exit 130' INT TERM
until grep -q "Managed nodes are active" "$LOG"; do
  kill -0 $LP 2>/dev/null || { say "[$LABEL] ABORT: launch exited early: $(tail -1 "$LOG")"; exit 1; }
  [ $(( $(date +%s) - T0 )) -gt 300 ] && { say "[$LABEL] ABORT: nav2 not active after 300 s"; stop_sim; exit 1; }
  sleep 2
done
say "[$LABEL] nav2 active after $(( $(date +%s) - T0 )) s wall"; sleep 20

python3 "$PKG/tools/sim_checks/rt_sample.py" 30 "$LABEL" 2>&1 | grep "^\[" | tee -a "$RES"
say "[$LABEL] true pose: $(gz topic -e -t /world/cbnu_scan/dynamic_pose/info -n 1 2>/dev/null | grep -A4 'name: "my_robot"' | tr -s ' \n' ' ')"
python3 "$PKG/tools/field_checks/localisation_preflight.py" --sim-time --duration 15 > "$TR/$LABEL.preflight.txt" 2>&1
say "[$LABEL] preflight exit=$?: $(grep -E '^\[scan\] (.*hits within|best)|^WARN' "$TR/$LABEL.preflight.txt" | tr '\n' ' ')"

if [ "${SWEEP:-1}" != 0 ]; then
  python3 "$PKG/tools/sim_checks/gt_logger.py" "$LABEL" "$TR" > "$TR/$LABEL.gt_logger.log" 2>&1 &
  GP=$!; sleep 5
  before=$(ls -t "$HOME"/sweep_CBNU_scan_*.csv 2>/dev/null | head -1); ST=$(date +%s)
  timeout ${SWEEP_TIMEOUT:-7200} ros2 run spatial_maps sweep_test.py --ros-args -p floor:=CBNU_scan \
    -p semantic_json:="$PKG/config/cbnu_scan_rooms.json" -p robot_start_x:="$SX" -p robot_start_y:="$SY" \
    ${SWEEP_ARGS:-} > "$TR/$LABEL.sweep.log" 2>&1
  say "[$LABEL] sweep exit=$? wall=$(( ($(date +%s) - ST) / 60 )) min"
  kill -INT $GP 2>/dev/null; sleep 3; kill -KILL $GP 2>/dev/null
  pkill -f "gz topic -e -t /world/cbnu_scan/dynamic_pose/info" 2>/dev/null
  after=$(ls -t "$HOME"/sweep_CBNU_scan_*.csv 2>/dev/null | head -1)
  if [ -n "$after" ] && [ "$after" != "$before" ]; then
    mv "$after" "$TR/$LABEL.sweep.csv"; say "[$LABEL] sweep csv=$TR/$LABEL.sweep.csv"
    python3 "$PKG/tools/sim_checks/gt_analyse.py" "$LABEL" "$TR/$LABEL.sweep.csv" "$TR" 2>&1 | grep "^\[\|Error\|Traceback" | tee -a "$RES"
  else
    say "[$LABEL] sweep produced no new CSV"
  fi
fi

grep -qiE "segmentation fault|core dumped|process has died" "$LOG" && say "[$LABEL] WARNING: crash text in launch log"
say "[$LABEL] amcl_down=$(grep -c 'amcl IS DOWN' "$LOG")"
stop_sim; sleep 3
say "[$LABEL] done=$(date '+%F %T') leftover_gz=$(pgrep -fc '^gz sim')"
