#!/bin/bash
# Attaches gt_logger.py to every sweep run started by run_trial.sh, detaches when
# the run ends, and appends gt_analyse.py output to the run's .result file.
# Runs until the trial chain named in $1 exits. Passive: sends nothing to the sim.
# Usage: TRIALS_DIR=... gt_supervisor.sh <chain script path>
H="$(cd "$(dirname "$0")" && pwd)"
OUT="${TRIALS_DIR:-$HOME/sim_trials}"
CHAIN=$1
source /opt/ros/jazzy/setup.bash
source /home/jason/ros2_ws/install/setup.bash
log() { echo "$(date '+%F %T') gt: $*" | tee -a $OUT/overnight.log; }
sweep_label() { pgrep -af "sim_checks/run_trial.sh" | awk '$6 ~ /sweep/ {print $5}' | sort -u | head -1; }
running() { pgrep -af "sim_checks/run_trial.sh" | awk -v l="$1" '$5 == l' | grep -q .; }

log "supervisor started"
while pgrep -f "^bash $CHAIN" >/dev/null; do   # anchored: this script's own args contain $CHAIN
  L=$(sweep_label)
  if [ -n "$L" ]; then
    for f in $OUT/$L.gt_traj.csv $OUT/$L.gt_events.csv; do
      [ -s $f ] && mv $f $f.interrupted.$(date +%s)
    done
    python3 $H/gt_logger.py $L $OUT > $OUT/$L.gt_logger.log 2>&1 &
    GP=$!
    log "logger attached to $L (pid $GP)"
    while running $L; do sleep 5; done
    kill -INT $GP 2>/dev/null
    for i in $(seq 1 10); do kill -0 $GP 2>/dev/null || break; sleep 1; done
    kill -KILL $GP 2>/dev/null
    pkill -f "gz topic -e -t /world/1f/dynamic_pose/info" 2>/dev/null
    csv=$(grep -ho "sweep csv=.*" $OUT/$L.result 2>/dev/null | cut -d= -f2)
    if [ -n "$csv" ]; then
      python3 $H/gt_analyse.py $L $csv $OUT 2>&1 | grep "^\[\|Error\|Traceback" >> $OUT/$L.result
      log "analysed $L: $(grep -h 'truly within\|inside occupied' $OUT/$L.result | tr '\n' ' ')"
    else
      log "logger detached from $L (no sweep CSV: run interrupted or failed)"
    fi
  fi
  sleep 5
done
log "supervisor exiting: chain finished"
