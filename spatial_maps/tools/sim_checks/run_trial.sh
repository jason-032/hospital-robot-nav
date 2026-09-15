#!/bin/bash
# Usage: run_trial.sh <boxes|mesh> <label> <tests: comma list of rt,scan,contact,sweep>
S="$(cd "$(dirname "$0")" && pwd)"
TR="${TRIALS_DIR:-$HOME/sim_trials}"
MODE=$1; LABEL=$2; TESTS=$3
WORLD=/home/jason/ros2_ws/src/spatial_maps/worlds/1f.world
mkdir -p $TR
LOG=$TR/$LABEL.launch.log; RES=$TR/$LABEL.result
: > $RES
say() { echo "$*" | tee -a $RES; }

source /opt/ros/jazzy/setup.bash
source /home/jason/ros2_ws/install/setup.bash
set -u
cd $S

if pgrep -f "^gz sim" >/dev/null || pgrep -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch" >/dev/null; then
  say "[$LABEL] ABORT: a simulation is already running"; exit 1
fi

cp $S/worlds/1f_$MODE.world $WORLD
say "[$LABEL] mode=$MODE world_md5=$(md5sum < $WORLD | cut -c1-8) start=$(date '+%F %T')"

setsid ros2 launch spatial_maps spatial_maps_1f.launch.py headless:=true > $LOG 2>&1 &
LP=$!
T0=$(date +%s)
until grep -q "Managed nodes are active" $LOG; do
  if ! kill -0 $LP 2>/dev/null; then say "[$LABEL] launch exited early"; break; fi
  if [ $(( $(date +%s) - T0 )) -gt 300 ]; then say "[$LABEL] nav2 not active after 300 s"; break; fi
  sleep 2
done
say "[$LABEL] nav2 active after $(( $(date +%s) - T0 )) s wall"
sleep 20

for t in ${TESTS//,/ }; do
  case $t in
    rt)      python3 rt_sample.py 60 $LABEL 2>&1 | grep "^\[" | tee -a $RES ;;
    scan)    timeout 300 python3 scan_check.py $LABEL 2>&1 | grep "^\[" | tee -a $RES ;;
    contact) timeout 900 python3 contact_test.py $LABEL ${DRIVE_S:-35} 2>&1 | grep "^\[" | tee -a $RES ;;
    sweep)
      ( while true; do python3 rt_sample.py 30 $LABEL-during-sweep 2>&1 | grep "^\[" >> $TR/$LABEL.rt_during; sleep 570; done ) &
      SP=$!
      before=$(ls -t /home/jason/sweep_1F_*.csv 2>/dev/null | head -1)
      ST=$(date +%s)
      timeout 21600 ros2 run spatial_maps sweep_test.py --ros-args -p floor:=1F -p skip_inaccessible:=true \
        > $TR/$LABEL.sweep.log 2>&1
      say "[$LABEL] sweep exit=$? wall=$(( ($(date +%s) - ST) / 60 )) min"
      kill $SP 2>/dev/null; pkill -f "rt_sample.py 30 $LABEL-during-sweep" 2>/dev/null
      after=$(ls -t /home/jason/sweep_1F_*.csv 2>/dev/null | head -1)
      [ "$after" != "$before" ] && say "[$LABEL] sweep csv=$after" || say "[$LABEL] sweep produced no new CSV"
      ;;
  esac
done

if grep -qiE "segmentation fault|core dumped|\[gz-[0-9]+\] .*process has died" $LOG; then
  say "[$LABEL] WARNING: crash text in launch log"
fi

say "[$LABEL] amcl_down=$(grep -c 'amcl IS DOWN' $LOG) nodes_active_msgs=$(grep -c 'Managed nodes are active' $LOG)"
kill -INT -- -$LP 2>/dev/null
for i in $(seq 1 40); do kill -0 $LP 2>/dev/null || break; sleep 1; done
kill -KILL -- -$LP 2>/dev/null
pkill -KILL -f "^gz sim .*1f.world" 2>/dev/null
sleep 3
say "[$LABEL] done=$(date '+%F %T') leftover_gz=$(pgrep -fc '^gz sim')"
