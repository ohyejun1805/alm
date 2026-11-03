#!/bin/bash
# Reproduce every check in order: perception and signs, closed-loop scenarios, safety stops.
# Usage: run_all_checks.sh [results_dir] [repeats]   (workspace must already be sourced)
set -u
PKG_DIR="$(rospack find rail_align_demo)"
RESULTS="$(realpath -m "${1:-$PKG_DIR/../../results}")"
REPEATS="${2:-3}"
LOGS="$RESULTS/logs"
mkdir -p "$LOGS"
failures=0

stop_sim() {
  kill -INT "$1" 2>/dev/null
  wait "$1" 2>/dev/null
  sleep 3
}

wait_for_estimator() {
  timeout 60 rostopic echo -n1 /rail_align/errors >/dev/null 2>&1 || { echo "estimator never published"; return 1; }
}

echo "== perception and error signs =="
roslaunch rail_align_demo demo.launch gui:=false control:=false record:=false >"$LOGS/perception.launch.log" 2>&1 &
sim=$!
sleep 5
wait_for_estimator && rosrun rail_align_demo verify_perception.py --results-dir "$RESULTS" || failures=$((failures + 1))
stop_sim "$sim"

echo "== closed-loop scenarios =="
run_scenario() {
  local tag="$1"; shift
  timeout 300 roslaunch rail_align_demo demo.launch gui:=false exit_on_finish:=true results_dir:="$RESULTS" "$@" >"$LOGS/$tag.launch.log" 2>&1
  grep -aoE "ALIGNED lateral=[^ ]+ m heading=[^ ]+ deg longitudinal=[^ ]+ m|RAIL_END: .*|FAILED: .*" "$LOGS/$tag.launch.log" | sed "s/^/$tag: /" || true
  sleep 3
}
for i in $(seq 1 "$REPEATS"); do
  run_scenario "left_$i" start:=left
  run_scenario "right_$i" start:=right
done
run_scenario left_inward start:=left_inward
run_scenario right_outward start:=right_outward
run_scenario side_left start:=side_left
run_scenario side_right start:=side_right
run_scenario custom start:=custom x:=-2.6 y:=0.15 yaw_deg:=-20

echo "== safety stops =="
for mode in occlude pause railpause nofeed blind; do
  start=left
  [ "$mode" = blind ] && start=blind
  roslaunch rail_align_demo demo.launch gui:=false record:=false start:=$start >"$LOGS/safety_$mode.launch.log" 2>&1 &
  sim=$!
  sleep 5
  wait_for_estimator && rosrun rail_align_demo verify_safety.py "$mode" --results-dir "$RESULTS" || failures=$((failures + 1))
  stop_sim "$sim"
done

echo "== summary =="
rosrun rail_align_demo summarize_results.py "$RESULTS/summary.csv" | tee "$RESULTS/scenarios.md"
echo "check failures: $failures"
exit "$failures"
