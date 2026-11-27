#!/usr/bin/env bash
# With/without test of the predictive layer: N runs of one mode, full restart each run.
# Headless, no RViz. Gazebo + Nav2 (layer on or off) + obstacle detector + tracker in BOTH modes.
# The goal, the box start and the recording are done by ab_record.py; numbers by ab_analyze.py.
# Pass rule: NOTES.md. This script does not judge it.
# Usage: scripts/ab_batch.sh PREFIX on|off COUNT DELAY_S
#   DELAY_S = sim seconds from "goal accepted" to the box start call.
set -u

usage() { echo "Usage: $0 PREFIX on|off COUNT DELAY_S" >&2; exit 1; }
[ $# -eq 4 ] || usage
P=$1; MODE=$2; N=$3; DELAY=$4

PS_PATTERN="gz|component_container|rviz|moving_obstacle|obstacle_detector|obstacle_tracker|parameter_bridge|robot_state_publisher|ros2 launch"
[[ $P =~ ^[A-Za-z0-9_-]+$ ]] || { echo "PREFIX must be letters, digits, _ or -" >&2; exit 1; }
# The prefix appears in our own command lines (script args, recorder), so it must not match.
if echo "$P" | grep -qE "$PS_PATTERN"; then echo "PREFIX must not match: $PS_PATTERN" >&2; exit 1; fi
case $MODE in
  on) LAYER=true ;;
  off) LAYER=false ;;
  *) usage ;;
esac
[[ $N =~ ^[1-9][0-9]*$ ]] || { echo "COUNT must be a positive integer" >&2; exit 1; }
[[ $DELAY =~ ^[0-9]+(\.[0-9]+)?$ ]] || { echo "DELAY_S must be a number >= 0" >&2; exit 1; }

GZ_TIMEOUT=60        # wait for "Entity creation successful"
NAV2_TIMEOUT=120     # wait for "Managed nodes are active" x2
PERCEPTION_WAIT=10   # after starting detector + tracker, before the recorder
RECORD_TIMEOUT=240   # whole recorder; its own goal timeout is 90 s
LOGDIR=~/run_logs
HERE=$(cd "$(dirname "$0")" && pwd)

# setup.bash reads unset variables, so relax -u while sourcing it.
set +u
source ~/ros2_ws/install/setup.bash || { echo "Cannot source ~/ros2_ws/install/setup.bash" >&2; exit 1; }
set -u

mkdir -p "$LOGDIR"
if compgen -G "$LOGDIR/${P}_*" > /dev/null; then
  echo "REFUSING: log files for prefix '$P' already exist in $LOGDIR:" >&2
  ls "$LOGDIR/${P}_"* >&2
  exit 1
fi

leftovers() { ps aux | grep -E "$PS_PATTERN" | grep -v grep; }

if [ -n "$(leftovers)" ]; then
  echo "REFUSING: processes already running:" >&2
  leftovers >&2
  exit 1
fi

LAUNCH_PID=""; PERC_PID=""; TRACK_PID=""
declare -a RESULT   # empty = valid, else the reason the run does not count

# count_before_shutdown PATTERN FILE -> count of PATTERN lines before the first shutdown line
count_before_shutdown() {
  [ -f "$2" ] || { echo 0; return; }
  awk -v pat="$1" '/signal_handler|user interrupted with ctrl-c/{exit} index($0, pat){c++}
                   END{print c+0}' "$2"
}

# wait_count FILE PATTERN COUNT TIMEOUT -> 0 if FILE has >= COUNT lines with PATTERN in time
wait_count() {
  local f=$1 pat=$2 want=$3 to=$4 t=0
  while [ "$t" -lt "$to" ]; do
    [ "$(count_before_shutdown "$pat" "$f")" -ge "$want" ] && return 0
    sleep 1; t=$((t + 1))
  done
  return 1
}

group_alive() { [ -n "$1" ] && kill -0 -- "-$1" 2>/dev/null; }

# Stop one process group: SIGINT, wait up to 15 s, SIGTERM, wait 5 s, SIGKILL.
stop_group() {
  local pg=$1 t
  [ -n "$pg" ] || return 0
  group_alive "$pg" || return 0
  kill -INT -- "-$pg" 2>/dev/null
  for t in $(seq 15); do group_alive "$pg" || return 0; sleep 1; done
  kill -TERM -- "-$pg" 2>/dev/null
  for t in $(seq 5); do group_alive "$pg" || return 0; sleep 1; done
  kill -KILL -- "-$pg" 2>/dev/null
  sleep 1
}

stop_all() {
  stop_group "$TRACK_PID"; stop_group "$PERC_PID"; stop_group "$LAUNCH_PID"
  TRACK_PID=""; PERC_PID=""; LAUNCH_PID=""
}

on_signal() { echo; echo "Interrupted, stopping everything..."; stop_all; exit 130; }
trap on_signal INT TERM
trap stop_all EXIT

# mode_ok LOG -> 0 if the launch log shows the requested layer mode
mode_ok() {
  local arg local_n global_n
  arg=$(count_before_shutdown "predictive_layer:=true, Nav2 params file" "$1")
  local_n=$(count_before_shutdown '[local_costmap.local_costmap]: Initialized plugin "predictive_layer"' "$1")
  global_n=$(count_before_shutdown '[global_costmap.global_costmap]: Initialized plugin "predictive_layer"' "$1")
  echo "Run mode check ($MODE): launch line=$arg, local layer=$local_n, global layer=$global_n"
  if [ "$MODE" = on ]; then
    [ "$arg" -ge 1 ] && [ "$local_n" -ge 1 ] && [ "$global_n" -ge 1 ]
  else
    [ "$arg" -eq 0 ] && [ "$local_n" -eq 0 ] && [ "$global_n" -eq 0 ]
  fi
}

# one_run INDEX RUN_ID: starts everything and records. Sets RESULT[INDEX] if the run is bad.
one_run() {
  local i=$1 id=$2 rc
  local launch_log=$LOGDIR/${2}_launch.log

  # 1. Gazebo + Nav2 (same launch path for both modes)
  setsid ros2 launch diffdrive_nav2_ros2 bringup.launch.py headless:=true rviz:=false \
    moving_obstacle:=true predictive_layer:="$LAYER" > "$launch_log" 2>&1 &
  LAUNCH_PID=$!
  wait_count "$launch_log" "Entity creation successful" 1 "$GZ_TIMEOUT" \
    || { RESULT[$i]="INVALID(gazebo)"; return; }
  wait_count "$launch_log" "Managed nodes are active" 2 "$NAV2_TIMEOUT" \
    || { RESULT[$i]="INVALID(nav2)"; return; }
  echo "Nav2 active x2"
  mode_ok "$launch_log" || { RESULT[$i]="MODE_MISMATCH"; return; }

  # 2. Detector, then tracker (both modes, same CPU load)
  setsid ros2 launch diffdrive_nav2_ros2 perception.launch.py > "$LOGDIR/${id}_perception.log" 2>&1 &
  PERC_PID=$!
  sleep 2
  setsid ros2 launch diffdrive_nav2_ros2 tracking.launch.py > "$LOGDIR/${id}_tracking.log" 2>&1 &
  TRACK_PID=$!
  sleep "$PERCEPTION_WAIT"
  if ! group_alive "$PERC_PID" || ! group_alive "$TRACK_PID" \
     || grep -q "process has died" "$LOGDIR/${id}_perception.log" "$LOGDIR/${id}_tracking.log"; then
    RESULT[$i]="INVALID(perception)"; return
  fi

  # 3. Goal, box start, recording
  timeout -s INT "$RECORD_TIMEOUT" python3 "$HERE/ab_record.py" "$id" --delay "$DELAY" \
    --outdir "$LOGDIR" > "$LOGDIR/${id}_record.log" 2>&1
  rc=$?
  case $rc in
    0) ;;
    3) RESULT[$i]="INVALID(tf)" ;;
    4) RESULT[$i]="INVALID(goal)" ;;
    5) RESULT[$i]="INVALID(setup)" ;;
    *) RESULT[$i]="INVALID(recorder_rc=$rc)" ;;
  esac
  if grep -qE "process has died" "$LOGDIR/${id}_perception.log" "$LOGDIR/${id}_tracking.log"; then
    RESULT[$i]="INVALID(perception)"
  fi
}

# field KEY FILE -> value of the KEY= line
field() { sed -n "s/^$1=//p" "$2" 2>/dev/null; }

print_table() {
  local i id s f
  echo
  printf "%-22s %-21s %-10s %-7s %-7s %-7s %-5s %-8s %-6s %-9s %-9s %-5s\n" \
    run result contact t_sim t_wall min_d stops nonbox plans delay_req delay_got cpu%
  for i in $(seq "$1"); do
    id=$(printf "%s_%02d_%s" "$P" "$i" "$MODE")
    s=$LOGDIR/${id}_summary.txt
    if [ -n "${RESULT[$i]:-}" ]; then
      printf "%-22s %-21s\n" "$id" "${RESULT[$i]}"
      continue
    fi
    f=()
    for k in result contact time_to_goal_sim_s time_to_goal_wall_s min_centre_dist_m \
             collision_monitor_stops track_ids_not_box global_plans \
             box_delay_requested_sim_s box_delay_measured_sim_s recorder_cpu_pct; do
      f+=("$(field "$k" "$s")")
    done
    printf "%-22s %-21s %-10s %-7s %-7s %-7s %-5s %-8s %-6s %-9s %-9s %-5s\n" "$id" "${f[@]}"
  done
  echo
  echo "Logs: $LOGDIR/${P}_*"
}

DONE=0
for i in $(seq "$N"); do
  ID=$(printf "%s_%02d_%s" "$P" "$i" "$MODE")
  if compgen -G "$LOGDIR/${ID}_*" > /dev/null; then
    echo "STOP: files for $ID already exist" >&2; break
  fi
  if [ -n "$(leftovers)" ]; then
    echo "STOP: processes running before $ID:" >&2; leftovers >&2; break
  fi
  echo "=== Run $i / $N: $ID (delay $DELAY sim s) ==="
  one_run "$i" "$ID"

  stop_all
  for t in $(seq 10); do [ -z "$(leftovers)" ] && break; sleep 1; done
  DONE=$i
  LAUNCH_LOG=$LOGDIR/${ID}_launch.log
  if [ -z "${RESULT[$i]:-}" ]; then
    # Contact detection must have been working, or "no contact" means nothing.
    if [ "$(count_before_shutdown "self-test ok" "$LAUNCH_LOG")" -lt 1 ] \
       || [ "$(count_before_shutdown "CONTACT DETECTION NOT WORKING" "$LAUNCH_LOG")" -gt 0 ]; then
      RESULT[$i]="DETECT_FAIL"
    else
      STOPS=$(count_before_shutdown "Robot to stop" "$LAUNCH_LOG")
      python3 "$HERE/ab_analyze.py" "$ID" --stops "$STOPS" --outdir "$LOGDIR" > /dev/null \
        || RESULT[$i]="ANALYZE_FAIL"
    fi
  fi

  if [ -n "$(leftovers)" ]; then
    echo "ABORT: processes still running after $ID:" >&2
    leftovers >&2
    break
  fi
  echo "$ID finished: ${RESULT[$i]:-ok}, nothing left running."
done

print_table "$DONE"
