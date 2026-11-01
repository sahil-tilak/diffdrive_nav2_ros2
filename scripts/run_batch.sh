#!/usr/bin/env bash
# Run N Nav2 goal tests (map (3.0, 0.0), heading +x) with a full restart each run,
# then print a results table.
# Usage: scripts/run_batch.sh PREFIX COUNT HEADLESS(true|false) RVIZ(true|false)
set -u

usage() { echo "Usage: $0 PREFIX COUNT HEADLESS(true|false) RVIZ(true|false)" >&2; exit 1; }
[ $# -eq 4 ] || usage
P=$1; N=$2; HEADLESS=$3; RVIZ=$4

PS_PATTERN="gz|component_container|rviz"
[[ $P =~ ^[A-Za-z0-9_-]+$ ]] || { echo "PREFIX must be letters, digits, _ or -" >&2; exit 1; }
# The prefix appears in our own command lines (script args, tee), so it must not match the ps check.
if echo "$P" | grep -qE "$PS_PATTERN"; then echo "PREFIX must not contain gz, rviz or component_container" >&2; exit 1; fi
[[ $N =~ ^[1-9][0-9]*$ ]] || { echo "COUNT must be a positive integer" >&2; exit 1; }
[[ $HEADLESS == true || $HEADLESS == false ]] || usage
[[ $RVIZ == true || $RVIZ == false ]] || usage

GZ_TIMEOUT=60      # wait for "Entity creation successful"
NAV2_TIMEOUT=120   # wait for "Managed nodes are active" x2
GOAL_TIMEOUT=90
ACTIVE_WAIT=10     # after "Managed nodes are active" x2, before RViz/goal
RVIZ_WAIT=15
LOGDIR=~/run_logs

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

PKG=$(ros2 pkg prefix diffdrive_nav2_ros2) || { echo "Package not found" >&2; exit 1; }
PARAMS=$PKG/share/diffdrive_nav2_ros2/config/nav2_params.yaml
MAP=$PKG/share/diffdrive_nav2_ros2/maps/test_world_map.yaml
TMPD=$(mktemp -d)

GZ_PID=""; NAV2_PID=""; RVIZ_PID=""
declare -a RESULT   # empty = valid, else INVALID(reason)

# wait_count FILE PATTERN COUNT TIMEOUT -> 0 if FILE has >= COUNT lines with PATTERN in time
wait_count() {
  local f=$1 pat=$2 want=$3 to=$4 t=0 c
  while [ "$t" -lt "$to" ]; do
    c=$(grep -c "$pat" "$f" 2>/dev/null); c=${c:-0}
    [ "$c" -ge "$want" ] && return 0
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
  stop_group "$RVIZ_PID"; stop_group "$NAV2_PID"; stop_group "$GZ_PID"
  RVIZ_PID=""; NAV2_PID=""; GZ_PID=""
}

on_signal() { echo; echo "Interrupted, stopping everything..."; stop_all; exit 130; }
trap on_signal INT TERM
trap stop_all EXIT

# count_before_shutdown PATTERN FILE -> count of PATTERN lines before first "signal_handler" line
count_before_shutdown() {
  [ -f "$2" ] || { echo 0; return; }
  awk -v pat="$1" '/signal_handler/{exit} index($0, pat){c++} END{print c+0}' "$2"
}

print_table() {
  local i NN nav goal nodes status stops errors time ymm
  echo
  printf "%-4s %-12s %-20s %-6s %-7s %-6s %-8s %-8s\n" run nodes_active status stops errors time y_min y_max
  for i in $(seq "$1"); do
    NN=$(printf "%02d" "$i")
    nav=$LOGDIR/${P}_${NN}_nav2.log
    goal=$LOGDIR/${P}_${NN}_goal.log
    nodes=$(grep -c "Managed nodes are active" "$nav" 2>/dev/null || true); nodes=${nodes:-0}
    # Count only lines before shutdown (first "signal_handler"); whole file if none.
    stops=$(count_before_shutdown "Robot to stop" "$nav")
    errors=$(count_before_shutdown "ERROR" "$nav")
    status=$(grep "status:" "$goal" 2>/dev/null | tail -1 | awk '{print $NF}'); status=${status:--}
    [ -n "${RESULT[$i]:-}" ] && status=${RESULT[$i]}
    time=$(awk '/navigation_time:/{n=1; next} n && /^ *sec:/{t=$2; n=0} END{print (t=="" ? "-" : t)}' "$goal" 2>/dev/null); time=${time:--}
    ymm=$(awk '/^Feedback:/{f=1} f && /position:/{p=1; next} f && p && /^ *y:/{v=$2+0; p=0;
               if (!seen || v<mn) mn=v; if (!seen || v>mx) mx=v; seen=1}
               END{if (seen) printf "%.3f %.3f", mn, mx; else print "- -"}' "$goal" 2>/dev/null)
    [ -n "$ymm" ] || ymm="- -"
    printf "%-4s %-12s %-20s %-6s %-7s %-6s %-8s %-8s\n" "$NN" "$nodes" "$status" "$stops" "$errors" "$time" $ymm
  done
  local nlogs
  nlogs=$(compgen -G "$LOGDIR/${P}_*.log" | wc -l)
  echo
  echo "Log files for '$P': $nlogs (expected 2 x $N = $((2 * N)))"
  echo "Gazebo output (not counted): $TMPD"
}

DONE=0
for i in $(seq "$N"); do
  NN=$(printf "%02d" "$i")
  NAV2_LOG=$LOGDIR/${P}_${NN}_nav2.log
  GOAL_LOG=$LOGDIR/${P}_${NN}_goal.log
  GZ_LOG=$TMPD/${P}_${NN}_gazebo.out
  if [ -e "$NAV2_LOG" ] || [ -e "$GOAL_LOG" ]; then
    echo "STOP: $NAV2_LOG or $GOAL_LOG already exists" >&2; break
  fi
  echo "=== Run $NN / $N ==="

  # 1. Gazebo
  setsid ros2 launch diffdrive_nav2_ros2 gazebo.launch.py headless:="$HEADLESS" > "$GZ_LOG" 2>&1 &
  GZ_PID=$!
  if ! wait_count "$GZ_LOG" "Entity creation successful" 1 "$GZ_TIMEOUT"; then
    echo "Run $NN: Gazebo timeout"; RESULT[$i]="INVALID(gazebo)"
  else
    sleep 5
    # 2. Nav2
    setsid bash -c 'ros2 launch nav2_bringup bringup_launch.py map:="$1" params_file:="$2" use_sim_time:=true 2>&1 | tee -i "$3"' \
      _ "$MAP" "$PARAMS" "$NAV2_LOG" > /dev/null &
    NAV2_PID=$!
    # 3. Wait for managed nodes active x2
    if ! wait_count "$NAV2_LOG" "Managed nodes are active" 2 "$NAV2_TIMEOUT"; then
      echo "Run $NN: Nav2 timeout"; RESULT[$i]="INVALID(nav2)"
    else
      sleep "$ACTIVE_WAIT"
      # 4. RViz
      if [ "$RVIZ" = true ]; then
        setsid ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
          --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
        RVIZ_PID=$!
        sleep "$RVIZ_WAIT"
      fi
      # 5. Goal
      timeout "$GOAL_TIMEOUT" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}}}" \
        --feedback 2>&1 | tee -i "$GOAL_LOG" > /dev/null
      if [ "${PIPESTATUS[0]}" -eq 124 ]; then
        echo "Run $NN: goal timeout"; RESULT[$i]="INVALID(goal)"
      fi
    fi
  fi

  # 6. Stop everything and check
  stop_all
  for t in $(seq 10); do [ -z "$(leftovers)" ] && break; sleep 1; done
  DONE=$i
  if [ -n "$(leftovers)" ]; then
    echo "ABORT: processes still running after run $NN:" >&2
    leftovers >&2
    break
  fi
  echo "Run $NN finished: ${RESULT[$i]:-ok}, nothing left running."
done

print_table "$DONE"
