#!/bin/bash
# run_sim.sh - Foolproof PX4 Swarm SITL Demo
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DURATION="${DURATION:-480}"
UAV_COUNT="${UAV_COUNT:-9}"
AUTO_ARM="${AUTO_ARM:-true}"
ENABLE_QGC="${ENABLE_QGC:-true}"
ENABLE_GZ_GUI="${ENABLE_GZ_GUI:-true}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SCRIPT="$SCRIPT_DIR/UAV_X_Individual_Drone_Node_v4_PX4.py"
LOG_DIR="$SCRIPT_DIR/uav_x_logs"
QGC_BIN="${QGC_BIN:-$HOME/Downloads/QGroundControl-x86_64.AppImage}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_GZ_ENV="$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
PX4_AVAILABLE=false
# 3x3 spawn grid — 20m x 20m, 10m spacing, starts at (15,15)
GRID_COLS=3
GRID_SPACING=10
GRID_X_START=15
GRID_Y_START=15
UAV_PIDS=()
PX4_PIDS=()
DRYRUN_PIDS=()

start_local_dry_run() {
  echo "[WARN] PX4/Gazebo stack is unavailable or not responding; using local dry-run swarm simulation instead."
  local uav_id grid_col grid_row origin_x origin_y
  for ((uav_id=1; uav_id<=UAV_COUNT; uav_id++)); do
    grid_col=$(( (uav_id - 1) % GRID_COLS ))
    grid_row=$(( (uav_id - 1) / GRID_COLS ))
    origin_x=$(( GRID_X_START + grid_col * GRID_SPACING ))
    origin_y=$(( GRID_Y_START + grid_row * GRID_SPACING ))
    python3 "$SCRIPT" --dry-run --duration "$DURATION" --uav-id "$uav_id" --swarm-size "$UAV_COUNT" --origin-x "$origin_x" --origin-y "$origin_y" --gcs-x 0 --gcs-y 0 --gcs-z 0 --log-dir "$LOG_DIR" > "$LOG_DIR/uav_${uav_id}_dryrun.log" 2>&1 &
    DRYRUN_PIDS+=("$!")
  done
  sleep "$DURATION"
  for ((uav_id=1; uav_id<=UAV_COUNT; uav_id++)); do
    if ! wait "${DRYRUN_PIDS[uav_id-1]}"; then
      echo "[WARN] UAV $uav_id dry-run process exited early."
    fi
  done
  echo "[INFO] Local dry-run swarm complete. Logs in $LOG_DIR/"
  exit 0
}

fail_and_fallback() {
  echo "[WARN] PX4 runtime startup failed; falling back to local dry-run simulation."
  cleanup
  start_local_dry_run
}

if [[ ! "$DURATION" =~ ^[1-9][0-9]*$ ]]; then
  echo "[ERROR] DURATION must be a positive whole number of seconds."
  exit 1
fi

if [[ ! "$UAV_COUNT" =~ ^[1-9][0-9]*$ ]] || (( UAV_COUNT > 16 )); then
  echo "[ERROR] UAV_COUNT must be a whole number from 1 to 16."
  exit 1
fi

if [[ ! -f "$SCRIPT" ]]; then
  echo "[ERROR] UAV node script not found: $SCRIPT"
  exit 1
fi

if [[ "$ENABLE_QGC" != "true" && "$ENABLE_QGC" != "false" ]]; then
  echo "[ERROR] ENABLE_QGC must be 'true' or 'false'."
  exit 1
fi

if [[ "$ENABLE_GZ_GUI" != "true" && "$ENABLE_GZ_GUI" != "false" ]]; then
  echo "[ERROR] ENABLE_GZ_GUI must be 'true' or 'false'."
  exit 1
fi

mkdir -p "$LOG_DIR"

if ! command -v python3 >/dev/null 2>&1; then
  echo "[ERROR] python3 not found in PATH"
  exit 1
fi

if [[ -d "$PX4_DIR" && -x "$PX4_BIN" && -f "$PX4_GZ_ENV" ]] && command -v MicroXRCEAgent >/dev/null 2>&1 && command -v ros2 >/dev/null 2>&1 && command -v gz >/dev/null 2>&1; then
  PX4_AVAILABLE=true
else
  PX4_AVAILABLE=false
fi

if [[ "$PX4_AVAILABLE" == "false" ]]; then
  start_local_dry_run
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "[ERROR] python3 not found in PATH"
  exit 1
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "[ERROR] MicroXRCEAgent not found in PATH. Source the PX4 environment or install it."
  exit 1
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 not found. Source the ROS 2 environment before running SITL."
  exit 1
fi

if ! command -v gz >/dev/null 2>&1; then
  echo "[ERROR] Gazebo Sim command 'gz' not found in PATH."
  exit 1
fi

if [[ "$ENABLE_QGC" == "true" ]]; then
  if ! command -v ss >/dev/null 2>&1; then
    echo "[ERROR] ss is required to verify the QGroundControl MAVLink listener."
    exit 1
  fi
  if [[ ! -x "$QGC_BIN" ]]; then
    QGC_BIN="$(command -v qgroundcontrol || true)"
  fi
  if [[ -z "$QGC_BIN" || ! -x "$QGC_BIN" ]]; then
    echo "[WARN] QGroundControl not found; disabling QGC (not needed for SITL arming)."
    ENABLE_QGC=false
  fi
fi

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
PX4_GZ_ENV="$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
if [[ ! -x "$PX4_BIN" || ! -f "$PX4_GZ_ENV" ]]; then
  echo "[ERROR] PX4 SITL binary or generated Gazebo environment is missing under $PX4_DIR/build/px4_sitl_default"
  exit 1
fi

set +u
source "$PX4_GZ_ENV"
set -u
PROJECT_GZ_WORLDS="$SCRIPT_DIR/Tools/simulation/gz/worlds"
if [[ ! -f "$PROJECT_GZ_WORLDS/default.sdf" ]]; then
  echo "[ERROR] Project Gazebo world not found: $PROJECT_GZ_WORLDS/default.sdf"
  exit 1
fi
export PX4_GZ_WORLDS="$PROJECT_GZ_WORLDS"
export GZ_SIM_RESOURCE_PATH="$PROJECT_GZ_WORLDS:${GZ_SIM_RESOURCE_PATH:-}"

stop_process_group() {
  local leader="$1"
  local pids
  pids="$(ps -eo pid=,pgid= | awk -v group="$leader" '$2 == group {print $1}')"
  if [[ -z "$pids" ]]; then return; fi
  for pid in $pids; do kill -TERM "$pid" 2>/dev/null || true; done
  for _ in {1..5}; do
    pids="$(ps -eo pid=,pgid= | awk -v group="$leader" '$2 == group {print $1}')"
    if [[ -z "$pids" ]]; then return; fi
    sleep 1
  done
  for pid in $pids; do kill -KILL "$pid" 2>/dev/null || true; done
}

cleanup() {
  trap - EXIT SIGINT SIGTERM
  for pid in "${QGC_PID:-}" "${UAV_PIDS[@]}" "${AGENT_PID:-}" "${PX4_PIDS[@]}" "${GZ_GUI_PID:-}" "${GZ_PID:-}"; do
    if [[ -n "$pid" ]]; then
      stop_process_group "$pid"
      wait "$pid" 2>/dev/null || true
    fi
  done
  QGC_PID=""; AGENT_PID=""
  UAV_PIDS=(); PX4_PIDS=()
  GZ_GUI_PID=""; GZ_PID=""

  rm -f /tmp/px4_lock* /tmp/px4-sock* /tmp/px4_* 2>/dev/null || true

  echo ""
  echo "========================================================================"
  echo "  Gazebo simulation ended. Generating swarm network evaluation report..."
  echo "========================================================================"
  if [[ -f "$SCRIPT_DIR/generate_summary_report.py" ]]; then
    LOG_DIR="$LOG_DIR" python3 "$SCRIPT_DIR/generate_summary_report.py" 2>&1 | tee "${LOG_DIR}/swarm_network_evaluation.log" || true
  fi
  exit 0
}

trap cleanup EXIT SIGINT SIGTERM

wait_for_gz_topic() {
  local topic="$1"
  local timeout_seconds="${2:-90}"
  local deadline=$((SECONDS + timeout_seconds))
  echo "[INFO] Waiting up to ${timeout_seconds}s for Gazebo topic $topic..."
  while (( SECONDS < deadline )); do
    if timeout 5s gz topic -i -t "$topic" 2>/dev/null | grep -q '^Publishers'; then
      echo "[INFO] Gazebo topic ready: $topic"
      return 0
    fi
    sleep 1
  done
  echo "[ERROR] Timed out waiting for Gazebo topic $topic"
  return 1
}

wait_for_ros_topic() {
  local topic="$1"
  local timeout_seconds="${2:-60}"
  local deadline=$((SECONDS + timeout_seconds))
  echo "[INFO] Waiting up to ${timeout_seconds}s for ROS 2 topic discovery: $topic..."
  while (( SECONDS < deadline )); do
    if timeout 8s ros2 topic list --no-daemon --spin-time 2 2>/dev/null | grep -Fxq "$topic"; then
      echo "[INFO] ROS 2 topic discovered: $topic"
      return 0
    fi
    sleep 1
  done
  echo "[ERROR] Timed out waiting for ROS 2 topic $topic"
  return 1
}

wait_for_udp_port() {
  local port="$1"
  local timeout_seconds="${2:-30}"
  local deadline=$((SECONDS + timeout_seconds))
  echo "[INFO] Waiting up to ${timeout_seconds}s for UDP port $port..."
  while (( SECONDS < deadline )); do
    if ss -H -lun "sport = :$port" 2>/dev/null | grep -q ":$port"; then
      echo "[INFO] UDP port ready: $port"
      return 0
    fi
    sleep 1
  done
  echo "[ERROR] Timed out waiting for UDP port $port"
  return 1
}

echo "[INFO] Project dir: $SCRIPT_DIR"
echo "[INFO] PX4 dir: $PX4_DIR"
echo "[INFO] UAV script: $SCRIPT"
echo "[INFO] Swarm size: $UAV_COUNT (maximum 16)"
echo "[INFO] QGroundControl enabled: $ENABLE_QGC"
echo "[INFO] Gazebo GUI enabled: $ENABLE_GZ_GUI"

echo "Cleaning up any old leftover simulator processes..."
pkill -9 -f MicroXRCEAgent || true
pkill -9 -f "px4 " || true
pkill -9 -f "gz sim" || true
pkill -9 -f "UAV_X_Individual_Drone_Node" || true
rm -f /tmp/px4_lock* /tmp/px4-sock* /tmp/px4_* 2>/dev/null || true
sleep 1

# Clean previous CSV logs to ensure a fresh evaluation report for this run
rm -f "$LOG_DIR"/uav_*_log.csv "$LOG_DIR"/uav_*_dryrun.log "$LOG_DIR"/swarm_network_evaluation.log 2>/dev/null || true

echo "Starting isolated simulator processes..."

if [[ "$ENABLE_QGC" == "true" ]]; then
  if ! pgrep -f '[Q]GroundControl|[q]groundcontrol' >/dev/null 2>&1; then
    echo "Starting QGroundControl..."
    setsid "$QGC_BIN" > qgroundcontrol.log 2>&1 &
    QGC_PID=$!
  else
    echo "Reusing the running QGroundControl process."
  fi
  wait_for_udp_port 14550 30 || { cleanup; exit 1; }
else
  echo "[INFO] QGroundControl disabled; drones arm via DDS offboard heartbeat only."
fi

echo "Starting MicroXRCE-DDS Agent..."
cd "$PX4_DIR" || exit 1
pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
sleep 0.5
setsid MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/agent.log" 2>&1 &
AGENT_PID=$!
for _ in {1..20}; do
  if ! kill -0 "$AGENT_PID" 2>/dev/null; then
    echo "[ERROR] MicroXRCE-DDS Agent exited; see $LOG_DIR/agent.log"
    cat "$LOG_DIR/agent.log" 2>/dev/null || true
    exit 1
  fi
  if grep -q "running" "$LOG_DIR/agent.log" 2>/dev/null; then break; fi
  sleep 1
done
if ! grep -q "running" "$LOG_DIR/agent.log" 2>/dev/null; then
  echo "[ERROR] MicroXRCE-DDS Agent did not become ready; see $LOG_DIR/agent.log"
  cat "$LOG_DIR/agent.log" 2>/dev/null || true
  exit 1
fi

echo "Starting project Gazebo world..."
PX4_GZ_WORLD=default setsid gz sim --verbose=1 -r -s "$PROJECT_GZ_WORLDS/default.sdf" > "$LOG_DIR/gz_sim.log" 2>&1 &
GZ_PID=$!
wait_for_gz_topic "/world/default/clock" 120 || { echo "[ERROR] Gazebo clock failed. See $LOG_DIR/gz_sim.log:"; cat "$LOG_DIR/gz_sim.log" 2>/dev/null; cleanup; exit 1; }
if [[ "$ENABLE_GZ_GUI" == "true" ]]; then
  echo "Starting Gazebo GUI..."
  setsid gz sim -g > "$LOG_DIR/gz_gui.log" 2>&1 &
  GZ_GUI_PID=$!
fi

ARM_ARGS=()
if [[ "$AUTO_ARM" == "true" ]]; then
  ARM_ARGS=(--arm --takeoff 5.0)
elif [[ "$AUTO_ARM" != "false" ]]; then
  echo "[ERROR] AUTO_ARM must be 'true' or 'false'."
  cleanup
  exit 1
fi
export PX4_GZ_NO_FOLLOW=1

for ((uav_id=1; uav_id<=UAV_COUNT; uav_id++)); do
  grid_col=$(( (uav_id - 1) % GRID_COLS ))
  grid_row=$(( (uav_id - 1) / GRID_COLS ))
  origin_x=$(( GRID_X_START + grid_col * GRID_SPACING ))
  origin_y=$(( GRID_Y_START + grid_row * GRID_SPACING ))
  namespace="px4_$uav_id"
  inst_idx=$((uav_id - 1))
  echo "Launching PX4 SITL Instance $uav_id at grid (col=$grid_col row=$grid_row) world=(${origin_x},${origin_y})..."
  # All instances attach to the already-running gz sim server (PX4_GZ_STANDALONE=1)
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_UXRCE_DDS_NS="$namespace" \
    PX4_GZ_MODEL_POSE="${origin_x},${origin_y}" PX4_SIM_MODEL=gz_x500 \
    setsid "$PX4_BIN" -i "$inst_idx" > "$LOG_DIR/px4_${uav_id}.log" 2>&1 &
  PX4_PIDS+=("$!")

  wait_for_ros_topic "/$namespace/fmu/out/vehicle_status_v4" 60 || fail_and_fallback

  # Stagger spawns to prevent Gazebo from crashing under load
  sleep 2
done

echo "Launching $UAV_COUNT UAV Python Nodes..."
for ((uav_id=1; uav_id<=UAV_COUNT; uav_id++)); do
  grid_col=$(( (uav_id - 1) % GRID_COLS ))
  grid_row=$(( (uav_id - 1) / GRID_COLS ))
  origin_x=$(( GRID_X_START + grid_col * GRID_SPACING ))
  origin_y=$(( GRID_Y_START + grid_row * GRID_SPACING ))
  setsid python3 "$SCRIPT" --backend px4 --namespace "px4_$uav_id" --uav-id "$uav_id" \
    --swarm-size "$UAV_COUNT" --origin-x "$origin_x" --origin-y "$origin_y" \
    "${ARM_ARGS[@]}" --duration "$DURATION" --log-dir "$LOG_DIR" &
  UAV_PIDS+=("$!")
done

echo "Simulation running. Drones will autonomously RTL and the simulation will exit once all drones have disarmed."
# Remove sleep $DURATION so we rely on wait below

echo "Waiting for UAV nodes to finish their control loops..."
simulation_status=0
for ((uav_id=1; uav_id<=UAV_COUNT; uav_id++)); do
  if ! wait "${UAV_PIDS[uav_id-1]}"; then
    echo "[ERROR] UAV $uav_id process exited with an error."
    simulation_status=1
  fi
done

echo "Simulation complete. Exiting."
exit "$simulation_status"
