#!/usr/bin/env bash
# =============================================================================
# run_experiments.sh — TFM Meaconing Detection Experiment Runner
# =============================================================================
# Automates running experiments E0-E4, modifying params.yaml between runs
# and recording a rosbag for each.
#
# Usage (from ROS environment):
#   cd ~/tfm_meaconing_ws
#   source install/setup.bash
#   export TURTLEBOT3_MODEL=waffle
#   ./src/collaborative_detection/scripts/run_experiments.sh
#
# Experiments:
#   E0 — Baseline (no attack, validates no false positives, 90s)
#   E1 — Slow meaconing drift (0.1 m/s, 90s)
#   E2 — Fast meaconing drift (0.5 m/s, 90s)
#   E3 — Hot start (attack from t=0, 90s)
#   E4 — Wide robot separation (5m apart, 90s)
# =============================================================================

set -e

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
WS_DIR="${HOME}/tfm_meaconing_ws"
PARAMS_SRC="${WS_DIR}/src/collaborative_detection/config/params.yaml"
PARAMS_BAK="${PARAMS_SRC}.bak"
RESULTS_DIR="${WS_DIR}/results"
EXPERIMENT_DURATION=90      # seconds per experiment (excluding startup)
RECORD_TOPICS=(
    /robot1/gnss_spoofed
    /robot2/gnss_spoofed
    /robots/uwb_distance
    /system/cusum_value
    /system/delta_value
    /system/meaconing_alert
    /meaconing/active
    /robot1/odom
    /robot2/odom
)

mkdir -p "${RESULTS_DIR}"

# ---------------------------------------------------------------------------
# Helper: modify a ROS parameter in the YAML file (supports dotted nested keys)
# ---------------------------------------------------------------------------
set_param() {
    local key="$1"
    local value="$2"
    python3 -c "
import yaml

with open('${PARAMS_SRC}') as f:
    data = yaml.safe_load(f)

params = data['/**']['ros__parameters']

# Walk dotted path into nested dicts, creating intermediate levels if needed
parts = '${key}'.split('.')
d = params
for i, p in enumerate(parts[:-1]):
    if p not in d or not isinstance(d[p], dict):
        d[p] = {}
    d = d[p]
d[parts[-1]] = ${value}

with open('${PARAMS_SRC}', 'w') as f:
    yaml.dump(data, f, default_flow_style=False, sort_keys=False)
print(f'  [params] ${key} = ${value}')
"
}

# ---------------------------------------------------------------------------
# Helper: reset params to defaults + copy a snapshot to results/
# ---------------------------------------------------------------------------
reset_params() {
    cp "${PARAMS_BAK}" "${PARAMS_SRC}"
}

snapshot_params() {
    local name="$1"
    cp "${PARAMS_SRC}" "${RESULTS_DIR}/${name}_params.yaml"
}

# ---------------------------------------------------------------------------
# Helper: kill all experiment processes
# ---------------------------------------------------------------------------
kill_experiment() {
    echo "  [cleanup] Stopping processes..."
    # Kill rosbag first so it finalises the bag cleanly
    pkill -f "ros2 bag record" 2>/dev/null || true
    sleep 1
    # Kill the launch
    pkill -f "experiment.launch.py" 2>/dev/null || true
    sleep 2
    # Kill Gazebo
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "gz server" 2>/dev/null || true
    sleep 2
    echo "  [cleanup] Done"
}

# ---------------------------------------------------------------------------
# Run a single experiment
# ---------------------------------------------------------------------------
run_experiment() {
    local name="$1"
    local duration="$2"
    shift 2
    local extra_launch_args="$*"

    echo ""
    echo "============================================================================"
    echo "  EXPERIMENT: ${name}"
    echo "  Duration:   ${duration}s"
    echo "============================================================================"

    # --- Build ---
    echo "  [build] Running colcon build..."
    cd "${WS_DIR}"
    colcon build --packages-select collaborative_detection 2>&1 | tail -3

    # --- Launch ---
    echo "  [launch] Starting experiment..."
    ros2 launch collaborative_detection experiment.launch.py ${extra_launch_args} \
        > "/tmp/exp_${name}.log" 2>&1 &
    LAUNCH_PID=$!
    echo "  [launch] PID=${LAUNCH_PID}"

    # --- Wait for Gazebo + robots to fully start (~12s observed) ---
    echo "  [wait] Waiting for Gazebo startup (15s)..."
    sleep 15

    # --- Record rosbag ---
    echo "  [record] Starting rosbag → ${RESULTS_DIR}/${name}"
    ros2 bag record -o "${RESULTS_DIR}/${name}" \
        "${RECORD_TOPICS[@]}" \
        > "/tmp/bag_${name}.log" 2>&1 &
    RECORD_PID=$!
    echo "  [record] PID=${RECORD_PID}"

    # --- Run the experiment ---
    local remaining=$((duration - 15))
    echo "  [run] Recording for ${remaining}s..."
    sleep "${remaining}"

    # --- Stop ---
    echo "  [stop] Stopping rosbag..."
    kill "${RECORD_PID}" 2>/dev/null || true
    sleep 2

    echo "  [stop] Stopping launch..."
    kill "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2

    kill_experiment

    snapshot_params "${name}"
    echo "  [done] Rosbag saved: ${RESULTS_DIR}/${name}"
}

# =============================================================================
# Main
# =============================================================================

# Backup original params
cp "${PARAMS_SRC}" "${PARAMS_BAK}"
trap 'reset_params; kill_experiment' EXIT

echo ""
echo "========================================================================"
echo "  TFM Meaconing Detection — Experiment Suite"
echo "  Results directory: ${RESULTS_DIR}"
echo "========================================================================"

# ---------------------------------------------------------------------------
# E0 — BASELINE (no attack) — validates zero false positives
# ---------------------------------------------------------------------------
reset_params
set_param activation_delay 9999.0    # never activates (float!)
run_experiment "E0_baseline" "${EXPERIMENT_DURATION}"

# ---------------------------------------------------------------------------
# E1 — SLOW MEACONING DRIFT
# ---------------------------------------------------------------------------
set_param activation_delay 30.0
set_param drift_velocity 0.1
run_experiment "E1_slow_drift" "${EXPERIMENT_DURATION}"

# ---------------------------------------------------------------------------
# E2 — FAST MEACONING DRIFT
# ---------------------------------------------------------------------------
set_param drift_velocity 0.5
run_experiment "E2_fast_drift" "${EXPERIMENT_DURATION}"

# ---------------------------------------------------------------------------
# E3 — HOT START (attack from t=0)
# ---------------------------------------------------------------------------
set_param activation_delay 0.0
set_param drift_velocity 0.2        # back to default
set_param startup_delay 3.0         # shorter warmup — data arrives ~t=5s
run_experiment "E3_hot_start" "${EXPERIMENT_DURATION}"

# ---------------------------------------------------------------------------
# E4 — WIDE SEPARATION (5m between robots)
# ---------------------------------------------------------------------------
set_param activation_delay 30.0
set_param startup_delay 5.0
set_param robot2.x 5.0              # dotted path → updates nested robot2: {x: 5.0}
run_experiment "E4_wide_separation" "${EXPERIMENT_DURATION}" "x2:=5.0"

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo "========================================================================"
echo "  ALL EXPERIMENTS COMPLETE"
echo "  Results: ${RESULTS_DIR}/"
ls -d "${RESULTS_DIR}"/E* 2>/dev/null || echo "  (no results found)"
echo "========================================================================"
