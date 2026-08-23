#!/usr/bin/env bash
# =============================================================================
# run_experiment.sh — run ONE TFM meaconing experiment (one at a time)
# =============================================================================
# Runs a single experiment end-to-end: clean slate → set params → build →
# launch → record rosbag → tear down everything. Designed to be invoked one
# experiment at a time so Gazebo + node processes never accumulate across runs.
#
# Usage (from the ROS environment):
#   cd ~/tfm_meaconing_ws
#   source install/setup.bash
#   export TURTLEBOT3_MODEL=waffle
#   ./src/collaborative_detection/scripts/run_experiment.sh e1
#
#   ./src/collaborative_detection/scripts/run_experiment.sh e0 --duration 60
#   ./src/collaborative_detection/scripts/run_experiment.sh e2 --gui   # keep the GUI (debugging)
#
# Experiments:
#   e0  baseline          activation_delay=9999   (no attack → no false positives)
#   e1  slow_drift        drift_velocity=0.1
#   e2  fast_drift        drift_velocity=0.5
#   e3  hot_start         activation_delay=0      (attack from t=0)
#   e4  wide_separation   robot2.x=5.0
#   e5  waypoint_attack   GNSS-based multi-waypoint route + single-robot meaconing
#                          robot1 navigates via gnss_spoofed (drifts under attack)
#                          robot2 navigates via gnss_clean (unaffected)
#   e5_ref reference       same route, no meaconing (reference trajectory)
# =============================================================================

set -eo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
WS_DIR="${HOME}/tfm_meaconing_ws"
PARAMS_SRC="${WS_DIR}/src/collaborative_detection/config/params.yaml"
PARAMS_BAK="${PARAMS_SRC}.bak"
RESULTS_DIR="${WS_DIR}/results"
DURATION=90
GUI=false

RECORD_TOPICS=(
    /robot1/gnss_spoofed
    /robot2/gnss_spoofed
    /robot1/gnss_clean
    /robot2/gnss_clean
    /robots/uwb_distance
    /system/cusum_value
    /system/cusum_plus
    /system/cusum_minus
    /system/delta_value
    /system/delta_raw
    /system/meaconing_alert
    /meaconing/active
    /robot1/odom
    /robot2/odom
)

mkdir -p "${RESULTS_DIR}"

# ---------------------------------------------------------------------------
# Usage
# ---------------------------------------------------------------------------
usage() {
    sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
EXP=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        e0|e1|e2|e3|e4|e5|e5_ref)
            EXP="$1"
            ;;
        --duration)
            DURATION="$2"
            shift
            ;;
        --gui)
            GUI=true
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage
            ;;
    esac
    shift
done

if [[ -z "${EXP}" ]]; then
    echo "Error: specify an experiment (e0, e1, e2, e3, e4, e5, e5_ref)" >&2
    usage
fi

# ---------------------------------------------------------------------------
# Experiment definitions
# ---------------------------------------------------------------------------
case "${EXP}" in
    e0) NAME="baseline" ;;
    e1) NAME="slow_drift" ;;
    e2) NAME="fast_drift" ;;
    e3) NAME="hot_start" ;;
    e4) NAME="wide_separation" ;;
    e5) NAME="waypoint_attack" ;;
    e5_ref) NAME="waypoint_reference" ;;
esac
RUN_ID="${EXP}_${NAME}"

# Per-experiment launch arguments (spawn positions, waypoint mode, etc.)
case "${EXP}" in
    e4) EXP_LAUNCH_ARGS="x2:=5.0" ;;
    e5|e5_ref) EXP_LAUNCH_ARGS="waypoint_mode:=true x2:=0.0 y2:=2.0" ;;
    *)  EXP_LAUNCH_ARGS="" ;;
esac

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
set_param() {
    local key="$1"
    local value="$2"
    python3 -c "
import yaml
with open('${PARAMS_SRC}') as f:
    data = yaml.safe_load(f)
params = data['/**']['ros__parameters']
parts = '${key}'.split('.')
d = params
for p in parts[:-1]:
    if p not in d or not isinstance(d[p], dict):
        d[p] = {}
    d = d[p]
d[parts[-1]] = ${value}
with open('${PARAMS_SRC}', 'w') as f:
    yaml.dump(data, f, default_flow_style=False, sort_keys=False)
print(f'  [params] ${key} = ${value}')
"
}

apply_params() {
    # Reset to the pristine defaults, then apply the deltas for this experiment.
    cp "${PARAMS_BAK}" "${PARAMS_SRC}"
    case "${EXP}" in
        e0)
            set_param activation_delay 9999.0
            ;;
        e1)
            set_param activation_delay 30.0
            set_param drift_velocity 0.1
            ;;
        e2)
            set_param activation_delay 30.0
            set_param drift_velocity 0.5
            ;;
        e3)
            set_param activation_delay 2.0
            set_param drift_velocity 0.2
            set_param startup_delay 3.0
            ;;
        e4)
            set_param activation_delay 30.0
            set_param startup_delay 5.0
            set_param robot2.x 5.0
            ;;
        e5)
            set_param activation_delay 30.0
            set_param drift_velocity 0.2
            set_param robot2.x 0.0
            set_param robot2.y 2.0
            set_param robot2_waypoint_mode True
            set_param startup_delay 10.0
            ;;
        e5_ref)
            set_param activation_delay 9999.0
            set_param robot2.x 0.0
            set_param robot2.y 2.0
            set_param robot2_waypoint_mode True
            set_param startup_delay 10.0
            ;;
    esac
}

snapshot_params() {
    cp "${PARAMS_SRC}" "${RESULTS_DIR}/${RUN_ID}_params.yaml"
}

# ---------------------------------------------------------------------------
# kill_everything — kill every process this project can leave behind.
#
# This is the important part: killing only the `ros2 launch` process does NOT
# stop the node processes it spawned, so they pile up across runs (a leftover
# robot_mover_node pinned at 100% CPU was starving later experiments). We kill
# each executable by name, then force-kill any stragglers.
# ---------------------------------------------------------------------------
kill_everything() {
    local mode="${1:-term}"   # 'term' or 'kill'
    local sig="-TERM"
    [[ "${mode}" == "kill" ]] && sig="-KILL"

    local pat
    for pat in \
        "robot_mover_node" \
        "waypoint_follower_node" \
        "cusum_detector_node" \
        "gnss_sim_node" \
        "gnss_viz_node" \
        "uwb_sim_node" \
        "meaconing_injector" \
        "parameter_bridge" \
        "robot_state_publisher" \
        "ros_gz_sim" \
        "ros2 bag record" \
        "experiment.launch.py" \
        "two_robots.launch.py" \
        "gz sim" \
        "gz server" \
        "gzclient" \
        "gzserver"; do
        pkill "${sig}" -f "${pat}" 2>/dev/null || true
    done
}

teardown() {
    echo "  [teardown] Stopping all experiment processes..."
    # Rosbag first so it finalises cleanly
    pkill -TERM -f "ros2 bag record" 2>/dev/null || true
    sleep 2
    kill_everything term
    sleep 3
    # Force-kill anything that ignored SIGTERM (hung DDS init, etc.)
    kill_everything kill
    echo "  [teardown] Done"
}

# ---------------------------------------------------------------------------
# Main flow
# ---------------------------------------------------------------------------
main() {
    echo ""
    echo "============================================================================"
    echo "  EXPERIMENT: ${RUN_ID}"
    echo "  Duration:   ${DURATION}s"
    echo "  GUI:        ${GUI}"
    echo "============================================================================"

    # --- 0. Guard: remove nested colcon artifacts that shadow the real install ---
    # Running `colcon build` from inside src/ creates src/{install,build,log},
    # which leak onto AMENT_PREFIX_PATH and make every node read STALE default
    # params (the historical bug: every experiment ran with activation_delay=30,
    # drift_velocity=0.2 regardless of what was set).
    for nested in src/install src/build src/log; do
        if [[ -d "${WS_DIR}/${nested}" ]]; then
            echo "  [guard] Removing stale nested colcon dir: ${nested}"
            rm -rf "${WS_DIR}/${nested}"
        fi
    done
    PKG_PREFIX="$(ros2 pkg prefix collaborative_detection 2>/dev/null || true)"
    if [[ -n "${PKG_PREFIX}" && "${PKG_PREFIX}" != "${WS_DIR}/install/collaborative_detection" ]]; then
        echo "  [guard] WARNING: package resolves to ${PKG_PREFIX}" >&2
        echo "  [guard]          expected ${WS_DIR}/install/collaborative_detection" >&2
        echo "  [guard]          Open a FRESH terminal and re-run: source install/setup.bash" >&2
    fi

    # --- 0. Clean slate: remove leftovers from any previous run ---
    echo "  [cleanup] Clearing leftover processes (if any)..."
    kill_everything term
    sleep 2
    kill_everything kill
    echo "  [cleanup] OK"

    # --- 0b. E5 dependency: run the reference pass if it doesn't exist ---
    if [[ "${EXP}" == "e5" ]]; then
        local ref_dir="${RESULTS_DIR}/e5_ref_waypoint_reference"
        local ref_params="${ref_dir}/e5_ref_waypoint_reference_params.yaml"
        local ref_stale=true

        if [[ -d "${ref_dir}" && -f "${ref_params}" ]]; then
            # Check if the reference was recorded with the current waypoint
            # configuration (multi-waypoint: waypoints_x) or the old one
            # (single waypoint: waypoint_x).  If the snapshot doesn't mention
            # waypoints_x, it's stale and must be regenerated.
            # The reference is stale if its params snapshot doesn't contain
            # waypoints1_x (old single-waypoint code used waypoint_x instead).
            if grep -q 'waypoints1_x' "${ref_params}" 2>/dev/null; then
                # Also verify the spawn offset matches (catches old refs
                # recorded with robot2.y = 0 instead of 2).
                if grep -q "robot2.y.*2" "${ref_params}" 2>/dev/null; then
                    ref_stale=false
                    echo "  [e5 prereq] Reference trajectory found and appears fresh"
                else
                    echo "  [e5 prereq] Reference snapshot is stale (wrong robot2 Y offset)"
                fi
            else
                echo "  [e5 prereq] Reference snapshot is stale (old single-waypoint code)"
            fi
        fi

        if [[ "${ref_stale}" == "true" ]]; then
            if [[ -d "${ref_dir}" ]]; then
                echo "  [e5 prereq] Removing stale reference: ${ref_dir}"
                rm -rf "${ref_dir}"
            fi
            echo ""
            echo "  [e5 prereq] Reference trajectory missing or stale → running e5_ref first..."
            echo "  [e5 prereq] (This records the ground-truth trajectory without meaconing)"
            echo ""
            # Restore params before calling ourselves (the trap hasn't fired yet)
            cp "${PARAMS_BAK}" "${PARAMS_SRC}"
            # Run the reference pass (with the same duration and GUI flag)
            local gui_flag=""
            [[ "${GUI}" == true ]] && gui_flag="--gui"
            "$0" e5_ref --duration "${DURATION}" ${gui_flag}
            if [[ ! -d "${ref_dir}" ]]; then
                echo "  [e5 prereq] ERROR: e5_ref did not produce results at ${ref_dir}" >&2
                exit 1
            fi
            echo ""
            echo "  [e5 prereq] Reference trajectory recorded. Proceeding with attack experiment..."
            echo ""
            # Re-apply e5 params (the recursive call restored params from its own
            # trap — the build+sync below will pick them up)
            apply_params
        fi
    fi

    # --- 1. Params ---
    echo "  [params] Applying ${RUN_ID} parameters..."
    apply_params

    # --- 2. Build ---
    echo "  [build] Running colcon build..."
    cd "${WS_DIR}"
    if ! colcon build --packages-select collaborative_detection \
            > "/tmp/build_${RUN_ID}.log" 2>&1; then
        echo "  [build] FAILED — see /tmp/build_${RUN_ID}.log" >&2
        tail -25 "/tmp/build_${RUN_ID}.log" >&2
        exit 1
    fi
    tail -3 "/tmp/build_${RUN_ID}.log"

    # --- 3. Sync params to the install tree (explicit + bulletproof) ---
    # colcon copies data_files on build, but this guarantees the nodes read the
    # exact params we just set — even if the build's install step is skipped or
    # cached (the historical bug: every experiment ran with default params).
    INSTALL_PARAMS="${WS_DIR}/install/collaborative_detection/share/collaborative_detection/config/params.yaml"
    cp "${PARAMS_SRC}" "${INSTALL_PARAMS}"
    if ! diff -q "${PARAMS_SRC}" "${INSTALL_PARAMS}" >/dev/null; then
        echo "  [verify] params.yaml still differs after copy" >&2
        exit 1
    fi
    echo "  [verify] params.yaml synced to install"

    # --- 4. Launch (headless by default) ---
    local gui_arg="gui:=false"
    [[ "${GUI}" == true ]] && gui_arg="gui:=true"
    echo "  [launch] Starting experiment (${gui_arg})..."
    ros2 launch collaborative_detection experiment.launch.py "${gui_arg}" ${EXP_LAUNCH_ARGS} \
        > "/tmp/exp_${RUN_ID}.log" 2>&1 &
    LAUNCH_PID=$!
    echo "  [launch] PID=${LAUNCH_PID}"

    # --- 5. Wait for Gazebo + robots to start ---
    echo "  [wait] Waiting 15s for Gazebo startup..."
    sleep 15

    # --- 5b. Confirm the nodes read the params we set (not stale defaults) ---
    echo "  [verify] Effective params reported by the nodes:"
    grep -m1 'Attack will auto-activate' "/tmp/exp_${RUN_ID}.log" 2>/dev/null \
        || echo "  [verify]   (activation message not found yet)"
    grep -m1 'Meaconing Injector started' "/tmp/exp_${RUN_ID}.log" 2>/dev/null \
        || echo "  [verify]   (injector start line not found yet)"

    # --- 6. Record rosbag ---
    rm -rf "${RESULTS_DIR}/${RUN_ID}"
    echo "  [record] rosbag → ${RESULTS_DIR}/${RUN_ID}"
    ros2 bag record -o "${RESULTS_DIR}/${RUN_ID}" \
        "${RECORD_TOPICS[@]}" \
        > "/tmp/bag_${RUN_ID}.log" 2>&1 &
    RECORD_PID=$!

    # --- 7. Run for the remaining time ---
    local remaining=$((DURATION - 15))
    echo "  [run] Recording for ${remaining}s (Ctrl+C to stop early)..."
    sleep "${remaining}" || true

    # --- 8. Tear down ---
    kill "${RECORD_PID}" 2>/dev/null || true
    sleep 2
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    teardown

    snapshot_params
    echo "  [done] Rosbag: ${RESULTS_DIR}/${RUN_ID}"
    echo ""
}

# Back up pristine params, restore them on exit
cp "${PARAMS_SRC}" "${PARAMS_BAK}"
trap 'cp "${PARAMS_BAK}" "${PARAMS_SRC}" 2>/dev/null || true; kill_everything kill' EXIT

main
