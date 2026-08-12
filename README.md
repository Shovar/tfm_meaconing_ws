# Collaborative GNSS Meaconing Detection via UWB Ranging and CUSUM

**Master's Thesis (TFM)** — Security Architecture for Autonomous Robot Navigation  
Antonio García Alcón — Universidad Europea de Madrid, 2026

---

## Overview

This project implements a **collaborative meaconing detection system** for multi-robot GNSS navigation. A *meaconing attack* consists of receiving legitimate GNSS signals, delaying them, and rebroadcasting them — causing all victim receivers in range to report the same fake position. The attack is particularly dangerous for autonomous vehicle fleets because it is undetectable by any single receiver.

The detection strategy is based on a simple insight:

> **Compare two independent measurements of the same physical quantity — the inter-robot distance.**

| Source | Measurement | Vulnerable to meaconing? |
|---|---|---|
| GNSS positions | $D_{GNSS} = \|\vec{p}_A - \vec{p}_B\|$ | ✅ Yes — both spoofed to same point → $D_{GNSS} \approx 0$ |
| UWB ranging | $D_{UWB}$ (physical radio distance) | ❌ No — measures true Euclidean distance |

Under normal operation $D_{GNSS} \approx D_{UWB}$ (within sensor noise). Under a meaconing attack, $D_{GNSS}$ collapses to near-zero while $D_{UWB}$ remains at the true physical distance, creating a **persistent positive bias** in $\delta = D_{UWB} - D_{GNSS}$.

A **CUSUM** (Cumulative Sum) sequential detector accumulates this bias and triggers an alarm when the statistic crosses a threshold:

$$S_k = \max(0, S_{k-1} + \delta_k - \beta), \qquad \text{alarm if } S_k > \tau$$

The CUSUM is superior to a fixed threshold because it **accumulates evidence over time** rather than reacting to single-sample noise.

### Key references

- Bhatti & Humphreys (2017). *Hostile Control of Ships via False GPS Signals.*
- Chen et al. (2022). *A Survey of Robot Swarms' Relative Localization Method.* Sensors (MDPI), 22(11), 4212.
- Fishberg et al. (2024). *MURP: Multi-Agent Ultra-Wideband Relative Pose Estimation.*

---

## Project Structure

```
tfm_meaconing_ws/                         # ROS 2 workspace root
├── README.md                             # ← this file
├── EXPERIMENTS.md                        # Detailed experiment guide (Spanish)
│
├── src/collaborative_detection/          # Source package
│   ├── package.xml                       # ROS 2 package manifest
│   ├── setup.py                          # Python entry points
│   ├── setup.cfg
│   │
│   ├── config/
│   │   └── params.yaml                   # All tunable parameters (noise, CUSUM, attack)
│   │
│   ├── resource/
│   │   └── collaborative_detection       # ROS 2 package marker (required by ament)
│   │
│   ├── launch/
│   │   ├── experiment.launch.py          # Full pipeline: Gazebo + sensors + CUSUM
│   │   └── two_robots.launch.py          # Two TurtleBot3 robots in Gazebo Sim
│   │
│   ├── scripts/
│   │   └── run_experiments.sh            # Automated experiment runner (E0–E4)
│   │
│   ├── analysis/
│   │   └── plot_results.ipynb            # Jupyter notebook for rosbag analysis
│   │
│   └── collaborative_detection/          # Python package
│       ├── __init__.py
│       └── nodes/
│           ├── gnss_sim_node.py           # GNSS simulator (odometry → noisy GNSS)
│           ├── uwb_sim_node.py            # UWB ranging simulator (odometry → distance)
│           ├── meaconing_injector.py      # Attack injector (spoofs GNSS positions)
│           ├── cusum_detector_node.py     # CUSUM sequential detector
│           ├── robot_mover_node.py        # Autonomous circular motion controller
│           └── gnss_viz_node.py           # RViz2 Marker visualizer
│
├── build/                                # Colcon build artifacts (auto-generated)
├── install/                              # Colcon install artifacts (auto-generated)
├── log/                                  # Build logs (auto-generated)
└── results/                              # Experiment rosbags + params snapshots
    ├── E0_baseline/
    ├── E1_slow_drift/
    ├── E2_fast_drift/
    ├── E3_hot_start/
    └── E4_wide_separation/
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        GAZEBO SIM                                   │
│  ┌──────────┐                     ┌──────────┐                      │
│  │ Robot1   │──── UWB ranging ────│ Robot2   │   Physical layer     │
│  │ (x₁,y₁)  │    (D_UWB ≈ 3m)     │ (x₂,y₂)  │                      │
│  └────┬─────┘                     └────┬─────┘                      │
│       │ odom                            │ odom                       │
└───────┼─────────────────────────────────┼───────────────────────────┘
        │                                 │
        ▼                                 ▼
┌──────────────┐                  ┌──────────────┐
│ GNSS Sim     │                  │ UWB Sim      │   Sensor layer
│ + noise      │                  │ + noise      │
│ (world frame)│                  │ (world frame)│
└──────┬───────┘                  └──────┬───────┘
       │ gnss_clean                      │ uwb_distance
       ▼                                 │
┌──────────────┐                         │
│ Meaconing    │  ── gnss_spoofed ──┐    │   Attack layer
│ Injector     │                    │    │
│ (passthrough │                    │    │
│  or spoof)   │                    │    │
└──────────────┘                    │    │
                                    ▼    ▼
                            ┌──────────────────┐
                            │ CUSUM Detector   │   Detection layer
                            │ δ = D_UWB−D_GNSS │
                            │ S_k = max(0, ...) │
                            └────────┬─────────┘
                                     │ /system/meaconing_alert
                                     ▼
                                 🚨 ALARM
```

### Data Flow

1. **Gazebo Sim** runs two TurtleBot3 Waffle robots moving in autonomous circles.
2. **GNSS Sim Node** reads odometry from both robots, converts from local `odom` frame to global `world` frame using spawn offsets, adds Gaussian noise, and publishes `gnss_clean` at 30 Hz.
3. **UWB Sim Node** reads odometry from both robots, converts to world frame, computes the Euclidean distance, adds Gaussian noise (σ = 0.24 m), and publishes `uwb_distance` at 30 Hz.
4. **Meaconing Injector** subscribes to `gnss_clean` and, when inactive, passes it through as `gnss_spoofed`. When the attack activates (auto-delay or manual service call), both robots' GNSS outputs are replaced with the **same drifting fake point** plus independent noise — simulating a single-antenna meaconing attack.
5. **CUSUM Detector** subscribes to `gnss_spoofed` (both robots) and `uwb_distance`, computes $D_{GNSS}$ and $\delta$, updates the CUSUM statistic, and publishes alerts.

### Topics

| Topic | Type | Description |
|---|---|---|
| `/robot1/gnss_clean` | `PoseStamped` | Simulated GNSS position (world frame, with noise) |
| `/robot2/gnss_clean` | `PoseStamped` | Simulated GNSS position (world frame, with noise) |
| `/robot1/gnss_spoofed` | `PoseStamped` | GNSS position after meaconing injector (clean or spoofed) |
| `/robot2/gnss_spoofed` | `PoseStamped` | GNSS position after meaconing injector (clean or spoofed) |
| `/robots/uwb_distance` | `Float64` | Simulated UWB range between robots (m) |
| `/system/cusum_value` | `Float64` | Current CUSUM statistic $S_k$ |
| `/system/delta_value` | `Float64` | Current innovation $\delta_k = D_{UWB} - D_{GNSS}$ |
| `/system/meaconing_alert` | `Bool` | Detection alarm (`true` = meaconing detected) |
| `/meaconing/active` | `Bool` | Attack active status |
| `/robot1/cmd_vel` | `TwistStamped` | Velocity command for robot 1 |
| `/robot2/cmd_vel` | `TwistStamped` | Velocity command for robot 2 |

### Services

| Service | Type | Description |
|---|---|---|
| `/meaconing/set_active` | `SetBool` | Manually activate/deactivate the attack |
| `/system/reset_cusum` | `Trigger` | Reset CUSUM accumulator to zero |

---

## Prerequisites

- **macOS** (tested on Apple Silicon) or **Linux**
- **RoboStack** with ROS 2 Jazzy (via pixi)
- **Gazebo Sim** (Harmonic or Ionic, installed outside pixi)
- Python 3.12+

---

## Setup & Build

### 1. Activate the RoboStack environment

```bash
cd ~/robostack
pixi run -e jazzy
```

### 2. Build the workspace

```bash
cd ~/tfm_meaconing_ws
colcon build --packages-select collaborative_detection
source install/setup.bash
```

### 3. Launch the full experiment

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch collaborative_detection experiment.launch.py
```

This starts **everything** in sequence:

| t (s) | Event |
|---|---|
| 0 | Gazebo Sim server + GUI |
| 2–3 | Two TurtleBot3 robots spawn at (0,0) and (3,0) |
| 5 | GNSS + UWB simulators start publishing |
| 5.5 | Meaconing injector starts (passthrough mode) |
| 6 | CUSUM detector starts (5 s warmup) |
| 7 | Robots begin autonomous circular motion |
| 7.5 | GNSS visualization node starts (markers viewable in RViz2) |
| **30** | 🛑 Attack auto-activates (configurable) |

### 4. Verify the pipeline

In a second terminal (with `install/setup.bash` sourced):

```bash
# Check that sensors are publishing
ros2 topic echo /robots/uwb_distance    # Should show ~3 m
ros2 topic echo /system/cusum_value     # Should stay near 0 before attack

# Manually activate the attack (skip the 30 s wait)
ros2 service call /meaconing/set_active std_srvs/srv/SetBool "{data: true}"

# Watch the CUSUM statistic grow and trigger the alarm
ros2 topic echo /system/cusum_value
ros2 topic echo /system/meaconing_alert   # Should become true
```

---

## Experiments (E0–E4)

The script `scripts/run_experiments.sh` automates five experiments:

| Experiment | Parameter | Description |
|---|---|---|
| **E0 — Baseline** | `activation_delay: 9999.0` | No attack — validates **zero false positives** |
| **E1 — Slow drift** | `drift_velocity: 0.1` | Subtle attack, measures detection sensitivity |
| **E2 — Fast drift** | `drift_velocity: 0.5` | Obvious attack, measures minimum TTD |
| **E3 — Hot start** | `activation_delay: 0.0` | Attack active from the beginning |
| **E4 — Wide separation** | `x2: 5.0` | Robots 5 m apart — tests distance effect on TTD |

### Running all experiments

```bash
cd ~/tfm_meaconing_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
./src/collaborative_detection/scripts/run_experiments.sh
```

Each experiment runs for **90 seconds** and records a rosbag in `results/E*_<name>/`. The script automatically:

1. Modifies `params.yaml` for the experiment
2. Rebuilds the package
3. Launches Gazebo + all nodes
4. Records a rosbag with all relevant topics
5. Kills all processes cleanly
6. Saves a parameter snapshot

> **Total runtime**: ~10 minutes for all 5 experiments.

### Manual activation for ad-hoc tests

```bash
# Activate attack
ros2 service call /meaconing/set_active std_srvs/srv/SetBool "{data: true}"

# Deactivate
ros2 service call /meaconing/set_active std_srvs/srv/SetBool "{data: false}"

# Reset CUSUM (useful between tests)
ros2 service call /system/reset_cusum std_srvs/srv/Trigger
```

---

## Parameter Reference

All parameters live in `config/params.yaml` under the `/**` wildcard node.

| Parameter | Default | Description |
|---|---|---|
| `sigma_gnss` | `1.0` | GNSS noise standard deviation (m) |
| `sigma_uwb` | `0.24` | UWB noise standard deviation (m) |
| `beta` | `0.5` | CUSUM drift parameter (minimum detectable bias, m) |
| `tau` | `3.0` | CUSUM detection threshold |
| `startup_delay` | `5.0` | CUSUM warmup period before accumulation begins (s) |
| `drift_velocity` | `0.2` | Fake position drift speed during attack (m/s) |
| `activation_delay` | `30.0` | Auto-activation delay for the attack (s) |
| `attack_type` | `single_antenna` | Attack mode: `single_antenna` (implemented) or `pattern` (future) |
| `random_seed` | `42` | Fixed seed for NumPy reproducibility |
| `update_rate` | `30.0` | Sensor/CUSUM update frequency (Hz) |
| `robot1.x` / `robot1.y` | `0.0` / `0.0` | Robot 1 world spawn position (m) |
| `robot2.x` / `robot2.y` | `3.0` / `0.0` | Robot 2 world spawn position (m) |
| `robot1_linear_vel` | `0.15` | Robot 1 linear velocity (m/s) |
| `robot1_angular_vel` | `0.30` | Robot 1 angular velocity (rad/s) |
| `robot2_linear_vel` | `0.12` | Robot 2 linear velocity (m/s) |
| `robot2_angular_vel` | `0.25` | Robot 2 angular velocity (rad/s) |

---

## CUSUM Detector Calibration

The CUSUM detector is **conservatively calibrated** to avoid false positives:

- **β = 0.5**: Each innovation δ must exceed 0.5 m to contribute positively. Under H₀, $\delta \sim \mathcal{N}(0, \sigma_\delta^2)$ with $\sigma_\delta \approx 1.03$ m (from $\sigma_{GNSS}=1.0$, $\sigma_{UWB}=0.24$). Since $\mathbb{E}[\delta - \beta] = -0.5$, $S_k$ tends to **stay at zero**.

- **τ = 3.0**: Three consecutive positive spikes of ~1.5 m are needed to cross the threshold — probability < 0.03% under H₀.

Under H₁ (attack), $D_{GNSS} \approx 0$ and $D_{UWB} \approx 3$ m → $\delta \approx 3$ m → $S_k$ grows ~2.5 m/step → crosses $\tau$ in ~2 steps (~67 ms at 30 Hz).

**Experiment E0 empirically validates** that $S_k = 0$ for the full 90 s without attack.

---

## Visualization

### RViz2 Markers (recommended)

The `gnss_viz_node` publishes Marker spheres:

- 🔵 **Blue spheres** — clean GNSS positions (true robot locations)
- 🔴 **Red spheres** — spoofed GNSS positions (both converge to the same drifting point during attack)

To view them, open **RViz2** and add a `Marker` display subscribed to `/visualization/gnss_spoofed_robot1`.

### Gazebo

During the attack, Gazebo shows the robots continuing their normal circular motion. The meaconing is **invisible in the physical simulation** because it only affects sensor readings — exactly as in a real attack. The discrepancy only becomes apparent when comparing GNSS-derived distance against UWB-measured distance.

---

## Analysis

After running experiments, open the Jupyter notebook:

```bash
# From the ROS 2 Jazzy environment
cd ~/tfm_meaconing_ws/src/collaborative_detection/analysis
jupyter notebook plot_results.ipynb
```

The notebook automatically:

1. Discovers all experiment rosbags in `results/`
2. Loads time series for all topics using `rosbag2_py` (MCAP format)
3. Diagnoses which topics have data
4. Generates plots:
   - **CUSUM evolution** — $S_k$ over time for each experiment
   - **UWB distance** — physical inter-robot distance
   - **Fixed threshold vs CUSUM** — demonstrates sequential detector advantage
   - **Detection metrics** — TTD, false alarm count

> **Requires** the ROS 2 Jazzy environment (`pixi run -e jazzy`) for `rosbag2_py` and `rclpy`.

---

## Key Design Decisions

- **World-frame GNSS**: The DiffDrive plugin publishes odometry in a per-robot local frame starting at (0,0) regardless of world spawn position. The GNSS and UWB simulators add the known spawn offset to obtain world-frame coordinates, making $D_{GNSS}$ and $D_{UWB}$ directly comparable.

- **Per-robot Gazebo topics**: Each robot's SDF is dynamically patched at launch time to use model-specific transport topics (`/model/robot1/odom`, `/model/robot2/odom`, etc.), preventing the two bridges from receiving identical data from the shared global topics Gazebo uses by default.

- **Signed CUSUM innovation**: Using $\delta = D_{UWB} - D_{GNSS}$ (not absolute value) allows noise to cancel under H₀ — positive and negative noise samples average to zero — while the attack signal contributes a consistent positive bias.

- **Deterministic reproducibility**: A fixed `random_seed: 42` ensures identical noise sequences across runs, making experiments comparable.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ros_gz_sim not found` | Wrong ROS distro (Humble lacks `ros_gz_sim`) | Use `pixi run -e jazzy` |
| Robots not moving in Gazebo | Bridge topic mismatch | Ensure `two_robots.launch.py` uses per-robot SDF patching |
| UWB distance ≈ 0 | Both robots' odometry in local (0,0) frame | Check that GNSS/UWB nodes read `robot1.x`/`robot2.x` spawn offsets |
| CUSUM publishes nothing | Meaconing injector crashed | Check params for type errors (e.g. `9999` integer when float expected) |
| `colcon build` fails | Missing `resource/` marker | `touch src/collaborative_detection/resource/collaborative_detection` |
| Notebook shows no data | Ran outside ROS env | Launch Jupyter from `pixi run -e jazzy` |

---

## License

Apache 2.0 — see `package.xml`.

---
