# Experiments Guide — Collaborative GNSS Meaconing Detection

**Master's Thesis (TFM)** — Security Architecture for Autonomous Robot Navigation
Antonio García Alcón — Universidad Europea de Madrid, 2026

---

## 1. Environment setup

```bash
# 1. Activate RoboStack (Jazzy)
conda activate base
cd ~/robostack
pixi run -e jazzy

# 2. One-time build. run_experiment.sh rebuilds automatically for each
#    experiment, so this is only needed once to generate install/setup.bash.
cd ~/tfm_meaconing_ws
colcon build --packages-select collaborative_detection
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
```

---

## 2. Quick smoke test (verify everything works)

The smoke test is **interactive**: you launch the full pipeline with the GUI,
watch Gazebo, and trigger the attack manually. The batch experiments (Section 3)
use `run_experiment.sh` instead, which runs headless and tears everything down.

### 2.1 Launch the pipeline

```bash
ros2 launch collaborative_detection experiment.launch.py
```

Gazebo opens with 2 TurtleBots. After ~7 s they start moving in circles.

### 2.2 Verify topics (in a second terminal)

```bash
# Terminal 2 — source the environment first
source ~/tfm_meaconing_ws/install/setup.bash

# List all topics
ros2 topic list
# You should see: /robot1/gnss_clean, /robot2/gnss_spoofed,
#                 /robots/uwb_distance, /system/cusum_value,
#                 /system/meaconing_alert, /meaconing/active, etc.

# Monitor the CUSUM statistic (should hover near 0)
ros2 topic echo /system/cusum_value

# Alarm (should be False with no attack)
ros2 topic echo /system/meaconing_alert
```

### 2.3 Activate the attack manually (skip the 30 s wait)

```bash
ros2 service call /meaconing/set_active std_srvs/srv/SetBool "{data: true}"
```

**Expected result:**

- `S_k` starts rising monotonically
- Within ~2-10 s (depending on `drift_velocity`), `S_k` crosses `tau`
- The `🚨 MEACONING DETECTED!` alarm fires ~2 s **after** the crossing, due to
  the confirmation window (`alert_confirm_time: 2.0`) that ensures it is a real
  attack and not a transient

### 2.4 Reset between tests

```bash
ros2 service call /system/reset_cusum std_srvs/srv/Trigger
ros2 service call /meaconing/set_active std_srvs/srv/SetBool "{data: false}"
```

---

## 3. Experiment scenarios (E0–E4)

Run each experiment **one at a time** with `run_experiment.sh`. The script handles
the whole lifecycle: clean slate → set parameters → build → sync params → launch
(headless) → record rosbag → tear down.

```bash
cd ~/tfm_meaconing_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
./src/collaborative_detection/scripts/run_experiment.sh e1
```

Flags:

- `--duration N` — run length in seconds (default 90)
- `--gui` — keep the Gazebo GUI (headless by default)

> **Note on the alarm:** the CUSUM detector fires `🚨 MEACONING DETECTED!` ~2 s
> after `S_k` crosses `tau`, due to the confirmation window
> (`alert_confirm_time: 2.0`). This delay is intentional: it rejects transients
> and confirms a real attack.

| Experiment | Command | Parameter changes | Description |
|---|---|---|---|
| **E0 — Baseline** | `run_experiment.sh e0` | `activation_delay: 9999.0` | No attack — validates zero false positives |
| **E1 — Slow drift** | `run_experiment.sh e1` | `drift_velocity: 0.1` | Subtle attack, measures detection sensitivity |
| **E2 — Fast drift** | `run_experiment.sh e2` | `drift_velocity: 0.5` | Obvious attack, measures minimum TTD |
| **E3 — Hot start** | `run_experiment.sh e3` | `activation_delay: 2.0` | Attack active from the beginning |
| **E4 — Wide separation** | `run_experiment.sh e4` | `robot2.x: 5.0` | Robots 5 m apart — tests distance effect on TTD |

### E0 — Baseline (no attack, measure FAR)

**Objective:** verify there are no false alarms without an attack.

```bash
./src/collaborative_detection/scripts/run_experiment.sh e0
```

The script sets `activation_delay: 9999.0` (the attack never auto-activates),
records ~90 s to `results/e0_baseline/`, and tears everything down.

### E1 — Slow drift (measure TTD with a subtle attack)

**Objective:** measure time-to-detect with a slow drift.

```bash
./src/collaborative_detection/scripts/run_experiment.sh e1
```

Sets `drift_velocity: 0.1` m/s and `activation_delay: 30.0`. The attack drags the
reported GNSS positions toward a common fake point at 0.1 m/s, so `D_GNSS`
collapses slowly and the CUSUM rises gradually.

### E2 — Fast drift (measure TTD with an obvious attack)

**Objective:** measure time-to-detect with a fast drift.

```bash
./src/collaborative_detection/scripts/run_experiment.sh e2
```

Sets `drift_velocity: 0.5` m/s. The faster drag collapses `D_GNSS` sooner, so the
CUSUM crosses `tau` earlier than in E1.

### E3 — Hot start (attack from t=0)

**Objective:** measure TTD when the system starts already under attack.

```bash
./src/collaborative_detection/scripts/run_experiment.sh e3
```

Sets `activation_delay: 2.0` (attack active almost immediately) and a shorter
`startup_delay: 3.0` so the detector starts accumulating early.

### E4 — Wide separation

**Objective:** see whether the initial inter-robot distance affects TTD.

```bash
./src/collaborative_detection/scripts/run_experiment.sh e4
```

Spawns robot2 at x = 5.0 (5 m apart, instead of 3 m) and passes `x2:=5.0` to the
launch file.

---

## 4. τ Sweep (ROC curve)

To build the ROC curve (TTD vs FAR), repeat E1 while varying `tau`. The script
does not sweep `tau` directly, but it preserves any value you set in `params.yaml`
(it only overrides the experiment-specific keys listed above). For each `tau`:

```bash
# 1. Edit src/collaborative_detection/config/params.yaml → tau: <value>
# 2. Run E1
./src/collaborative_detection/scripts/run_experiment.sh e1
# 3. Keep the rosbag by renaming its folder
mv results/e1_slow_drift results/e1_tau_5.0
```

| τ | Expected |
|---|---|
| 0.5 | Very fast TTD, high FAR |
| 1.0 | Fast TTD, moderate FAR |
| 2.0 | Balanced |
| 5.0 | Slow TTD, low FAR |
| 10.0 | Very slow TTD, FAR ~0 |

---

## 5. Offline analysis

After the rosbags are recorded, generate the plots and metrics with
`plot_results.py` (headless — no Jupyter needed):

```bash
cd ~/tfm_meaconing_ws
source install/setup.bash
python3 src/collaborative_detection/analysis/plot_results.py
```

> Run this from the Jazzy environment, whose Python has `rosbag2_py`,
> `matplotlib` and `numpy`.

The script:

1. Loads every rosbag in `results/` with `rosbag2_py`
2. Extracts the time series for `S_k`, `δ`, and the alarms
3. Computes TTD (time to the first confirmed alert after activation)
4. Computes the false-alarm count during the no-attack period
5. Generates the PNG plots in `results/plots/`

---

## 6. Key parameter summary

| Parameter | Default | Meaning |
|---|---|---|
| `sigma_gnss` | 1.0 | GNSS noise (m) — civil GPS with SBAS |
| `sigma_uwb` | 0.24 | UWB noise (m) — Fishberg 2024 |
| `beta` | 0.5 | Minimum detectable CUSUM bias (m) |
| `tau` | 3.0 | CUSUM alarm threshold |
| `filter_window` | 30 | Moving-average window over δ (samples, 30 ≈ 1 s @ 30 Hz) |
| `alert_confirm_time` | 2.0 | Seconds S_k must stay above τ before the alarm fires |
| `startup_delay` | 10.0 | Warmup seconds (from the first data sample) before accumulating CUSUM |
| `drift_velocity` | 0.2 | Attack drag speed (m/s) |
| `activation_delay` | 30.0 | Time until auto-activation (s) |
| `attack_type` | single_antenna | Attack type |
| `random_seed` | 42 | NumPy seed (reproducibility) |
| `robot1_linear_vel` | 0.15 | Robot 1 linear velocity (m/s) |
| `robot1_angular_vel` | 0.30 | Robot 1 angular velocity (rad/s) |
| `robot2_linear_vel` | 0.12 | Robot 2 linear velocity (m/s) |
| `robot2_angular_vel` | 0.25 | Robot 2 angular velocity (rad/s) |

---

## 7. Troubleshooting

| Problem | Solution |
|---|---|
| `ros_gz_sim` not found | Use `pixi run -e jazzy` (not humble) |
| `turtlebot3_gazebo` not found | Add `ros-jazzy-turtlebot3-simulations` to `pixi.toml` |
| `oneshot` error in `create_timer` | Already fixed — uses `timer.cancel()` instead |
| OGRE rendering errors | Cosmetic on macOS — does not affect the experiment |
| Thread affinity warnings | Normal on macOS with DDS — ignore |
| Gazebo shows no GUI | Verify `gz sim` is installed (`brew install gz-harmonic`) |

---
