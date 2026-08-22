#!/usr/bin/env python3
"""
Plot TFM meaconing-detection results from rosbags (headless — no Jupyter needed).

Run it with the jazzy Python, which has rosbag2_py + matplotlib:

    cd ~/tfm_meaconing_ws
    /Users/toni/robostack/.pixi/envs/jazzy/bin/python3.12 \
        src/collaborative_detection/analysis/plot_results.py

What it does:
  1. Auto-discovers every experiment in ~/tfm_meaconing_ws/results/ (E* or e*)
  2. Loads each rosbag (MCAP) and extracts time series
  3. Prints a diagnostics + TTD / false-alarm table to the terminal
  4. Saves PNG plots to ~/tfm_meaconing_ws/results/plots/
  5. For E5: compares attack trajectory vs reference, plots physical drift
"""

import os
from pathlib import Path
from collections import defaultdict

import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless backend — write PNGs instead of opening a window
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RESULTS_DIR = Path.home() / "tfm_meaconing_ws" / "results"
PLOTS_DIR = RESULTS_DIR / "plots"
TAU = 3.0  # CUSUM detection threshold (must match params.yaml)


def load_rosbag(bag_path: str) -> dict:
    """Load a rosbag2 (MCAP) and return {topic: {'time': np.array, 'value': np.array}}."""
    storage_options = StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    data = defaultdict(lambda: {"time": [], "value": []})
    t0 = None

    while reader.has_next():
        topic, msg_bytes, timestamp_ns = reader.read_next()
        ts = timestamp_ns / 1e9
        if t0 is None:
            t0 = ts

        if topic not in type_map:
            continue

        msg = deserialize_message(msg_bytes, get_message(type_map[topic]))
        data[topic]["time"].append(ts - t0)

        if hasattr(msg, "data"):
            # std_msgs/Float64, std_msgs/Bool
            val = msg.data
            val = 1.0 if isinstance(val, bool) and val else float(val)
            data[topic]["value"].append(float(val))
        elif hasattr(msg, "pose"):
            # PoseStamped -> msg.pose.position ; Odometry -> msg.pose.pose.position
            pos = getattr(msg.pose, "position", None) or msg.pose.pose.position
            data[topic]["value"].append(pos.x)
        else:
            data[topic]["value"].append(0.0)

    result = {}
    for topic, d in data.items():
        if len(d["time"]) > 0:
            result[topic] = {
                "time": np.array(d["time"]),
                "value": np.array(d["value"]),
            }
    return result


def load_odom_trajectory(bag_path: str, topic: str = "/robot1/odom") -> dict:
    """
    Load odometry positions (x, y) from a rosbag.

    Returns {'time': np.array, 'x': np.array, 'y': np.array}, or None on failure.
    Odometry uses msg.pose.pose.position, so we extract x and y separately.
    """
    storage_options = StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        print(f"  [odom] Topic {topic} not found in bag — skipping trajectory")
        return None

    times = []
    xs = []
    ys = []
    t0 = None

    while reader.has_next():
        t, msg_bytes, timestamp_ns = reader.read_next()
        if t != topic:
            continue
        ts = timestamp_ns / 1e9
        if t0 is None:
            t0 = ts

        msg = deserialize_message(msg_bytes, get_message(type_map[topic]))
        pos = msg.pose.pose.position
        times.append(ts - t0)
        xs.append(pos.x)
        ys.append(pos.y)

    if len(times) == 0:
        return None

    return {
        "time": np.array(times),
        "x": np.array(xs),
        "y": np.array(ys),
    }


def attack_start_time(data: dict):
    """Return the time (s) when /meaconing/active first became True, or None."""
    active = data.get("/meaconing/active")
    if active is None or len(active["time"]) == 0:
        return None
    idx = np.where(active["value"] > 0.5)[0]
    return float(active["time"][idx[0]]) if len(idx) > 0 else None


def alert_time(data: dict):
    """
    Return the time (s) when /system/meaconing_alert first became True (confirmed),
    or None if no alert was ever active.
    """
    alert = data.get("/system/meaconing_alert")
    if alert is None or len(alert["time"]) == 0:
        return None
    idx = np.where(alert["value"] > 0.5)[0]
    return float(alert["time"][idx[0]]) if len(idx) > 0 else None


def main():
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    exp_dirs = sorted(d for d in RESULTS_DIR.glob("[Ee]*") if d.is_dir())
    if not exp_dirs:
        print(f"No experiment folders found in {RESULTS_DIR}")
        print("Run an experiment first, e.g.:")
        print("  ./src/collaborative_detection/scripts/run_experiment.sh e1")
        return

    print(f"Found experiments: {[d.name for d in exp_dirs]}")

    experiments = {}
    for d in exp_dirs:
        print(f"\n--- Loading {d.name} ---")
        experiments[d.name] = load_rosbag(d)

    # ------------------------------------------------------------------ #
    # 1. Diagnostics: which topics have data                              #
    # ------------------------------------------------------------------ #
    print("\n" + "=" * 60)
    print("DIAGNOSTICS — topics with data")
    print("=" * 60)
    for name, data in experiments.items():
        with_data = [t for t, d in data.items() if len(d["time"]) > 0]
        print(f"  {name}: {with_data}")

    # ------------------------------------------------------------------ #
    # 2. S_k evolution (one subplot per experiment)                       #
    # ------------------------------------------------------------------ #
    n = len(experiments)
    fig, axes = plt.subplots(n, 1, figsize=(14, 3 * n), sharex=True, squeeze=False)
    axes = axes[:, 0]

    for ax, (name, data) in zip(axes, experiments.items()):
        cusum = data.get("/system/cusum_value")
        delta = data.get("/system/delta_value")
        alert = data.get("/system/meaconing_alert")
        t_attack = attack_start_time(data)

        if cusum and len(cusum["time"]) > 0:
            ax.plot(cusum["time"], cusum["value"], "b-", lw=1.2, label="S_k (CUSUM)")
        if delta and len(delta["time"]) > 0:
            ax.plot(delta["time"], delta["value"], "g-", lw=0.5, alpha=0.5,
                    label="delta (innovation)")
        if alert and len(alert["time"]) > 0:
            m = alert["value"] > 0.5
            if np.any(m):
                ax.fill_between(alert["time"], 0, TAU * 1.5, where=m,
                                alpha=0.2, color="red", label="Alarm active")

        ax.axhline(TAU, color="red", ls="--", lw=1.5, label=f"tau = {TAU}")
        if t_attack is not None:
            ax.axvline(t_attack, color="purple", ls=":", lw=1.5, label="Attack")
        ax.set_ylabel("Value")
        ax.set_title(name)
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-1, TAU * 2)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("CUSUM statistic S_k per experiment", fontsize=15)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = PLOTS_DIR / "cusum_evolution.png"
    fig.savefig(out, bbox_inches="tight", dpi=200)
    plt.close(fig)
    print(f"\nSaved: {out}")

    # ------------------------------------------------------------------ #
    # 3. UWB distance per experiment                                      #
    # ------------------------------------------------------------------ #
    for name, data in experiments.items():
        uwb = data.get("/robots/uwb_distance")
        if not uwb or len(uwb["time"]) == 0:
            continue
        t_attack = attack_start_time(data)
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(uwb["time"], uwb["value"], "b-", lw=1.0, alpha=0.8,
                label="D_UWB (physical)")
        if t_attack is not None:
            ax.axvline(t_attack, color="purple", ls=":", lw=1.5, label="Attack")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance (m)")
        ax.set_title(f"{name} — UWB distance")
        ax.legend()
        ax.grid(True, alpha=0.3)
        out = PLOTS_DIR / f"uwb_distance_{name}.png"
        fig.tight_layout()
        fig.savefig(out, bbox_inches="tight", dpi=150)
        plt.close(fig)
        print(f"Saved: {out}")

    # ------------------------------------------------------------------ #
    # 4. Fixed threshold vs CUSUM (delta + S_k)                           #
    # ------------------------------------------------------------------ #
    for name, data in experiments.items():
        cusum = data.get("/system/cusum_value")
        delta = data.get("/system/delta_value")
        if not (cusum and delta and len(cusum["time"]) > 0 and len(delta["time"]) > 0):
            print(f"{name}: no CUSUM/delta data — check the launch log")
            continue

        t_min = min(len(cusum["time"]), len(delta["time"]))
        t = cusum["time"][:t_min]
        d = delta["value"][:t_min]
        s = cusum["value"][:t_min]
        t_attack = attack_start_time(data)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)

        ax1.plot(t, d, lw=0.5, color="steelblue", alpha=0.7, label="delta(t)")
        ax1.axhline(2.0, color="orange", ls="--", lw=1.5, label="Fixed thr = 2.0 m")
        ax1.set_ylabel("delta(t) (m)")
        ax1.set_title(f"{name} — Fixed threshold")
        ax1.legend(loc="upper left")
        ax1.grid(True, alpha=0.3)

        ax2.plot(t, s, lw=1.2, color="darkred", label="S_k (CUSUM)")
        ax2.axhline(TAU, color="red", ls="--", lw=1.5, label=f"tau = {TAU}")
        ax2.fill_between(t, 0, TAU, alpha=0.05, color="green")
        top = max(s) * 1.1 if max(s) > TAU else TAU * 2
        ax2.fill_between(t, TAU, top, alpha=0.05, color="red")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("S_k")
        ax2.set_title(f"{name} — CUSUM detector")
        ax2.legend(loc="upper left")
        ax2.grid(True, alpha=0.3)

        if t_attack is not None:
            for ax in (ax1, ax2):
                ax.axvline(t_attack, color="purple", ls=":", lw=1.5)

        out = PLOTS_DIR / f"threshold_vs_cusum_{name}.png"
        fig.tight_layout()
        fig.savefig(out, bbox_inches="tight", dpi=150)
        plt.close(fig)
        print(f"Saved: {out}")

    # ------------------------------------------------------------------ #
    # 5. TTD + false-alarm table                                          #
    # ------------------------------------------------------------------ #
    print("\n" + "=" * 60)
    print("DETECTION METRICS")
    print("=" * 60)
    print(f"{'Experiment':<22} {'Attack t':>9}  {'TTD (s)':>9}  {'False alarms':>13}")
    print("-" * 60)

    for name, data in experiments.items():
        cusum = data.get("/system/cusum_value")
        alert = data.get("/system/meaconing_alert")
        t_attack = attack_start_time(data)

        if not (cusum and len(cusum["time"]) > 0):
            print(f"{name:<22} {'—':>9}  {'NO DATA':>9}  {'—':>13}")
            continue

        # TTD: time of the first *confirmed* alarm relative to the attack.
        ttd = None
        if alert and len(alert["time"]) > 0 and np.any(alert["value"] > 0.5):
            first = float(alert["time"][np.where(alert["value"] > 0.5)[0][0]])
            if t_attack is not None:
                ttd = first - t_attack

        # False alarms: confirmed alerts before the attack (or all if no attack)
        n_false = 0
        if alert and len(alert["time"]) > 0:
            if t_attack is not None:
                n_false = int(np.sum(alert["value"][alert["time"] < t_attack] > 0.5))
            else:
                n_false = int(np.sum(alert["value"] > 0.5))

        t_attack_str = f"{t_attack:.1f}s" if t_attack is not None else "never"
        ttd_str = f"{ttd:.2f}s" if ttd is not None else "N/A"
        print(f"{name:<22} {t_attack_str:>9}  {ttd_str:>9}  {n_false:>13}")

    # ------------------------------------------------------------------ #
    # 6. E5: Physical drift (attack trajectory vs reference trajectory)   #
    # ------------------------------------------------------------------ #
    e5_attack_dirs = [d for d in exp_dirs if d.name.startswith("e5_") and "ref" not in d.name]
    e5_ref_dir = None
    for d in exp_dirs:
        if "e5_ref" in d.name or "reference" in d.name:
            e5_ref_dir = d
            break

    if e5_attack_dirs and e5_ref_dir is not None:
        print("\n" + "=" * 60)
        print("E5 — PHYSICAL DRIFT ANALYSIS")
        print("=" * 60)

        ref_traj = load_odom_trajectory(e5_ref_dir, "/robot1/odom")
        if ref_traj is None:
            print("  Could not load reference trajectory from e5_ref — skipping E5 drift plot")
        else:
            for e5d in e5_attack_dirs:
                atk_traj = load_odom_trajectory(e5d, "/robot1/odom")
                atk_data = experiments.get(e5d.name, {})
                if atk_traj is None:
                    print(f"  Could not load attack odom from {e5d.name} — skipping")
                    continue

                t_attack = attack_start_time(atk_data)
                t_alert = alert_time(atk_data)

                # Match by relative timestamp: interpolate ref onto attack time grid
                # Both start at t≈0 but may have different lengths / sampling rates
                common_len = min(len(ref_traj["time"]), len(atk_traj["time"]))
                t_common = atk_traj["time"][:common_len]

                # Interpolate reference onto attack time grid
                ref_x = np.interp(t_common, ref_traj["time"], ref_traj["x"])
                ref_y = np.interp(t_common, ref_traj["time"], ref_traj["y"])

                atk_x = atk_traj["x"][:common_len]
                atk_y = atk_traj["y"][:common_len]

                drift = np.sqrt((atk_x - ref_x) ** 2 + (atk_y - ref_y) ** 2)

                # --- Plot: drift vs time ---
                fig, ax = plt.subplots(figsize=(14, 5))
                ax.plot(t_common, drift, "b-", lw=1.2, label="Physical drift (m)")

                if t_attack is not None:
                    ax.axvline(t_attack, color="purple", ls=":", lw=2.0,
                               label="Attack activated")
                if t_alert is not None:
                    ax.axvline(t_alert, color="red", ls="--", lw=2.0,
                               label="CUSUM alert")

                # Annotate the drift value at detection time
                if t_alert is not None:
                    idx_alert = np.searchsorted(t_common, t_alert)
                    if idx_alert < len(drift):
                        drift_at_alert = drift[idx_alert]
                        ax.axhline(drift_at_alert, color="red", ls=":", lw=0.8, alpha=0.5)
                        ax.annotate(
                            f"Drift at detection: {drift_at_alert:.2f} m",
                            xy=(t_alert, drift_at_alert),
                            xytext=(t_alert + 2, drift_at_alert + 0.1),
                            arrowprops=dict(arrowstyle="->", color="red"),
                            fontsize=10,
                            color="red",
                        )

                ax.set_xlabel("Time (s)")
                ax.set_ylabel("Physical drift (m)")
                ax.set_title(
                    f"{e5d.name} — Physical deviation from reference trajectory\n"
                    f"(Waypoint follower under meaconing attack)"
                )
                ax.legend(loc="upper left")
                ax.grid(True, alpha=0.3)

                out = PLOTS_DIR / f"e5_physical_drift_{e5d.name}.png"
                fig.tight_layout()
                fig.savefig(out, bbox_inches="tight", dpi=150)
                plt.close(fig)
                print(f"Saved: {out}")

                # --- Also plot the trajectories side by side (top-down view) ---
                fig, ax = plt.subplots(figsize=(10, 8))
                ax.plot(ref_x, ref_y, "b-", lw=1.0, alpha=0.7, label="Reference (no attack)")
                ax.plot(atk_x, atk_y, "r-", lw=1.0, alpha=0.7, label="Attack (spoofed GNSS)")

                # Mark start points
                ax.scatter(ref_x[0], ref_y[0], c="blue", marker="o", s=60, zorder=5,
                           label="Start (ref)")
                ax.scatter(atk_x[0], atk_y[0], c="red", marker="o", s=60, zorder=5,
                           label="Start (attack)")

                # Mark positions at attack time and alert time
                if t_attack is not None:
                    ia = np.searchsorted(t_common, t_attack)
                    if ia < common_len:
                        ax.scatter(atk_x[ia], atk_y[ia], c="purple", marker="X", s=100,
                                   zorder=6, label="Attack activated")
                if t_alert is not None:
                    ib = np.searchsorted(t_common, t_alert)
                    if ib < common_len:
                        ax.scatter(atk_x[ib], atk_y[ib], c="darkred", marker="D", s=100,
                                   zorder=6, label="CUSUM alert")

                ax.set_xlabel("World X (m)")
                ax.set_ylabel("World Y (m)")
                ax.set_title(f"{e5d.name} — Robot trajectories (top-down)")
                ax.legend(loc="best")
                ax.set_aspect("equal")
                ax.grid(True, alpha=0.3)

                out = PLOTS_DIR / f"e5_trajectories_{e5d.name}.png"
                fig.tight_layout()
                fig.savefig(out, bbox_inches="tight", dpi=150)
                plt.close(fig)
                print(f"Saved: {out}")

                # --- Summary ---
                drift_final = drift[-1] if len(drift) > 0 else 0.0
                drift_max = float(np.max(drift))
                print(f"  {e5d.name}:")
                print(f"    Max drift:          {drift_max:.3f} m")
                print(f"    Final drift:        {drift_final:.3f} m")
                if t_attack is not None and t_alert is not None:
                    ia = np.searchsorted(t_common, t_attack)
                    ib = np.searchsorted(t_common, t_alert)
                    if ia < len(drift) and ib < len(drift):
                        print(f"    Drift at attack:    {drift[ia]:.3f} m")
                        print(f"    Drift at detection: {drift[ib]:.3f} m")
                print(f"    Attack → alert lag: "
                      f"{t_alert - t_attack:.2f} s" if (t_attack and t_alert) else "    Attack → alert: N/A")

    elif e5_attack_dirs and e5_ref_dir is None:
        print("\n[E5] No reference trajectory found — run e5_ref first, then e5")

    print(f"\nPlots written to: {PLOTS_DIR}")


if __name__ == "__main__":
    main()