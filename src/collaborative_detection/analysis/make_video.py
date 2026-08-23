#!/usr/bin/env python3
"""
Make synchronized experiment video from rosbag data.

Supports E5 (waypoint_attack) and E6 (dual_meaconing).

Generates an MP4 with three synchronized panels:
  1. CUSUM S_k + delta (innovation) with a moving time cursor
  2. Physical drift ‖p_real(t) − p_ref(t)‖ with attack/alert markers + TTD
  3. Top-down robot trajectories with multi-waypoint route markers

Usage:
    cd ~/tfm_meaconing_ws
    /path/to/jazzy/python3 \
        src/collaborative_detection/analysis/make_video.py e5_waypoint_attack
    # or
    /path/to/jazzy/python3 \
        src/collaborative_detection/analysis/make_video.py e6_dual_meaconing

Output:
    ~/tfm_meaconing_ws/results/videos/e5_experiment.mp4
    ~/tfm_meaconing_ws/results/videos/e6_experiment.mp4
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RESULTS_DIR = Path.home() / "tfm_meaconing_ws" / "results"
VIDEOS_DIR = RESULTS_DIR / "videos"
TAU = 3.0

# Waypoint routes for each experiment type
EXP_CONFIG = {
    "e5": {
        "name": "waypoint_attack",
        "ref_name": "waypoint_reference",
        "wp_r1": [(5.0, 0.0), (5.0, 5.0), (0.0, 5.0)],
        "wp_r2": [(5.0, 2.0), (5.0, 7.0), (0.0, 7.0)],
        "r1_uses_spoofed": True,
        "r2_uses_spoofed": False,
        "r2_y_offset": 2.0,
    },
    "e6": {
        "name": "dual_meaconing",
        "ref_name": "waypoint_reference",
        "ref_exp_type": "e5",
        "wp_r1": [(5.0, 0.0), (5.0, 5.0), (0.0, 5.0)],
        "wp_r2": [(5.0, 2.0), (5.0, 7.0), (0.0, 7.0)],
        "r1_uses_spoofed": True,
        "r2_uses_spoofed": True,
        "r2_y_offset": 2.0,
    },
}


def _find_global_t0(bag_path, topics):
    """Find the earliest timestamp across all specified topics in a bag."""
    storage_options = StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    min_ts = None

    while reader.has_next():
        t, msg_bytes, ts_ns = reader.read_next()
        if t in topics:
            ts = ts_ns / 1e9
            if min_ts is None or ts < min_ts:
                min_ts = ts
    return min_ts


def _load_odom_trajectory(bag_path, topic, t0):
    """Load (x, y, time) from an Odometry topic using a common t0. Returns dict or None."""
    storage_options = StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        return None

    times, xs, ys = [], [], []

    while reader.has_next():
        t, msg_bytes, ts_ns = reader.read_next()
        if t != topic:
            continue
        ts = ts_ns / 1e9
        msg = deserialize_message(msg_bytes, get_message(type_map[topic]))
        pos = msg.pose.pose.position
        times.append(ts - t0)
        xs.append(pos.x)
        ys.append(pos.y)

    if not times:
        return None
    return {"time": np.array(times), "x": np.array(xs), "y": np.array(ys)}


def _load_scalar(bag_path, topic, t0):
    """Load a scalar topic (Float64 or Bool) → {time, value} using a common t0. Returns None if empty."""
    storage_options = StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        return None

    times, vals = [], []

    while reader.has_next():
        t, msg_bytes, ts_ns = reader.read_next()
        if t != topic:
            continue
        ts = ts_ns / 1e9
        msg = deserialize_message(msg_bytes, get_message(type_map[t]))
        v = msg.data
        v = float(1.0 if isinstance(v, bool) and v else v)
        times.append(ts - t0)
        vals.append(v)

    if not times:
        return None
    return {"time": np.array(times), "value": np.array(vals)}


def _detect_experiment(exp_folder):
    """Detect if folder is e5 or e6 based on name."""
    exp_lower = exp_folder.lower()
    if exp_lower.startswith("e5"):
        return "e5"
    elif exp_lower.startswith("e6"):
        return "e6"
    else:
        # Default to e5 for backwards compatibility
        return "e5"


def main():
    parser = argparse.ArgumentParser(description="Generate experiment video (E5 or E6)")
    parser.add_argument("experiment",
                        help="Experiment folder name, e.g. e5_waypoint_attack or e6_dual_meaconing")
    parser.add_argument("--speedup", type=float, default=2.0)
    parser.add_argument("--max-time", type=float, default=None)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--dpi", type=int, default=150)
    args = parser.parse_args()

    # Detect experiment type from folder name
    exp_type = _detect_experiment(args.experiment)
    config = EXP_CONFIG[exp_type]

    # Locate rosbags
    atk_dir = RESULTS_DIR / args.experiment
    ref_exp_type = config.get('ref_exp_type', exp_type)
    ref_dir = RESULTS_DIR / f"{ref_exp_type}_ref_{config['ref_name']}"

    if not atk_dir.is_dir():
        print(f"Attack bag not found: {atk_dir}", file=sys.stderr)
        for d in sorted(RESULTS_DIR.glob("*")):
            if d.is_dir():
                print(f"  {d.name}", file=sys.stderr)
        sys.exit(1)
    if not ref_dir.is_dir():
        print(f"Reference bag not found: {ref_dir}", file=sys.stderr)
        print(f"  Expected: {ref_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Experiment type: {exp_type.upper()} ({config['name']})")
    print(f"Attack bag:    {atk_dir}")
    print(f"Reference bag: {ref_dir}")

    # --- Find global t0 for attack bag ---
    atk_topics = [
        "/system/cusum_value", "/system/delta_value", "/system/meaconing_alert",
        "/meaconing/active", "/meaconing/activation_event",
        "/robot1/odom", "/robot2/odom",
    ]
    atk_t0 = _find_global_t0(atk_dir, atk_topics)
    if atk_t0 is None:
        print("Could not determine attack bag start time", file=sys.stderr)
        sys.exit(1)
    print(f"Attack bag t0: {atk_t0:.3f}")

    # --- Find global t0 for reference bag ---
    ref_topics = ["/robot1/odom", "/robot2/odom"]
    ref_t0 = _find_global_t0(ref_dir, ref_topics)
    if ref_t0 is None:
        print("Could not determine reference bag start time", file=sys.stderr)
        sys.exit(1)
    print(f"Reference bag t0: {ref_t0:.3f}")

    # --- Load data with common t0 ---
    print("Loading attack data ...")
    cusum = _load_scalar(atk_dir, "/system/cusum_value", atk_t0)
    delta_f = _load_scalar(atk_dir, "/system/delta_value", atk_t0)
    alert = _load_scalar(atk_dir, "/system/meaconing_alert", atk_t0)
    active = _load_scalar(atk_dir, "/meaconing/active", atk_t0)
    activation_event = _load_scalar(atk_dir, "/meaconing/activation_event", atk_t0)
    odom_r1 = _load_odom_trajectory(atk_dir, "/robot1/odom", atk_t0)
    odom_r2 = _load_odom_trajectory(atk_dir, "/robot2/odom", atk_t0)

    print("Loading reference data ...")
    ref_odom_r1 = _load_odom_trajectory(ref_dir, "/robot1/odom", ref_t0)
    ref_odom_r2 = _load_odom_trajectory(ref_dir, "/robot2/odom", ref_t0)

    if cusum is None or delta_f is None:
        print("CUSUM/delta data missing", file=sys.stderr)
        sys.exit(1)
    if odom_r1 is None or ref_odom_r1 is None:
        print("Robot1 odometry data missing", file=sys.stderr)
        sys.exit(1)

    # --- Time window ---
    t_end = max(cusum["time"][-1], odom_r1["time"][-1])
    if args.max_time is not None:
        t_end = min(t_end, args.max_time)

    print(f"Simulation time: 0 → {t_end:.1f} s")
    print(f"Video duration:  {t_end / args.speedup:.1f} s  (×{args.speedup})")

    # --- Uniform time grid ---
    n_frames = int(t_end * args.fps / args.speedup)
    dt_frame = args.speedup / args.fps
    t_video = np.arange(n_frames) * dt_frame

    def _interp(series, t_target):
        if series is None or len(series["time"]) == 0:
            return np.full_like(t_target, np.nan)
        return np.interp(t_target, series["time"], series["value"],
                         left=np.nan, right=np.nan)

    s_k = _interp(cusum, t_video)
    d_f = _interp(delta_f, t_video)
    alert_v = _interp(alert, t_video)
    active_v = _interp(active, t_video)

    # Robot1 odom (attack experiment)
    r1_x = np.interp(t_video, odom_r1["time"], odom_r1["x"],
                     left=np.nan, right=np.nan)
    r1_y = np.interp(t_video, odom_r1["time"], odom_r1["y"],
                     left=np.nan, right=np.nan)

    # Robot2 odom (attack experiment)
    if odom_r2 is not None:
        r2_x = np.interp(t_video, odom_r2["time"], odom_r2["x"],
                         left=np.nan, right=np.nan)
        r2_y = np.interp(t_video, odom_r2["time"], odom_r2["y"],
                         left=np.nan, right=np.nan) + config["r2_y_offset"]
        has_r2 = True
    else:
        r2_x = np.full_like(t_video, np.nan)
        r2_y = np.full_like(t_video, np.nan)
        has_r2 = False

    # Reference trajectories (no attack)
    ref_r1_x = np.interp(t_video, ref_odom_r1["time"], ref_odom_r1["x"],
                         left=np.nan, right=np.nan)
    ref_r1_y = np.interp(t_video, ref_odom_r1["time"], ref_odom_r1["y"],
                         left=np.nan, right=np.nan)

    if ref_odom_r2 is not None:
        ref_r2_x = np.interp(t_video, ref_odom_r2["time"], ref_odom_r2["x"],
                             left=np.nan, right=np.nan)
        ref_r2_y = np.interp(t_video, ref_odom_r2["time"], ref_odom_r2["y"],
                             left=np.nan, right=np.nan) + config["r2_y_offset"]
        has_ref_r2 = True
    else:
        ref_r2_x = np.full_like(t_video, np.nan)
        ref_r2_y = np.full_like(t_video, np.nan)
        has_ref_r2 = False

    # --- Compute drift ---
    # Robot1 always compares to its reference
    drift_r1 = np.sqrt((r1_x - ref_r1_x) ** 2 + (r1_y - ref_r1_y) ** 2)

    # Robot2 drift depends on experiment type
    if has_r2 and has_ref_r2:
        drift_r2 = np.sqrt((r2_x - ref_r2_x) ** 2 + (r2_y - ref_r2_y) ** 2)
    else:
        drift_r2 = np.full_like(t_video, np.nan)

    # For the drift plot, show the relevant robot(s)
    if config["r2_uses_spoofed"]:
        # E6: both robots are meaconed, show both drifts
        drift = np.minimum(drift_r1, drift_r2)  # or could show max/average
        drift_label = "Drift (min of R1,R2)"
    else:
        # E5: only robot1 is meaconed
        drift = drift_r1
        drift_label = "Robot1 drift"

    # Attack / alert times
    t_attack = None
    if activation_event is not None and len(activation_event["time"]) > 0:
        t_attack = float(activation_event["time"][0])
    elif active is not None:
        idx = np.where(active["value"] > 0.5)[0]
        if len(idx) > 0:
            t_attack = float(active["time"][idx[0]])

    t_alert = None
    if alert is not None:
        idx = np.where(alert["value"] > 0.5)[0]
        if len(idx) > 0:
            t_alert = float(alert["time"][idx[0]])

    ttd = (t_alert - t_attack) if (t_attack and t_alert) else None

    print(f"Attack time:  {t_attack:.1f}s" if t_attack else "Attack time:  never")
    print(f"Alert time:   {t_alert:.1f}s" if t_alert else "Alert time:   never")
    print(f"TTD:          {ttd:.2f}s" if ttd else "TTD:          N/A")

    # ---------------------------------------------------------------------- #
    #  Figure setup                                                          #
    # ---------------------------------------------------------------------- #
    fig = plt.figure(figsize=(20, 10), dpi=args.dpi)
    gs = GridSpec(2, 2, figure=fig,
                  width_ratios=[1.2, 1.0],
                  height_ratios=[1, 1],
                  hspace=0.35, wspace=0.30)

    ax_cusum = fig.add_subplot(gs[:, 0])
    ax_drift = fig.add_subplot(gs[0, 1])
    ax_traj = fig.add_subplot(gs[1, 1])

    # --- Left panel: CUSUM + delta ---
    ax_cusum.set_title(f"CUSUM detector ({exp_type.upper()})", fontsize=13, fontweight="bold")
    ax_cusum.set_xlabel("Time (s)")
    ax_cusum.set_ylabel(r"$S_k$ / $\delta$ (m)", color="black")
    ax_cusum.set_xlim(0, t_end)
    ax_cusum.set_ylim(-1, max(TAU * 2, np.nanmax(s_k) * 1.1 + 0.5))
    ax_cusum.axhline(TAU, color="red", ls="--", lw=1.5, label=f"τ = {TAU}")
    ax_cusum.grid(True, alpha=0.3)
    ax_cusum.plot(t_video, s_k, color="steelblue", lw=0.6, alpha=0.4)
    ax_cusum.plot(t_video, d_f, color="green", lw=0.4, alpha=0.3)
    if t_attack is not None:
        ax_cusum.axvline(t_attack, color="purple", ls=":", lw=1.5, alpha=0.6)
    if t_alert is not None:
        ax_cusum.axvline(t_alert, color="red", ls="--", lw=1.5, alpha=0.6)

    cusum_line, = ax_cusum.plot([], [], "b-", lw=1.8, label=r"$S_k$ (CUSUM)")
    delta_line, = ax_cusum.plot([], [], "g-", lw=0.8, alpha=0.7,
                                label=r"$\delta$ (innovation)")
    cursor_line = ax_cusum.axvline(0, color="orange", lw=2.0, alpha=0.8)
    time_text = ax_cusum.text(0.02, 0.96, "", transform=ax_cusum.transAxes,
                              fontsize=10, va="top", fontfamily="monospace",
                              bbox=dict(boxstyle="round", facecolor="wheat",
                                        alpha=0.8))
    ax_cusum.legend(loc="upper right", fontsize=8)

    # --- Right-top: Physical drift ---
    ax_drift.set_title(rf"Physical drift $\|\mathbf{{p}}(t) - \mathbf{{p}}_{{ref}}(t)\|$ ({drift_label})",
                       fontsize=13, fontweight="bold")
    ax_drift.set_xlabel("Time (s)")
    ax_drift.set_ylabel("Drift (m)")
    ax_drift.set_xlim(0, t_end)
    drift_max = max(np.nanmax(drift) * 1.1, 0.5)
    ax_drift.set_ylim(-0.05 * drift_max, drift_max)
    ax_drift.grid(True, alpha=0.3)
    ax_drift.plot(t_video, drift, color="darkblue", lw=0.6, alpha=0.3)
    if t_attack is not None:
        ax_drift.axvline(t_attack, color="purple", ls=":", lw=1.5, alpha=0.6,
                         label="Attack activated")
    if t_alert is not None:
        ax_drift.axvline(t_alert, color="red", ls="--", lw=1.5, alpha=0.6,
                         label="CUSUM alert")
    ax_drift.legend(loc="lower right", fontsize=7.5)

    drift_line, = ax_drift.plot([], [], "b-", lw=1.5)
    drift_cursor = ax_drift.axvline(0, color="orange", lw=2.0, alpha=0.8)
    drift_info = [""]
    if ttd is not None:
        drift_info.append(f"TTD = {ttd:.2f} s")
    drift_val_text = ax_drift.text(0.98, 0.92, "\n".join(drift_info),
                                   transform=ax_drift.transAxes,
                                   fontsize=11, ha="right", va="top",
                                   fontfamily="monospace", fontweight="bold",
                                   bbox=dict(boxstyle="round", facecolor="white",
                                             alpha=0.8))

    # --- Right-bottom: Trajectories ---
    ax_traj.set_title("Robot trajectories (top-down)", fontsize=13, fontweight="bold")
    ax_traj.set_xlabel("World X (m)")
    ax_traj.set_ylabel("World Y (m)")
    ax_traj.set_aspect("equal")
    ax_traj.grid(True, alpha=0.3)

    # Collect all points for bounds
    all_x = np.concatenate([r1_x[~np.isnan(r1_x)], ref_r1_x[~np.isnan(ref_r1_x)]])
    all_y = np.concatenate([r1_y[~np.isnan(r1_y)], ref_r1_y[~np.isnan(ref_r1_y)]])
    if has_r2:
        all_x = np.concatenate([all_x, r2_x[~np.isnan(r2_x)]])
        all_y = np.concatenate([all_y, r2_y[~np.isnan(r2_y)]])
    if has_ref_r2:
        all_x = np.concatenate([all_x, ref_r2_x[~np.isnan(ref_r2_x)]])
        all_y = np.concatenate([all_y, ref_r2_y[~np.isnan(ref_r2_y)]])

    # Include waypoints in bounds
    all_wp = config["wp_r1"] + config["wp_r2"]
    wp_xs = [w[0] for w in all_wp]
    wp_ys = [w[1] for w in all_wp]
    all_x = np.concatenate([all_x, wp_xs])
    all_y = np.concatenate([all_y, wp_ys])

    x_margin = max((np.nanmax(all_x) - np.nanmin(all_x)) * 0.15, 1.0)
    y_margin = max((np.nanmax(all_y) - np.nanmin(all_y)) * 0.15, 1.0)
    ax_traj.set_xlim(np.nanmin(all_x) - x_margin, np.nanmax(all_x) + x_margin)
    ax_traj.set_ylim(np.nanmin(all_y) - y_margin, np.nanmax(all_y) + y_margin)

    # Static trajectories (faint)
    ax_traj.plot(ref_r1_x, ref_r1_y, color="blue", lw=0.6, alpha=0.3,
                 label="Robot1 ref (no attack)")
    ax_traj.plot(r1_x, r1_y, color="red", lw=0.6, alpha=0.3,
                 label="Robot1 (meaconed GNSS)" if config["r1_uses_spoofed"] else "Robot1 (clean GNSS)")

    if has_r2:
        ax_traj.plot(r2_x, r2_y, color="orange" if config["r2_uses_spoofed"] else "green",
                     lw=0.6, alpha=0.3,
                     label="Robot2 (meaconed GNSS)" if config["r2_uses_spoofed"] else "Robot2 (clean GNSS)")
    if has_ref_r2:
        ax_traj.plot(ref_r2_x, ref_r2_y, color="cyan", lw=0.6, alpha=0.3,
                     label="Robot2 ref (no attack)")

    # Route waypoints
    for i, (wx, wy) in enumerate(config["wp_r1"]):
        ax_traj.scatter(wx, wy, marker="*", s=140, c="orangered",
                        edgecolors="black", linewidths=0.8, zorder=10)
        ax_traj.annotate(f"R1-WP{i+1}", (wx + 0.15, wy + 0.15),
                         fontsize=7, fontweight="bold", color="darkred")
    for i, (wx, wy) in enumerate(config["wp_r2"]):
        ax_traj.scatter(wx, wy, marker="*", s=140, c="limegreen",
                        edgecolors="black", linewidths=0.8, zorder=10)
        ax_traj.annotate(f"R2-WP{i+1}", (wx + 0.15, wy + 0.15),
                         fontsize=7, fontweight="bold", color="darkgreen")

    # Connection lines for both routes
    for route, color in [(config["wp_r1"], "red"), (config["wp_r2"], "green")]:
        for i in range(len(route)):
            w1 = route[i]
            w2 = route[(i + 1) % len(route)]
            ax_traj.plot([w1[0], w2[0]], [w1[1], w2[1]],
                         "--", lw=0.6, alpha=0.25, color=color)

    # Animated dots
    dot_r1_ref, = ax_traj.plot([], [], "o", color="blue", ms=8, zorder=5)
    dot_r1_atk, = ax_traj.plot([], [], "o", color="red", ms=8, zorder=5)
    dot_r2 = None
    if has_r2 or has_ref_r2:
        dot_r2, = ax_traj.plot([], [], "o",
                               color="orange" if config["r2_uses_spoofed"] else "green",
                               ms=8, zorder=5)

    ax_traj.legend(loc="upper left", fontsize=7.5)

    # ---------------------------------------------------------------------- #
    #  Animation callback                                                    #
    # ---------------------------------------------------------------------- #

    def animate(i):
        """Draw frame i."""
        t_now = t_video[i]

        # ---- CUSUM panel ----
        mask = t_video <= t_now
        cusum_line.set_data(t_video[mask], s_k[mask])
        delta_line.set_data(t_video[mask], d_f[mask])
        cursor_line.set_xdata([t_now, t_now])
        s_now = s_k[i]
        status = "NORMAL"
        if not np.isnan(active_v[i]) and active_v[i] > 0.5:
            if not np.isnan(alert_v[i]) and alert_v[i] > 0.5:
                status = "ALARM"
            else:
                status = "ATTACK"
        time_text.set_text(f"t = {t_now:5.1f}s\nS_k = {s_now:5.2f}\n{status}")

        # ---- Drift panel ----
        drift_line.set_data(t_video[mask], drift[mask])
        drift_cursor.set_xdata([t_now, t_now])
        d_now = drift[i]
        info_lines = [f"drift = {d_now:.3f} m"]
        if ttd is not None:
            info_lines.append(f"TTD = {ttd:.2f} s")
        drift_val_text.set_text("\n".join(info_lines))

        # ---- Trajectory panel ----
        if not np.isnan(r1_x[i]):
            dot_r1_atk.set_data([r1_x[i]], [r1_y[i]])
        if not np.isnan(ref_r1_x[i]):
            dot_r1_ref.set_data([ref_r1_x[i]], [ref_r1_y[i]])
        if dot_r2 is not None:
            # Prefer attack data, fall back to reference
            if has_r2 and not np.isnan(r2_x[i]):
                dot_r2.set_data([r2_x[i]], [r2_y[i]])
            elif has_ref_r2 and not np.isnan(ref_r2_x[i]):
                dot_r2.set_data([ref_r2_x[i]], [ref_r2_y[i]])

        artists = (cusum_line, delta_line, cursor_line, time_text,
                   drift_line, drift_cursor, drift_val_text,
                   dot_r1_atk, dot_r1_ref)
        if dot_r2 is not None:
            artists += (dot_r2,)
        return artists

    # ---------------------------------------------------------------------- #
    #  Render                                                                #
    # ---------------------------------------------------------------------- #

    VIDEOS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = VIDEOS_DIR / f"{exp_type}_experiment.mp4"

    print(f"Rendering {n_frames} frames → {out_path} ...")

    ani = animation.FuncAnimation(
        fig, animate, frames=n_frames,
        interval=1000 / args.fps,
        blit=True,
    )

    writer = animation.FFMpegWriter(
        fps=args.fps,
        codec="libx264",
        bitrate=4000,
        extra_args=["-pix_fmt", "yuv420p"],
    )

    ani.save(str(out_path), writer=writer, dpi=args.dpi)
    plt.close(fig)

    size_mb = out_path.stat().st_size / (1024 * 1024)
    print(f"Done — {out_path}  ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()