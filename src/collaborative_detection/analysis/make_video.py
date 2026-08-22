#!/usr/bin/env python3
"""
Make synchronized experiment video from rosbag data.

Generates an MP4 with three synchronized panels:
  1. CUSUM S_k + delta (innovation) with a moving time cursor
  2. Physical drift ‖p_real(t) − p_ref(t)‖ with attack/alert markers
  3. Top-down robot trajectories (robot1 ref + attack, robot2 attack)

Usage:
    cd ~/tfm_meaconing_ws
    /path/to/jazzy/python3 \
        src/collaborative_detection/analysis/make_video.py e5_waypoint_attack

    # With custom speedup and duration
    /path/to/jazzy/python3 \
        src/collaborative_detection/analysis/make_video.py e5_waypoint_attack --speedup 4 --max-time 60

Output:
    ~/tfm_meaconing_ws/results/videos/e5_experiment.mp4
"""

import argparse
import os
import sys
from pathlib import Path
from collections import defaultdict

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

# --------------------------------------------------------------------------- #
#  Data loading                                                               #
# --------------------------------------------------------------------------- #


def _load_odom_trajectory(bag_path, topic):
    """Load (x, y, time) from an Odometry topic.  Returns dict or None."""
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
    t0 = None

    while reader.has_next():
        t, msg_bytes, ts_ns = reader.read_next()
        if t != topic:
            continue
        ts = ts_ns / 1e9
        if t0 is None:
            t0 = ts
        msg = deserialize_message(msg_bytes, get_message(type_map[topic]))
        pos = msg.pose.pose.position
        times.append(ts - t0)
        xs.append(pos.x)
        ys.append(pos.y)

    if not times:
        return None
    return {"time": np.array(times), "x": np.array(xs), "y": np.array(ys)}


def _load_scalar(bag_path, topic):
    """Load a scalar topic (Float64 or Bool) → {time, value}.  Returns None if empty."""
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
    t0 = None

    while reader.has_next():
        t, msg_bytes, ts_ns = reader.read_next()
        if t != topic:
            continue
        ts = ts_ns / 1e9
        if t0 is None:
            t0 = ts
        msg = deserialize_message(msg_bytes, get_message(type_map[t]))
        v = msg.data
        v = float(1.0 if isinstance(v, bool) and v else v)
        times.append(ts - t0)
        vals.append(v)

    if not times:
        return None
    return {"time": np.array(times), "value": np.array(vals)}


# --------------------------------------------------------------------------- #
#  Main                                                                       #
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(description="Generate E5 experiment video")
    parser.add_argument("experiment", help="Experiment folder name, e.g. e5_waypoint_attack")
    parser.add_argument("--speedup", type=float, default=2.0,
                        help="Playback speed multiplier (default: 2)")
    parser.add_argument("--max-time", type=float, default=None,
                        help="Stop video after this many seconds of sim time")
    parser.add_argument("--fps", type=int, default=30,
                        help="Output video frames per second (default: 30)")
    parser.add_argument("--dpi", type=int, default=150,
                        help="Output resolution DPI (default: 150)")
    args = parser.parse_args()

    # --- Locate rosbags --------------------------------------------------- #
    atk_dir = RESULTS_DIR / args.experiment
    ref_dir = RESULTS_DIR / "e5_ref_waypoint_reference"

    if not atk_dir.is_dir():
        print(f"Attack bag not found: {atk_dir}", file=sys.stderr)
        print("Available experiments:", file=sys.stderr)
        for d in sorted(RESULTS_DIR.glob("*")):
            if d.is_dir():
                print(f"  {d.name}", file=sys.stderr)
        sys.exit(1)
    if not ref_dir.is_dir():
        print(f"Reference bag not found: {ref_dir}", file=sys.stderr)
        print("Run e5_ref first.", file=sys.stderr)
        sys.exit(1)

    print(f"Attack bag:    {atk_dir}")
    print(f"Reference bag: {ref_dir}")

    # --- Load attack data ------------------------------------------------- #
    print("Loading attack data ...")
    cusum   = _load_scalar(atk_dir, "/system/cusum_value")
    delta_f = _load_scalar(atk_dir, "/system/delta_value")
    alert   = _load_scalar(atk_dir, "/system/meaconing_alert")
    active  = _load_scalar(atk_dir, "/meaconing/active")
    odom_r1 = _load_odom_trajectory(atk_dir, "/robot1/odom")
    odom_r2 = _load_odom_trajectory(atk_dir, "/robot2/odom")

    # --- Load reference data ---------------------------------------------- #
    print("Loading reference data ...")
    ref_odom_r1 = _load_odom_trajectory(ref_dir, "/robot1/odom")

    # --- Sanity checks ---------------------------------------------------- #
    if cusum is None or delta_f is None:
        print("CUSUM/delta data missing — check the rosbag", file=sys.stderr)
        sys.exit(1)
    if odom_r1 is None or ref_odom_r1 is None:
        print("Odometry data missing — check the rosbag", file=sys.stderr)
        sys.exit(1)

    # --- Time window ------------------------------------------------------ #
    t_end = max(cusum["time"][-1], odom_r1["time"][-1])
    if args.max_time is not None:
        t_end = min(t_end, args.max_time)

    print(f"Simulation time: 0 → {t_end:.1f} s")
    print(f"Video duration:  {t_end / args.speedup:.1f} s  (speedup ×{args.speedup})")
    print(f"Output FPS:       {args.fps}")

    # --- Resample everything to a uniform time grid ----------------------- #
    n_frames = int(t_end * args.fps / args.speedup)
    # Each output frame advances sim-time by  speedup / fps  seconds
    dt_frame = args.speedup / args.fps
    t_video = np.arange(n_frames) * dt_frame   # sim-time of each video frame

    def _interp(series, t_target):
        """Linearly interpolate a {time, value} series onto t_target."""
        if series is None or len(series["time"]) == 0:
            return np.full_like(t_target, np.nan)
        return np.interp(t_target, series["time"], series["value"],
                         left=np.nan, right=np.nan)

    s_k     = _interp(cusum, t_video)
    d_f     = _interp(delta_f, t_video)
    alert_v = _interp(alert, t_video)
    active_v = _interp(active, t_video)

    # Odometry interpolation
    r1_x = np.interp(t_video, odom_r1["time"], odom_r1["x"], left=np.nan, right=np.nan)
    r1_y = np.interp(t_video, odom_r1["time"], odom_r1["y"], left=np.nan, right=np.nan)

    # Robot2 — add spawn offset (0, 2) to convert odom → world
    if odom_r2 is not None:
        r2_x = np.interp(t_video, odom_r2["time"], odom_r2["x"], left=np.nan, right=np.nan) + 0.0
        r2_y = np.interp(t_video, odom_r2["time"], odom_r2["y"], left=np.nan, right=np.nan) + 2.0
        has_r2 = True
    else:
        r2_x = np.full_like(t_video, np.nan)
        r2_y = np.full_like(t_video, np.nan)
        has_r2 = False

    # Reference robot1 trajectory — interpolated onto same time grid
    ref_x = np.interp(t_video, ref_odom_r1["time"], ref_odom_r1["x"],
                      left=np.nan, right=np.nan)
    ref_y = np.interp(t_video, ref_odom_r1["time"], ref_odom_r1["y"],
                      left=np.nan, right=np.nan)

    # Physical drift
    drift = np.sqrt((r1_x - ref_x) ** 2 + (r1_y - ref_y) ** 2)

    # Attack / alert times
    t_attack = None
    if active is not None:
        idx = np.where(active["value"] > 0.5)[0]
        if len(idx) > 0:
            t_attack = float(active["time"][idx[0]])

    t_alert = None
    if alert is not None:
        idx = np.where(alert["value"] > 0.5)[0]
        if len(idx) > 0:
            t_alert = float(alert["time"][idx[0]])

    print(f"Attack time:  {t_attack:.1f}s" if t_attack else "Attack time:  never")
    print(f"Alert time:   {t_alert:.1f}s" if t_alert else "Alert time:   never")

    # ---------------------------------------------------------------------- #
    #  Figure setup                                                          #
    # ---------------------------------------------------------------------- #
    # 2-column layout:
    #   left  (wide):  CUSUM + delta
    #   right (top):   Physical drift
    #   right (bottom): Trajectories
    fig = plt.figure(figsize=(20, 10), dpi=args.dpi)
    gs = GridSpec(2, 2, figure=fig,
                  width_ratios=[1.2, 1.0],
                  height_ratios=[1, 1],
                  hspace=0.35, wspace=0.30)

    ax_cusum = fig.add_subplot(gs[:, 0])      # left column, full height
    ax_drift = fig.add_subplot(gs[0, 1])       # right top
    ax_traj  = fig.add_subplot(gs[1, 1])       # right bottom

    # --- Left panel: CUSUM + delta --------------------------------------- #
    ax_cusum.set_title("CUSUM detector", fontsize=13, fontweight="bold")
    ax_cusum.set_xlabel("Time (s)")
    ax_cusum.set_ylabel("S_k / δ (m)", color="black")
    ax_cusum.set_xlim(0, t_end)
    ax_cusum.set_ylim(-1, max(TAU * 2, np.nanmax(s_k) * 1.1 + 0.5))
    ax_cusum.axhline(TAU, color="red", ls="--", lw=1.5, label=f"τ = {TAU}")
    ax_cusum.grid(True, alpha=0.3)

    # Static: full CUSUM trace (faint)
    ax_cusum.plot(t_video, s_k, color="steelblue", lw=0.6, alpha=0.4)
    ax_cusum.plot(t_video, d_f, color="green", lw=0.4, alpha=0.3)

    # Attack / alert vertical lines
    if t_attack is not None:
        ax_cusum.axvline(t_attack, color="purple", ls=":", lw=1.5, alpha=0.6)
    if t_alert is not None:
        ax_cusum.axvline(t_alert, color="red", ls="--", lw=1.5, alpha=0.6)

    # Animated elements (drawn fresh each frame)
    cusum_line, = ax_cusum.plot([], [], "b-", lw=1.8, label="S_k (CUSUM)")
    delta_line, = ax_cusum.plot([], [], "g-", lw=0.8, alpha=0.7, label="δ (innovation)")
    cursor_line = ax_cusum.axvline(0, color="orange", lw=2.0, alpha=0.8)
    time_text = ax_cusum.text(0.02, 0.96, "", transform=ax_cusum.transAxes,
                              fontsize=10, va="top", fontfamily="monospace",
                              bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))
    ax_cusum.legend(loc="upper right", fontsize=8)

    # --- Right-top: Physical drift --------------------------------------- #
    ax_drift.set_title("Physical drift ‖p(t) − p_ref(t)‖", fontsize=13, fontweight="bold")
    ax_drift.set_xlabel("Time (s)")
    ax_drift.set_ylabel("Drift (m)")
    ax_drift.set_xlim(0, t_end)
    drift_max = max(np.nanmax(drift) * 1.1, 0.5)
    ax_drift.set_ylim(-0.05 * drift_max, drift_max)
    ax_drift.grid(True, alpha=0.3)

    # Static: full drift trace (faint)
    ax_drift.plot(t_video, drift, color="darkblue", lw=0.6, alpha=0.3)
    if t_attack is not None:
        ax_drift.axvline(t_attack, color="purple", ls=":", lw=1.5, alpha=0.6)
    if t_alert is not None:
        ax_drift.axvline(t_alert, color="red", ls="--", lw=1.5, alpha=0.6)

    drift_line, = ax_drift.plot([], [], "b-", lw=1.5)
    drift_cursor = ax_drift.axvline(0, color="orange", lw=2.0, alpha=0.8)
    drift_val_text = ax_drift.text(0.98, 0.92, "", transform=ax_drift.transAxes,
                                   fontsize=11, ha="right", va="top",
                                   fontfamily="monospace", fontweight="bold",
                                   bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    # --- Right-bottom: Trajectories (top-down) --------------------------- #
    ax_traj.set_title("Robot trajectories (top-down)", fontsize=13, fontweight="bold")
    ax_traj.set_xlabel("World X (m)")
    ax_traj.set_ylabel("World Y (m)")
    ax_traj.set_aspect("equal")
    ax_traj.grid(True, alpha=0.3)

    # Determine plot bounds from all trajectories
    all_x = np.concatenate([r1_x[~np.isnan(r1_x)], ref_x[~np.isnan(ref_x)]])
    all_y = np.concatenate([r1_y[~np.isnan(r1_y)], ref_y[~np.isnan(ref_y)]])
    if has_r2:
        all_x = np.concatenate([all_x, r2_x[~np.isnan(r2_x)]])
        all_y = np.concatenate([all_y, r2_y[~np.isnan(r2_y)]])

    x_margin = max((np.nanmax(all_x) - np.nanmin(all_x)) * 0.15, 1.0)
    y_margin = max((np.nanmax(all_y) - np.nanmin(all_y)) * 0.15, 1.0)
    ax_traj.set_xlim(np.nanmin(all_x) - x_margin, np.nanmax(all_x) + x_margin)
    ax_traj.set_ylim(np.nanmin(all_y) - y_margin, np.nanmax(all_y) + y_margin)

    # Static full trajectories (faint)
    ax_traj.plot(ref_x, ref_y, color="blue", lw=0.6, alpha=0.3, label="Robot1 (ref)")
    ax_traj.plot(r1_x, r1_y, color="red", lw=0.6, alpha=0.3, label="Robot1 (attack)")
    if has_r2:
        ax_traj.plot(r2_x, r2_y, color="green", lw=0.6, alpha=0.3, label="Robot2")

    # Waypoint marker
    way_x, way_y = 5.0, 0.0   # from params.yaml
    ax_traj.scatter(way_x, way_y, marker="*", s=120, c="gold", edgecolors="black",
                    linewidths=0.8, zorder=10, label=f"Waypoint ({way_x},{way_y})")

    # Animated dots
    dot_r1_ref, = ax_traj.plot([], [], "o", color="blue", ms=8, zorder=5)
    dot_r1_atk, = ax_traj.plot([], [], "o", color="red", ms=8, zorder=5)
    dot_r2,     = ax_traj.plot([], [], "o", color="green", ms=8, zorder=5)

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
        drift_val_text.set_text(f"drift = {d_now:.3f} m")

        # ---- Trajectory panel ----
        if not np.isnan(r1_x[i]):
            dot_r1_atk.set_data([r1_x[i]], [r1_y[i]])
        if not np.isnan(ref_x[i]):
            dot_r1_ref.set_data([ref_x[i]], [ref_y[i]])
        if has_r2 and not np.isnan(r2_x[i]):
            dot_r2.set_data([r2_x[i]], [r2_y[i]])

        return (cusum_line, delta_line, cursor_line, time_text,
                drift_line, drift_cursor, drift_val_text,
                dot_r1_atk, dot_r1_ref, dot_r2)

    # ---------------------------------------------------------------------- #
    #  Render                                                                #
    # ---------------------------------------------------------------------- #

    VIDEOS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = VIDEOS_DIR / "e5_experiment.mp4"

    print(f"Rendering {n_frames} frames → {out_path} ...")

    ani = animation.FuncAnimation(
        fig, animate, frames=n_frames,
        interval=1000 / args.fps,  # ms per frame (cosmetic for the writer)
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