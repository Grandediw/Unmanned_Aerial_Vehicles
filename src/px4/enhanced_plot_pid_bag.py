#!/usr/bin/env python3
"""
Enhanced PID Bag Plotter
Plots comprehensive PID cascade and control data from rosbag
Shows all available topics and creates detailed analysis plots
"""

import os
import sys
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def list_all_topics(cursor):
    """List all topics in the bag with their message counts"""
    cursor.execute("PRAGMA table_info(topics)")
    columns = cursor.fetchall()
    column_names = [col[1] for col in columns]

    if "message_count" in column_names:
        cursor.execute("SELECT name, type, message_count FROM topics ORDER BY name")
        topics = cursor.fetchall()
        has_count = True
    else:
        cursor.execute("SELECT name, type FROM topics ORDER BY name")
        topics = cursor.fetchall()
        has_count = False

    print("\n📡 ALL TOPICS IN BAG:")
    print("-" * 80)

    if has_count:
        for name, msg_type, count in topics:
            marker = (
                "🎯"
                if any(
                    topic in name
                    for topic in ["/cascade_pid/", "/mpc/", "/fmu/out/", "/fmu/in/"]
                )
                else "📊"
            )
            print(f"{marker} {name:45} | {msg_type:35} | {count:6} msgs")
    else:
        for name, msg_type in topics:
            cursor.execute(
                """
                SELECT COUNT(*) FROM messages m
                JOIN topics t ON m.topic_id = t.id
                WHERE t.name = ?
                """,
                (name,),
            )
            count = cursor.fetchone()[0]
            marker = (
                "🎯"
                if any(
                    topic in name
                    for topic in ["/cascade_pid/", "/mpc/", "/fmu/out/", "/fmu/in/"]
                )
                else "📊"
            )
            print(f"{marker} {name:45} | {msg_type:35} | {count:6} msgs")

    print("-" * 80)
    return topics


def safe_load_topic(cursor, topic_name):
    """Safely load a topic, return None if not found"""
    cursor.execute("SELECT id, name, type FROM topics WHERE name = ?", (topic_name,))
    row = cursor.fetchone()
    if row is None:
        return None, None
    topic_id, name, msg_type = row
    return topic_id, msg_type


def read_topic_messages(cursor, topic_id):
    """Read all messages for a topic"""
    if topic_id is None:
        return []
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,),
    )
    return cursor.fetchall()


def extract_xyz_from_point_stamped(rows, msg_type):
    """Extract XYZ from PointStamped messages"""
    if not rows:
        return np.array([]), np.array([]), np.array([]), np.array([])

    MsgType = get_message(msg_type)
    t, x, y, z = [], [], [], []

    for ts, raw in rows:
        try:
            msg = deserialize_message(raw, MsgType)
            t.append(ts * 1e-9)  # ns -> s
            x.append(msg.point.x)
            y.append(msg.point.y)
            z.append(msg.point.z)
        except Exception as e:
            print(f"⚠️ Failed to deserialize PointStamped: {e}")
            continue

    return np.array(t), np.array(x), np.array(y), np.array(z)


def extract_xyz_from_vector3_stamped(rows, msg_type):
    """Extract XYZ from Vector3Stamped messages (velocity, errors)"""
    if not rows:
        return np.array([]), np.array([]), np.array([]), np.array([])

    MsgType = get_message(msg_type)
    t, x, y, z = [], [], [], []

    for ts, raw in rows:
        try:
            msg = deserialize_message(raw, MsgType)
            t.append(ts * 1e-9)
            x.append(msg.vector.x)
            y.append(msg.vector.y)
            z.append(msg.vector.z)
        except Exception as e:
            print(f"⚠️ Failed to deserialize Vector3Stamped: {e}")
            continue

    return np.array(t), np.array(x), np.array(y), np.array(z)


def extract_float64_multiarray(rows, msg_type):
    """Extract data from Float64MultiArray messages (control outputs)"""
    if not rows:
        return np.array([]), np.array([])

    MsgType = get_message(msg_type)
    t, data_arrays = [], []

    for ts, raw in rows:
        try:
            msg = deserialize_message(raw, MsgType)
            t.append(ts * 1e-9)
            data_arrays.append(msg.data)
        except Exception as e:
            print(f"⚠️ Failed to deserialize Float64MultiArray: {e}")
            continue

    return np.array(t), np.array(data_arrays)


def find_and_prepare_db3_file(bag_dir):
    """Find .db3 file, handling compression if needed"""
    import subprocess

    # Look for .db3 files first
    db_files = [f for f in os.listdir(bag_dir) if f.endswith(".db3")]
    if db_files:
        return os.path.join(bag_dir, db_files[0])

    # Look for compressed .db3.zstd files
    zstd_files = [f for f in os.listdir(bag_dir) if f.endswith(".db3.zstd")]
    if zstd_files:
        compressed_file = os.path.join(bag_dir, zstd_files[0])
        decompressed_file = compressed_file.replace(".zstd", "")

        print(f"📦 Found compressed file: {compressed_file}")
        print(f"🔄 Decompressing to: {decompressed_file}")

        try:
            subprocess.run(["zstd", "-d", compressed_file], check=True)
            if os.path.exists(decompressed_file):
                print("✅ Decompression successful!")
                return decompressed_file
            else:
                print("❌ Decompression failed - file not created")
        except subprocess.CalledProcessError as e:
            print(f"❌ Decompression failed: {e}")
            print("💡 Try installing zstd: sudo apt install zstd")
        except FileNotFoundError:
            print("❌ zstd command not found")
            print("💡 Install with: sudo apt install zstd")
            print(f"💡 Or decompress manually: zstd -d {compressed_file}")

    print(f"🔍 Available files in '{bag_dir}':")
    for f in os.listdir(bag_dir):
        print(f"   {f}")

    return None


def create_analysis_plots(data, bag_dir):
    """Create comprehensive analysis plots from loaded data"""

    n_datasets = len(data)
    print(f"\n📊 Creating plots for {n_datasets} datasets...")

    if n_datasets <= 4:
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        axes = axes.flatten()
    elif n_datasets <= 6:
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        axes = axes.flatten()
    else:
        fig, axes = plt.subplots(3, 3, figsize=(20, 12))
        axes = axes.flatten()

    fig.suptitle(f"PID Cascade Flight Analysis - {os.path.basename(bag_dir)}", fontsize=16)

    plot_idx = 0

    # Position tracking XY
    if "pos_setpoint" in data and "pos_current" in data and plot_idx < len(axes):
        t_sp, x_sp, y_sp, z_sp = data["pos_setpoint"]
        t_cur, x_cur, y_cur, z_cur = data["pos_current"]

        if len(t_sp) > 0 and len(t_cur) > 0:
            axes[plot_idx].plot(t_sp, x_sp, "b-", label="X setpoint", linewidth=2)
            axes[plot_idx].plot(t_cur, x_cur, "r--", label="X actual", linewidth=1.5)
            axes[plot_idx].plot(t_sp, y_sp, "g-", label="Y setpoint", linewidth=2)
            axes[plot_idx].plot(t_cur, y_cur, "m--", label="Y actual", linewidth=1.5)
            axes[plot_idx].set_title("Position XY Tracking")
            axes[plot_idx].set_xlabel("Time [s]")
            axes[plot_idx].set_ylabel("Position [m]")
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1

    # Altitude tracking
    if "pos_setpoint" in data and "pos_current" in data and plot_idx < len(axes):
        t_sp, x_sp, y_sp, z_sp = data["pos_setpoint"]
        t_cur, x_cur, y_cur, z_cur = data["pos_current"]

        if len(t_sp) > 0 and len(t_cur) > 0:
            axes[plot_idx].plot(t_sp, z_sp, "b-", label="Z setpoint", linewidth=2)
            axes[plot_idx].plot(t_cur, z_cur, "r--", label="Z actual", linewidth=1.5)
            axes[plot_idx].set_title("Altitude Tracking")
            axes[plot_idx].set_xlabel("Time [s]")
            axes[plot_idx].set_ylabel("Altitude [m]")
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1

    # Position errors
    if "pos_error" in data and plot_idx < len(axes):
        t_err, x_err, y_err, z_err = data["pos_error"]
        if len(t_err) > 0:
            pos_error_norm = np.sqrt(x_err**2 + y_err**2 + z_err**2)
            axes[plot_idx].plot(t_err, pos_error_norm, "r-", linewidth=2, label="||e_pos||")
            axes[plot_idx].plot(t_err, np.abs(x_err), "b--", alpha=0.7, label="|X error|")
            axes[plot_idx].plot(t_err, np.abs(y_err), "g--", alpha=0.7, label="|Y error|")
            axes[plot_idx].plot(t_err, np.abs(z_err), "m--", alpha=0.7, label="|Z error|")
            axes[plot_idx].set_title("Position Errors")
            axes[plot_idx].set_xlabel("Time [s]")
            axes[plot_idx].set_ylabel("Error [m]")
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1

    # Velocity tracking
    if "vel_setpoint" in data and "vel_current" in data and plot_idx < len(axes):
        t_vsp, vx_sp, vy_sp, vz_sp = data["vel_setpoint"]
        t_vcur, vx_cur, vy_cur, vz_cur = data["vel_current"]

        if len(t_vsp) > 0 and len(t_vcur) > 0:
            vel_sp_norm = np.sqrt(vx_sp**2 + vy_sp**2)
            vel_cur_norm = np.sqrt(vx_cur**2 + vy_cur**2)
            axes[plot_idx].plot(t_vsp, vel_sp_norm, "b-", label="Speed setpoint", linewidth=2)
            axes[plot_idx].plot(t_vcur, vel_cur_norm, "r--", label="Speed actual", linewidth=1.5)
            axes[plot_idx].plot(t_vsp, vz_sp, "g:", label="Vz setpoint", linewidth=2)
            axes[plot_idx].plot(t_vcur, vz_cur, "k:", label="Vz actual", linewidth=1.5)
            axes[plot_idx].set_title("Velocity Tracking")
            axes[plot_idx].set_xlabel("Time [s]")
            axes[plot_idx].set_ylabel("Velocity [m/s]")
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1

    # XY trajectory
    if "pos_setpoint" in data and "pos_current" in data and plot_idx < len(axes):
        t_sp, x_sp, y_sp, z_sp = data["pos_setpoint"]
        t_cur, x_cur, y_cur, z_cur = data["pos_current"]

        if len(x_sp) > 0 and len(x_cur) > 0:
            axes[plot_idx].plot(x_sp, y_sp, "b-", label="Setpoint trajectory", linewidth=2)
            axes[plot_idx].plot(x_cur, y_cur, "r--", label="Actual trajectory", linewidth=1.5)
            axes[plot_idx].set_title("XY Trajectory")
