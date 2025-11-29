#!/usr/bin/env python3
import os
import sys
import sqlite3
import numpy as np
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Topics we care about
POS_SP_TOPIC = "/cascade_pid/position_setpoint"
POS_CUR_TOPIC = "/cascade_pid/position_current"
VEL_SP_TOPIC = "/cascade_pid/velocity_setpoint"
VEL_CUR_TOPIC = "/cascade_pid/velocity_current"
ATT_SP_TOPIC = "/cascade_pid/attitude_setpoint"
ATT_CUR_TOPIC = "/cascade_pid/attitude_current"
POS_ERR_TOPIC = "/cascade_pid/position_error"
VEL_ERR_TOPIC = "/cascade_pid/velocity_error"
CONTROL_OUT_TOPIC = "/cascade_pid/control_outputs"
VEHICLE_POS_TOPIC = "/fmu/out/vehicle_local_position"
VEHICLE_ATT_TOPIC = "/fmu/out/vehicle_attitude"
RATE_SETPOINT_TOPIC = "/fmu/in/vehicle_rates_setpoint"

def load_topic_id_and_type(cursor, topic_name):
    cursor.execute("SELECT id, name, type FROM topics WHERE name = ?", (topic_name,))
    row = cursor.fetchone()
    if row is None:
        raise RuntimeError(f"Topic '{topic_name}' not found in bag")
    topic_id, name, msg_type = row
    return topic_id, msg_type

def read_topic_messages(cursor, topic_id):
    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,),
    )
    return cursor.fetchall()

def extract_xyz_from_point_stamped(rows, msg_type):
    """rows: list of (timestamp, serialized_data)"""
    MsgType = get_message(msg_type)
    t = []
    x = []
    y = []
    z = []

    for ts, raw in rows:
        msg = deserialize_message(raw, MsgType)
        # rosbag2 timestamps are in nanoseconds
        t.append(ts * 1e-9)
        x.append(msg.point.x)
        y.append(msg.point.y)
        z.append(msg.point.z)

    return np.array(t), np.array(x), np.array(y), np.array(z)

def extract_xyz_from_vector3_stamped(rows, msg_type):
    """Extract XYZ from Vector3Stamped messages (velocity, errors)"""
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

def list_all_topics(cursor):
    """List all topics in the bag with their message counts"""
    cursor.execute("SELECT name, type, message_count FROM topics ORDER BY name")
    topics = cursor.fetchall()
    
    print("\n📡 ALL TOPICS IN BAG:")
    print("-" * 80)
    for name, msg_type, count in topics:
        # Highlight important topics
        marker = "🎯" if any(topic in name for topic in [
            "/cascade_pid/", "/mpc/", "/fmu/out/", "/fmu/in/"
        ]) else "📊"
        print(f"{marker} {name:45} | {msg_type:35} | {count:6} msgs")
    print("-" * 80)
    return topics

def safe_load_topic(cursor, topic_name):
    """Safely load a topic, return None if not found"""
    try:
        return load_topic_id_and_type(cursor, topic_name)
    except RuntimeError:
        return None, None

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
        decompressed_file = compressed_file.replace('.zstd', '')
        
        print(f"📦 Found compressed file: {compressed_file}")
        print(f"🔄 Decompressing to: {decompressed_file}")
        
        try:
            # Decompress using zstd
            subprocess.run(['zstd', '-d', compressed_file], check=True)
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
    
    # Show what files are available
    print(f"🔍 Available files in '{bag_dir}':")
    for f in os.listdir(bag_dir):
        print(f"   {f}")
    
    return None

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 plot_mpc_bag.py <bag_directory>")
        print("Example: python3 plot_mpc_bag.py mpc_flight_figure8_20251116_152221")
        sys.exit(1)

    bag_dir = sys.argv[1]
    if not os.path.isdir(bag_dir):
        print(f"❌ '{bag_dir}' is not a directory")
        sys.exit(1)

    # Find and prepare the .db3 file (handles compression)
    db_path = find_and_prepare_db3_file(bag_dir)
    if db_path is None:
        print(f"❌ No .db3 or .db3.zstd files found in '{bag_dir}'")
        sys.exit(1)
    print(f"📦 Using bag database: {db_path}")

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # --- Get topic IDs and types ---
    pos_sp_id, pos_sp_type = load_topic_id_and_type(cursor, POS_SP_TOPIC)
    pos_cur_id, pos_cur_type = load_topic_id_and_type(cursor, POS_CUR_TOPIC)

    print(f"▶ {POS_SP_TOPIC} -> id={pos_sp_id}, type={pos_sp_type}")
    print(f"▶ {POS_CUR_TOPIC} -> id={pos_cur_id}, type={pos_cur_type}")

    # --- Read messages ---
    sp_rows = read_topic_messages(cursor, pos_sp_id)
    cur_rows = read_topic_messages(cursor, pos_cur_id)

    print(f"   {len(sp_rows)} position setpoint messages")
    print(f"   {len(cur_rows)} position current messages")

    # --- Deserialize and extract xyz ---
    t_sp, x_sp, y_sp, z_sp = extract_xyz_from_point_stamped(sp_rows, pos_sp_type)
    t_cur, x_cur, y_cur, z_cur = extract_xyz_from_point_stamped(cur_rows, pos_cur_type)

    conn.close()

    # --- Plot ---
    # Position tracking
    if 'position' in topics_data:
        t_sp, x_sp, y_sp, z_sp = topics_data['position']['setpoint']
        t_cur, x_cur, y_cur, z_cur = topics_data['position']['current']
        
        if len(t_sp) > 0 and len(t_cur) > 0:
            axes[plot_idx].plot(t_sp, x_sp, 'b-', label="X setpoint", linewidth=2)
            axes[plot_idx].plot(t_cur, x_cur, 'r--', label="X current", linewidth=1.5)
            axes[plot_idx].plot(t_sp, y_sp, 'g-', label="Y setpoint", linewidth=2)
            axes[plot_idx].plot(t_cur, y_cur, 'm--', label="Y current", linewidth=1.5)
            axes[plot_idx].set_title("Position XY Tracking")
            axes[plot_idx].set_xlabel("Time [s]")
            axes[plot_idx].set_ylabel("Position [m]")
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
    
    # Altitude tracking
    if 'position' in topics_data:
        t_sp, x_sp, y_sp, z_sp = topics_data['position']['setpoint']
        t_cur, x_cur, y_cur, z_cur = topics_data['position']['current']
        
        if len(t_sp) > 0 and len(t_cur) > 0 and plot_idx < len(axes):
            axes[plot_idx].plot(t_sp, z_sp, 'b-', label="Z setpoint", linewidth=2)
            axes[plot_idx].plot(t_cur, z_cur, 'r--', label="Z current", linewidth=1.5)
            axes[plot_idx].set_title("Altitude Tracking")
            axes[plot_idx].set_xlabel("Time [s]")
            axes[plot_idx].set_ylabel("Altitude [m]")
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
    
    # Position errors
    if 'pos_error' in topics_data and plot_idx < len(axes):
        t_err, x_err, y_err, z_err = topics_data['pos_error']
        if len(t_err) > 0:
            pos_error_norm = np.sqrt(x_err**2 + y_err**2 + z_err**2)
            axes[plot_idx].plot(t_err, pos_error_norm, 'r-', linewidth=2)
            axes[plot_idx].set_title("Position Error Magnitude")
            axes[plot_idx].set_xlabel("Time [s]")
            axes[plot_idx].set_ylabel("Error [m]")
            axes[plot_idx].grid(True)
            plot_idx += 1
    
    # Velocity tracking
    if 'velocity' in topics_data and plot_idx < len(axes):
        t_vsp, vx_sp, vy_sp, vz_sp = topics_data['velocity']['setpoint']
        t_vcur, vx_cur, vy_cur, vz_cur = topics_data['velocity']['current']
        
        if len(t_vsp) > 0 and len(t_vcur) > 0:
            vel_sp_norm = np.sqrt(vx_sp**2 + vy_sp**2)
            vel_cur_norm = np.sqrt(vx_cur**2 + vy_cur**2)
            axes[plot_idx].plot(t_vsp, vel_sp_norm, 'b-', label="Vel SP", linewidth=2)
            axes[plot_idx].plot(t_vcur, vel_cur_norm, 'r--', label="Vel Current", linewidth=1.5)
            axes[plot_idx].set_title("Velocity Magnitude")
            axes[plot_idx].set_xlabel("Time [s]")
            axes[plot_idx].set_ylabel("Velocity [m/s]")
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
    
    # XY trajectory
    if 'position' in topics_data and plot_idx < len(axes):
        t_sp, x_sp, y_sp, z_sp = topics_data['position']['setpoint']
        t_cur, x_cur, y_cur, z_cur = topics_data['position']['current']
        
        if len(x_sp) > 0 and len(x_cur) > 0:
            axes[plot_idx].plot(x_sp, y_sp, 'b-', label="Setpoint path", linewidth=2)
            axes[plot_idx].plot(x_cur, y_cur, 'r--', label="Actual path", linewidth=1.5)
            axes[plot_idx].set_title("XY Trajectory")
            axes[plot_idx].set_xlabel("X [m]")
            axes[plot_idx].set_ylabel("Y [m]")
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            axes[plot_idx].axis('equal')
            plot_idx += 1
    
    # Control outputs (if available)
    if 'control' in topics_data and plot_idx < len(axes):
        t_ctrl, ctrl_data = topics_data['control']
        if len(t_ctrl) > 0 and len(ctrl_data) > 0:
            ctrl_array = np.array(ctrl_data)
            if ctrl_array.shape[1] >= 3:  # At least 3 control outputs
                axes[plot_idx].plot(t_ctrl, ctrl_array[:, 0], 'r-', label="Roll rate", linewidth=1.5)
                axes[plot_idx].plot(t_ctrl, ctrl_array[:, 1], 'g-', label="Pitch rate", linewidth=1.5)
                axes[plot_idx].plot(t_ctrl, ctrl_array[:, 2], 'b-', label="Yaw rate", linewidth=1.5)
                if ctrl_array.shape[1] >= 4:
                    axes[plot_idx].plot(t_ctrl, ctrl_array[:, 3], 'k-', label="Thrust", linewidth=1.5)
                axes[plot_idx].set_title("Control Outputs")
                axes[plot_idx].set_xlabel("Time [s]")
                axes[plot_idx].set_ylabel("Commands")
                axes[plot_idx].legend()
                axes[plot_idx].grid(True)
                plot_idx += 1
    
    # Hide unused subplots
    for i in range(plot_idx, len(axes)):
        axes[i].set_visible(False)
    
    plt.tight_layout()
    
    # Save the plot
    plot_filename = f"{bag_dir}_comprehensive_analysis.png"
    plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
    print(f"📊 Comprehensive analysis saved: {plot_filename}")
    
    plt.show()
    print("✅ Comprehensive plotting completed successfully!")

if __name__ == "__main__":
    main()
