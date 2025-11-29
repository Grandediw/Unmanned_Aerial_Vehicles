# #!/usr/bin/env python3
# """
# Enhanced MPC Bag Plotter
# Plots comprehensive MPC and control data from rosbag
# Shows all available topics and creates detailed analysis plots
# """

# import os
# import sys
# import sqlite3
# import numpy as np
# import matplotlib.pyplot as plt
# from rclpy.serialization import deserialize_message
# from rosidl_runtime_py.utilities import get_message

# def list_all_topics(cursor):
#     """List all topics in the bag with their message counts"""
#     # First, let's see what columns are available
#     cursor.execute("PRAGMA table_info(topics)")
#     columns = cursor.fetchall()
#     column_names = [col[1] for col in columns]
    
#     # Use available columns (some rosbag versions don't have message_count)
#     if 'message_count' in column_names:
#         cursor.execute("SELECT name, type, message_count FROM topics ORDER BY name")
#         topics = cursor.fetchall()
#         has_count = True
#     else:
#         cursor.execute("SELECT name, type FROM topics ORDER BY name")
#         topics = cursor.fetchall()
#         has_count = False
    
#     print("\n📡 ALL TOPICS IN BAG:")
#     print("-" * 80)
    
#     if has_count:
#         for name, msg_type, count in topics:
#             marker = "🎯" if any(topic in name for topic in [
#                 "/cascade_pid/", "/mpc/", "/fmu/out/", "/fmu/in/"
#             ]) else "📊"
#             print(f"{marker} {name:45} | {msg_type:35} | {count:6} msgs")
#     else:
#         for name, msg_type in topics:
#             # Count messages manually
#             cursor.execute("SELECT COUNT(*) FROM messages m JOIN topics t ON m.topic_id = t.id WHERE t.name = ?", (name,))
#             count = cursor.fetchone()[0]
#             marker = "🎯" if any(topic in name for topic in [
#                 "/cascade_pid/", "/mpc/", "/fmu/out/", "/fmu/in/"
#             ]) else "📊"
#             print(f"{marker} {name:45} | {msg_type:35} | {count:6} msgs")
    
#     print("-" * 80)
#     return topics

# def safe_load_topic(cursor, topic_name):
#     """Safely load a topic, return None if not found"""
#     cursor.execute("SELECT id, name, type FROM topics WHERE name = ?", (topic_name,))
#     row = cursor.fetchone()
#     if row is None:
#         return None, None
#     topic_id, name, msg_type = row
#     return topic_id, msg_type

# def read_topic_messages(cursor, topic_id):
#     """Read all messages for a topic"""
#     if topic_id is None:
#         return []
#     cursor.execute(
#         "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
#         (topic_id,),
#     )
#     return cursor.fetchall()

# def extract_xyz_from_point_stamped(rows, msg_type):
#     """Extract XYZ from PointStamped messages"""
#     if not rows:
#         return np.array([]), np.array([]), np.array([]), np.array([])
        
#     MsgType = get_message(msg_type)
#     t, x, y, z = [], [], [], []

#     for ts, raw in rows:
#         try:
#             msg = deserialize_message(raw, MsgType)
#             t.append(ts * 1e-9)  # Convert ns to seconds
#             x.append(msg.point.x)
#             y.append(msg.point.y)
#             z.append(msg.point.z)
#         except Exception as e:
#             print(f"⚠️ Failed to deserialize PointStamped: {e}")
#             continue

#     return np.array(t), np.array(x), np.array(y), np.array(z)

# def extract_xyz_from_vector3_stamped(rows, msg_type):
#     """Extract XYZ from Vector3Stamped messages (velocity, errors)"""
#     if not rows:
#         return np.array([]), np.array([]), np.array([]), np.array([])
        
#     MsgType = get_message(msg_type)
#     t, x, y, z = [], [], [], []

#     for ts, raw in rows:
#         try:
#             msg = deserialize_message(raw, MsgType)
#             t.append(ts * 1e-9)
#             x.append(msg.vector.x)
#             y.append(msg.vector.y)
#             z.append(msg.vector.z)
#         except Exception as e:
#             print(f"⚠️ Failed to deserialize Vector3Stamped: {e}")
#             continue

#     return np.array(t), np.array(x), np.array(y), np.array(z)

# def extract_float64_multiarray(rows, msg_type):
#     """Extract data from Float64MultiArray messages (control outputs)"""
#     if not rows:
#         return np.array([]), np.array([])
        
#     MsgType = get_message(msg_type)
#     t, data_arrays = [], []

#     for ts, raw in rows:
#         try:
#             msg = deserialize_message(raw, MsgType)
#             t.append(ts * 1e-9)
#             data_arrays.append(msg.data)
#         except Exception as e:
#             print(f"⚠️ Failed to deserialize Float64MultiArray: {e}")
#             continue

#     return np.array(t), np.array(data_arrays)
# def normalize_time_axes(data):
#     """Shift all time arrays so that the earliest timestamp becomes 0."""
#     t_min = None
    
#     # Find global minimum timestamp across all datasets
#     for key, entry in data.items():
#         t = entry[0]
#         if t.size == 0:
#             continue
#         cur_min = float(np.min(t))
#         if t_min is None or cur_min < t_min:
#             t_min = cur_min
    
#     if t_min is None:
#         return  # nothing to do
    
#     # Shift all timestamps
#     for key, entry in data.items():
#         t = entry[0] - t_min
        
#         # Re-pack tuple depending on how many items it has
#         if len(entry) == 4:
#             # (t, x, y, z)
#             data[key] = (t, entry[1], entry[2], entry[3])
#         elif len(entry) == 2:
#             # (t, data_array)
#             data[key] = (t, entry[1])


# def find_and_prepare_db3_file(bag_dir):
#     """Find .db3 file, handling compression if needed"""
#     import subprocess
    
#     # Look for .db3 files first
#     db_files = [f for f in os.listdir(bag_dir) if f.endswith(".db3")]
#     if db_files:
#         return os.path.join(bag_dir, db_files[0])
    
#     # Look for compressed .db3.zstd files
#     zstd_files = [f for f in os.listdir(bag_dir) if f.endswith(".db3.zstd")]
#     if zstd_files:
#         compressed_file = os.path.join(bag_dir, zstd_files[0])
#         decompressed_file = compressed_file.replace('.zstd', '')
        
#         print(f"📦 Found compressed file: {compressed_file}")
#         print(f"🔄 Decompressing to: {decompressed_file}")
        
#         try:
#             # Decompress using zstd
#             subprocess.run(['zstd', '-d', compressed_file], check=True)
#             if os.path.exists(decompressed_file):
#                 print("✅ Decompression successful!")
#                 return decompressed_file
#             else:
#                 print("❌ Decompression failed - file not created")
#         except subprocess.CalledProcessError as e:
#             print(f"❌ Decompression failed: {e}")
#             print("💡 Try installing zstd: sudo apt install zstd")
#         except FileNotFoundError:
#             print("❌ zstd command not found")
#             print("💡 Install with: sudo apt install zstd")
#             print(f"💡 Or decompress manually: zstd -d {compressed_file}")
    
#     # Show what files are available
#     print(f"🔍 Available files in '{bag_dir}':")
#     for f in os.listdir(bag_dir):
#         print(f"   {f}")
    
#     return None

# def main():
#     if len(sys.argv) != 2:
#         print("Usage: python3 enhanced_plot_mpc_bag.py <bag_directory>")
#         print("Example: python3 enhanced_plot_mpc_bag.py mpc_flight_figure8_20251116_152221")
#         sys.exit(1)

#     bag_dir = sys.argv[1]
#     if not os.path.isdir(bag_dir):
#         print(f"❌ '{bag_dir}' is not a directory")
#         sys.exit(1)

#     # Find and prepare the .db3 file (handles compression)
#     db_path = find_and_prepare_db3_file(bag_dir)
#     if db_path is None:
#         print(f"❌ No .db3 or .db3.zstd files found in '{bag_dir}'")
#         sys.exit(1)

#     print(f"📦 Using bag database: {db_path}")

#     try:
#         conn = sqlite3.connect(db_path)
#         cursor = conn.cursor()

#         # List all available topics first
#         all_topics = list_all_topics(cursor)
        
#         print("\\n🎯 LOADING AVAILABLE DATA FOR ANALYSIS...")
        
#         # Define topics we're interested in
#         topics_to_check = {
#             "pos_setpoint": "/cascade_pid/position_setpoint",
#             "pos_current": "/cascade_pid/position_current",
#             "vel_setpoint": "/cascade_pid/velocity_setpoint", 
#             "vel_current": "/cascade_pid/velocity_current",
#             "pos_error": "/cascade_pid/position_error",
#             "vel_error": "/cascade_pid/velocity_error",
#             "control_outputs": "/cascade_pid/control_outputs",
#             "px4_position": "/fmu/out/vehicle_local_position",
#             "px4_attitude": "/fmu/out/vehicle_attitude",
#             "rate_setpoint": "/fmu/in/vehicle_rates_setpoint"
#         }
        
#         # Load available data
#         loaded_data = {}
        
#         for key, topic_name in topics_to_check.items():
#             topic_id, msg_type = safe_load_topic(cursor, topic_name)
#             if topic_id is not None:
#                 rows = read_topic_messages(cursor, topic_id)
                
#                 if rows:
#                     # Determine extraction method based on message type
#                     if "PointStamped" in msg_type:
#                         data = extract_xyz_from_point_stamped(rows, msg_type)
#                     elif "Vector3Stamped" in msg_type:
#                         data = extract_xyz_from_vector3_stamped(rows, msg_type)
#                     elif "Float64MultiArray" in msg_type:
#                         data = extract_float64_multiarray(rows, msg_type)
#                     else:
#                         print(f"⚠️ Unknown message type: {msg_type} for {topic_name}")
#                         continue
                    
#                     loaded_data[key] = data
#                     print(f"✅ Loaded {len(rows)} messages from {topic_name}")
#                 else:
#                     print(f"⚠️ No messages found in {topic_name}")
#             else:
#                 print(f"❌ Topic not found: {topic_name}")

#         conn.close()
        
#         if not loaded_data:
#             print("❌ No usable data found for plotting")
#             return
#         # 🔧 Normalize time to start from 0 s
#         normalize_time_axes(loaded_data)

#         # Create comprehensive plots
#         create_analysis_plots(loaded_data, bag_dir)

#     except Exception as e:
#         print(f"❌ Error processing bag: {e}")
#         import traceback
#         traceback.print_exc()
#         sys.exit(1)

# def create_analysis_plots(data, bag_dir):
#     """Create comprehensive analysis plots from loaded data"""
    
#     # Count available datasets to determine subplot layout
#     n_datasets = len(data)
#     print(f"\\n📊 Creating plots for {n_datasets} datasets...")
    
#     if n_datasets <= 4:
#         fig, axes = plt.subplots(2, 2, figsize=(15, 10))
#         axes = axes.flatten()
#     elif n_datasets <= 6:
#         fig, axes = plt.subplots(2, 3, figsize=(18, 10))
#         axes = axes.flatten()
#     else:
#         fig, axes = plt.subplots(3, 3, figsize=(20, 12))
#         axes = axes.flatten()
    
#     fig.suptitle(f'MPC Flight Analysis - {os.path.basename(bag_dir)}', fontsize=16)
    
#     plot_idx = 0
    
#     # Position tracking
#     if 'pos_setpoint' in data and 'pos_current' in data and plot_idx < len(axes):
#         t_sp, x_sp, y_sp, z_sp = data['pos_setpoint']
#         t_cur, x_cur, y_cur, z_cur = data['pos_current']
        
#         if len(t_sp) > 0 and len(t_cur) > 0:
#             axes[plot_idx].plot(t_sp, x_sp, 'b-', label='X setpoint', linewidth=2)
#             axes[plot_idx].plot(t_cur, x_cur, 'r--', label='X actual', linewidth=1.5)
#             axes[plot_idx].plot(t_sp, y_sp, 'g-', label='Y setpoint', linewidth=2)  
#             axes[plot_idx].plot(t_cur, y_cur, 'm--', label='Y actual', linewidth=1.5)
#             axes[plot_idx].set_title('Position XY Tracking')
#             axes[plot_idx].set_xlabel('Time [s]')
#             axes[plot_idx].set_ylabel('Position [m]')
#             axes[plot_idx].legend()
#             axes[plot_idx].grid(True)
#             plot_idx += 1
    
#     # Altitude tracking
#     if 'pos_setpoint' in data and 'pos_current' in data and plot_idx < len(axes):
#         t_sp, x_sp, y_sp, z_sp = data['pos_setpoint']
#         t_cur, x_cur, y_cur, z_cur = data['pos_current']
        
#         if len(t_sp) > 0 and len(t_cur) > 0:
#             axes[plot_idx].plot(t_sp, z_sp, 'b-', label='Z setpoint', linewidth=2)
#             axes[plot_idx].plot(t_cur, z_cur, 'r--', label='Z actual', linewidth=1.5)
#             axes[plot_idx].set_title('Altitude Tracking')
#             axes[plot_idx].set_xlabel('Time [s]')
#             axes[plot_idx].set_ylabel('Altitude [m]')
#             axes[plot_idx].legend()
#             axes[plot_idx].grid(True)
#             plot_idx += 1
    
#     # Position errors
#     if 'pos_error' in data and plot_idx < len(axes):
#         t_err, x_err, y_err, z_err = data['pos_error']
#         if len(t_err) > 0:
#             pos_error_norm = np.sqrt(x_err**2 + y_err**2 + z_err**2)
#             axes[plot_idx].plot(t_err, pos_error_norm, 'r-', linewidth=2)
#             axes[plot_idx].plot(t_err, np.abs(x_err), 'b--', alpha=0.7, label='|X error|')
#             axes[plot_idx].plot(t_err, np.abs(y_err), 'g--', alpha=0.7, label='|Y error|')
#             axes[plot_idx].plot(t_err, np.abs(z_err), 'm--', alpha=0.7, label='|Z error|')
#             axes[plot_idx].set_title('Position Errors')
#             axes[plot_idx].set_xlabel('Time [s]')
#             axes[plot_idx].set_ylabel('Error [m]')
#             axes[plot_idx].legend()
#             axes[plot_idx].grid(True)
#             plot_idx += 1
    
#     # Velocity tracking
#     if 'vel_setpoint' in data and 'vel_current' in data and plot_idx < len(axes):
#         t_vsp, vx_sp, vy_sp, vz_sp = data['vel_setpoint']
#         t_vcur, vx_cur, vy_cur, vz_cur = data['vel_current']
        
#         if len(t_vsp) > 0 and len(t_vcur) > 0:
#             vel_sp_norm = np.sqrt(vx_sp**2 + vy_sp**2)
#             vel_cur_norm = np.sqrt(vx_cur**2 + vy_cur**2)
#             axes[plot_idx].plot(t_vsp, vel_sp_norm, 'b-', label='Speed setpoint', linewidth=2)
#             axes[plot_idx].plot(t_vcur, vel_cur_norm, 'r--', label='Speed actual', linewidth=1.5)
#             axes[plot_idx].plot(t_vsp, vz_sp, 'g:', label='Vz setpoint', linewidth=2)
#             axes[plot_idx].plot(t_vcur, vz_cur, 'k:', label='Vz actual', linewidth=1.5)
#             axes[plot_idx].set_title('Velocity Tracking')
#             axes[plot_idx].set_xlabel('Time [s]')
#             axes[plot_idx].set_ylabel('Velocity [m/s]')
#             axes[plot_idx].legend()
#             axes[plot_idx].grid(True)
#             plot_idx += 1
    
#     # XY trajectory
#     if 'pos_setpoint' in data and 'pos_current' in data and plot_idx < len(axes):
#         t_sp, x_sp, y_sp, z_sp = data['pos_setpoint']
#         t_cur, x_cur, y_cur, z_cur = data['pos_current']
        
#         if len(x_sp) > 0 and len(x_cur) > 0:
#             axes[plot_idx].plot(x_sp, y_sp, 'b-', label='Setpoint trajectory', linewidth=2)
#             axes[plot_idx].plot(x_cur, y_cur, 'r--', label='Actual trajectory', linewidth=1.5)
#             axes[plot_idx].set_title('XY Trajectory')
#             axes[plot_idx].set_xlabel('X [m]')
#             axes[plot_idx].set_ylabel('Y [m]')
#             axes[plot_idx].legend()
#             axes[plot_idx].grid(True)
#             axes[plot_idx].axis('equal')
#             plot_idx += 1
    
#     # Control outputs
#     if 'control_outputs' in data and plot_idx < len(axes):
#         t_ctrl, ctrl_data = data['control_outputs']
#         if len(t_ctrl) > 0 and len(ctrl_data) > 0:
#             ctrl_array = np.array(ctrl_data)
#             if ctrl_array.shape[1] >= 3:
#                 axes[plot_idx].plot(t_ctrl, ctrl_array[:, 0], 'r-', label='Output 1', linewidth=1.5)
#                 axes[plot_idx].plot(t_ctrl, ctrl_array[:, 1], 'g-', label='Output 2', linewidth=1.5)
#                 axes[plot_idx].plot(t_ctrl, ctrl_array[:, 2], 'b-', label='Output 3', linewidth=1.5)
#                 if ctrl_array.shape[1] >= 4:
#                     axes[plot_idx].plot(t_ctrl, ctrl_array[:, 3], 'k-', label='Output 4', linewidth=1.5)
#                 axes[plot_idx].set_title('Control Outputs')
#                 axes[plot_idx].set_xlabel('Time [s]')
#                 axes[plot_idx].set_ylabel('Control Values')
#                 axes[plot_idx].legend()
#                 axes[plot_idx].grid(True)
#                 plot_idx += 1
    
#     # Hide unused subplots
#     for i in range(plot_idx, len(axes)):
#         axes[i].set_visible(False)
    
    
#     plt.tight_layout()
    
#     # Save the plot
#     plot_filename = f"{bag_dir}_comprehensive_analysis.png"
#     plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
#     print(f"📊 Comprehensive analysis saved: {plot_filename}")
    
#     plt.show()
#     print("✅ Enhanced plotting completed successfully!")

# if __name__ == "__main__":
#     main()
#!/usr/bin/env python3
"""
Enhanced MPC Bag Plotter
Plots comprehensive MPC and control data from rosbag
Shows all available topics and creates detailed analysis plots
+ Computes evaluation metrics (RMS errors, saturation, ...)
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
    # First, let's see what columns are available
    cursor.execute("PRAGMA table_info(topics)")
    columns = cursor.fetchall()
    column_names = [col[1] for col in columns]
    
    # Use available columns (some rosbag versions don't have message_count)
    if 'message_count' in column_names:
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
            marker = "🎯" if any(topic in name for topic in [
                "/cascade_pid/", "/mpc/", "/fmu/out/", "/fmu/in/"
            ]) else "📊"
            print(f"{marker} {name:45} | {msg_type:35} | {count:6} msgs")
    else:
        for name, msg_type in topics:
            # Count messages manually
            cursor.execute("SELECT COUNT(*) FROM messages m JOIN topics t ON m.topic_id = t.id WHERE t.name = ?", (name,))
            count = cursor.fetchone()[0]
            marker = "🎯" if any(topic in name for topic in [
                "/cascade_pid/", "/mpc/", "/fmu/out/", "/fmu/in/"
            ]) else "📊"
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
            t.append(ts * 1e-9)  # Convert ns to seconds
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


def normalize_time_axes(data):
    """Shift all time arrays so that the earliest timestamp becomes 0."""
    t_min = None
    
    # Find global minimum timestamp across all datasets
    for key, entry in data.items():
        t = entry[0]
        if t.size == 0:
            continue
        cur_min = float(np.min(t))
        if t_min is None or cur_min < t_min:
            t_min = cur_min
    
    if t_min is None:
        return  # nothing to do
    
    # Shift all timestamps
    for key, entry in data.items():
        t = entry[0] - t_min
        
        # Re-pack tuple depending on how many items it has
        if len(entry) == 4:
            # (t, x, y, z)
            data[key] = (t, entry[1], entry[2], entry[3])
        elif len(entry) == 2:
            # (t, data_array)
            data[key] = (t, entry[1])


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


def compute_metrics(data):
    """
    Compute evaluation metrics from loaded bag data:
    - RMS pos error [m]
    - Max pos error [m]
    - RMS vel error [m/s]
    - Mean thrust saturation [% of samples]
    """
    metrics = {}

    # --- Position error from setpoint - current ---
    if 'pos_setpoint' in data and 'pos_current' in data:
        t_sp, x_sp, y_sp, z_sp = data['pos_setpoint']
        t_cur, x_cur, y_cur, z_cur = data['pos_current']

        if x_sp.size > 0 and x_cur.size > 0:
            # Align by length (they should have same length/frequency)
            n = min(len(x_sp), len(x_cur))
            ex = x_sp[:n] - x_cur[:n]
            ey = y_sp[:n] - y_cur[:n]
            ez = z_sp[:n] - z_cur[:n]

            pos_err_norm = np.sqrt(ex**2 + ey**2 + ez**2)
            metrics['rms_pos'] = float(np.sqrt(np.mean(pos_err_norm**2)))
            metrics['max_pos'] = float(np.max(np.abs(pos_err_norm)))

    # --- Velocity error from setpoint - current ---
    if 'vel_setpoint' in data and 'vel_current' in data:
        t_vsp, vx_sp, vy_sp, vz_sp = data['vel_setpoint']
        t_vcur, vx_cur, vy_cur, vz_cur = data['vel_current']

        if vx_sp.size > 0 and vx_cur.size > 0:
            n = min(len(vx_sp), len(vx_cur))
            evx = vx_sp[:n] - vx_cur[:n]
            evy = vy_sp[:n] - vy_cur[:n]
            evz = vz_sp[:n] - vz_cur[:n]

            vel_err_norm = np.sqrt(evx**2 + evy**2 + evz**2)
            metrics['rms_vel'] = float(np.sqrt(np.mean(vel_err_norm**2)))

    # --- Thrust saturation from control_outputs ---
    # Assumption: last column of Float64MultiArray is normalized thrust in [0,1]
    if 'control_outputs' in data:
        t_ctrl, ctrl_data = data['control_outputs']
        if t_ctrl.size > 0 and len(ctrl_data) > 0:
            ctrl_array = np.array(ctrl_data)
            thrust = ctrl_array[:, -1]       # last column = thrust (check this!)

            # Debug print to sanity-check
            print(f"DEBUG thrust: min={thrust.min():.3f}, max={thrust.max():.3f}")

            # Saturated if very close to min or max
            # adjust 0.11 / 0.99 if your controller uses different limits
            sat_flags = (thrust >= 0.99) | (thrust <= 0.11)
            metrics['mean_thrust_sat_pct'] = float(100.0 * np.mean(sat_flags.astype(float)))

    # --- Attitude RMSE (roll, pitch, yaw) ---
    # Prefer using the explicit attitude_error topic if available
    if 'att_error' in data:
        t_att, roll_err, pitch_err, yaw_err = data['att_error']
        if roll_err.size > 0:
            # assume errors are in radians -> convert RMSE to degrees
            rad2deg = 180.0 / np.pi
            metrics['rms_roll_deg']  = float(np.sqrt(np.mean(roll_err**2))  * rad2deg)
            metrics['rms_pitch_deg'] = float(np.sqrt(np.mean(pitch_err**2)) * rad2deg)
            metrics['rms_yaw_deg']   = float(np.sqrt(np.mean(yaw_err**2))   * rad2deg)
    else:
        # Fallback: compute errors as setpoint - current if error topic missing
        if 'att_setpoint' in data and 'att_current' in data:
            t_sp, roll_sp, pitch_sp, yaw_sp = data['att_setpoint']
            t_cur, roll_cur, pitch_cur, yaw_cur = data['att_current']
            if roll_sp.size > 0 and roll_cur.size > 0:
                n = min(len(roll_sp), len(roll_cur))
                eroll  = roll_sp[:n]  - roll_cur[:n]
                epitch = pitch_sp[:n] - pitch_cur[:n]
                eyaw   = yaw_sp[:n]   - yaw_cur[:n]
                rad2deg = 180.0 / np.pi
                metrics['rms_roll_deg']  = float(np.sqrt(np.mean(eroll**2))  * rad2deg)
                metrics['rms_pitch_deg'] = float(np.sqrt(np.mean(epitch**2)) * rad2deg)
                metrics['rms_yaw_deg']   = float(np.sqrt(np.mean(eyaw**2))   * rad2deg)

    return metrics



def plot_metrics_summary(metrics, bag_dir):
    """Create a small figure summarizing scalar metrics and save it."""
    if not metrics:
        print("⚠️ No metrics could be computed (missing error/control topics).")
        return

    lines = []
    if 'rms_pos' in metrics:
        lines.append(f"RMS position error: {metrics['rms_pos']:.3f} m")
    if 'max_pos' in metrics:
        lines.append(f"Max position error:  {metrics['max_pos']:.3f} m")
    if 'rms_vel' in metrics:
        lines.append(f"RMS velocity error: {metrics['rms_vel']:.3f} m/s")
    if 'mean_thrust_sat_pct' in metrics:
        lines.append(f"Mean thrust saturation: {metrics['mean_thrust_sat_pct']:.1f} % of samples")

    # 🔽 new attitude metrics
    if 'rms_roll_deg' in metrics:
        lines.append(f"RMS roll error:  {metrics['rms_roll_deg']:.2f} deg")
    if 'rms_pitch_deg' in metrics:
        lines.append(f"RMS pitch error: {metrics['rms_pitch_deg']:.2f} deg")
    if 'rms_yaw_deg' in metrics:
        lines.append(f"RMS yaw error:   {metrics['rms_yaw_deg']:.2f} deg")

    print("\n📊 EVALUATION METRICS SUMMARY:")
    for L in lines:
        print("  " + L)

    fig, ax = plt.subplots(figsize=(6, 3))
    ax.axis('off')
    text = "\n".join(lines) if lines else "No metrics available."
    ax.text(0.05, 0.95, text, va='top', ha='left', fontsize=11, family='monospace')
    fig.suptitle(f'Metrics Summary – {os.path.basename(bag_dir)}', fontsize=14)

    metrics_filename = f"{bag_dir}_metrics_summary.png"
    plt.tight_layout()
    plt.savefig(metrics_filename, dpi=300, bbox_inches='tight')
    print(f"📊 Metrics summary saved: {metrics_filename}")



def main():
    if len(sys.argv) != 2:
        print("Usage: python3 enhanced_plot_mpc_bag.py <bag_directory>")
        print("Example: python3 enhanced_plot_mpc_bag.py mpc_flight_figure8_20251116_152221")
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

    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()

        # List all available topics first
        all_topics = list_all_topics(cursor)
        
        print("\n🎯 LOADING AVAILABLE DATA FOR ANALYSIS...")
        
        # Define topics we're interested in
        topics_to_check = {
            "pos_setpoint": "/cascade_pid/position_setpoint",
            "pos_current": "/cascade_pid/position_current",
            "vel_setpoint": "/cascade_pid/velocity_setpoint", 
            "vel_current": "/cascade_pid/velocity_current",
            "pos_error": "/cascade_pid/position_error",
            "vel_error": "/cascade_pid/velocity_error",
            "control_outputs": "/cascade_pid/control_outputs",
            "att_setpoint": "/cascade_pid/attitude_setpoint",
            "att_current": "/cascade_pid/attitude_current",
            "att_error": "/cascade_pid/attitude_error",     
            "px4_position": "/fmu/out/vehicle_local_position",
            "px4_attitude": "/fmu/out/vehicle_attitude",
            "rate_setpoint": "/fmu/in/vehicle_rates_setpoint"
        }
        
        # Load available data
        loaded_data = {}
        
        for key, topic_name in topics_to_check.items():
            topic_id, msg_type = safe_load_topic(cursor, topic_name)
            if topic_id is not None:
                rows = read_topic_messages(cursor, topic_id)
                
                if rows:
                    # Determine extraction method based on message type
                    if "PointStamped" in msg_type:
                        data = extract_xyz_from_point_stamped(rows, msg_type)
                    elif "Vector3Stamped" in msg_type:
                        data = extract_xyz_from_vector3_stamped(rows, msg_type)
                    elif "Float64MultiArray" in msg_type:
                        data = extract_float64_multiarray(rows, msg_type)
                    else:
                        print(f"⚠️ Unknown message type: {msg_type} for {topic_name}")
                        continue
                    
                    loaded_data[key] = data
                    print(f"✅ Loaded {len(rows)} messages from {topic_name}")
                else:
                    print(f"⚠️ No messages found in {topic_name}")
            else:
                print(f"❌ Topic not found: {topic_name}")

        conn.close()
        
        if not loaded_data:
            print("❌ No usable data found for plotting")
            return

        # 🔧 Normalize time to start from 0 s
        normalize_time_axes(loaded_data)

        # Compute and print metrics
        metrics = compute_metrics(loaded_data)
        plot_metrics_summary(metrics, bag_dir)

        # Create comprehensive plots
        create_analysis_plots(loaded_data, bag_dir)

    except Exception as e:
        print(f"❌ Error processing bag: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


def create_analysis_plots(data, bag_dir):
    """Create comprehensive analysis plots from loaded data"""
    
    # Count available datasets to determine subplot layout
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
    
    fig.suptitle(f'MPC Flight Analysis - {os.path.basename(bag_dir)}', fontsize=16)
    
    plot_idx = 0
    
    # Position tracking
    if 'pos_setpoint' in data and 'pos_current' in data and plot_idx < len(axes):
        t_sp, x_sp, y_sp, z_sp = data['pos_setpoint']
        t_cur, x_cur, y_cur, z_cur = data['pos_current']
        
        if len(t_sp) > 0 and len(t_cur) > 0:
            axes[plot_idx].plot(t_sp, x_sp, 'b-', label='X setpoint', linewidth=2)
            axes[plot_idx].plot(t_cur, x_cur, 'r--', label='X actual', linewidth=1.5)
            axes[plot_idx].plot(t_sp, y_sp, 'g-', label='Y setpoint', linewidth=2)  
            axes[plot_idx].plot(t_cur, y_cur, 'm--', label='Y actual', linewidth=1.5)
            axes[plot_idx].set_title('Position XY Tracking')
            axes[plot_idx].set_xlabel('Time [s]')
            axes[plot_idx].set_ylabel('Position [m]')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
    
    # Altitude tracking
    if 'pos_setpoint' in data and 'pos_current' in data and plot_idx < len(axes):
        t_sp, x_sp, y_sp, z_sp = data['pos_setpoint']
        t_cur, x_cur, y_cur, z_cur = data['pos_current']
        
        if len(t_sp) > 0 and len(t_cur) > 0:
            axes[plot_idx].plot(t_sp, z_sp, 'b-', label='Z setpoint', linewidth=2)
            axes[plot_idx].plot(t_cur, z_cur, 'r--', label='Z actual', linewidth=1.5)
            axes[plot_idx].set_title('Altitude Tracking')
            axes[plot_idx].set_xlabel('Time [s]')
            axes[plot_idx].set_ylabel('Altitude [m]')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
    
    # Position errors
    if 'pos_error' in data and plot_idx < len(axes):
        t_err, x_err, y_err, z_err = data['pos_error']
        if len(t_err) > 0:
            pos_error_norm = np.sqrt(x_err**2 + y_err**2 + z_err**2)
            axes[plot_idx].plot(t_err, pos_error_norm, 'r-', linewidth=2, label='‖pos error‖')
            axes[plot_idx].plot(t_err, np.abs(x_err), 'b--', alpha=0.7, label='|X error|')
            axes[plot_idx].plot(t_err, np.abs(y_err), 'g--', alpha=0.7, label='|Y error|')
            axes[plot_idx].plot(t_err, np.abs(z_err), 'm--', alpha=0.7, label='|Z error|')
            axes[plot_idx].set_title('Position Errors')
            axes[plot_idx].set_xlabel('Time [s]')
            axes[plot_idx].set_ylabel('Error [m]')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
    
    # Velocity tracking
    if 'vel_setpoint' in data and 'vel_current' in data and plot_idx < len(axes):
        t_vsp, vx_sp, vy_sp, vz_sp = data['vel_setpoint']
        t_vcur, vx_cur, vy_cur, vz_cur = data['vel_current']
        
        if len(t_vsp) > 0 and len(t_vcur) > 0:
            vel_sp_norm = np.sqrt(vx_sp**2 + vy_sp**2)
            vel_cur_norm = np.sqrt(vx_cur**2 + vy_cur**2)
            axes[plot_idx].plot(t_vsp, vel_sp_norm, 'b-', label='Speed setpoint', linewidth=2)
            axes[plot_idx].plot(t_vcur, vel_cur_norm, 'r--', label='Speed actual', linewidth=1.5)
            axes[plot_idx].plot(t_vsp, vz_sp, 'g:', label='Vz setpoint', linewidth=2)
            axes[plot_idx].plot(t_vcur, vz_cur, 'k:', label='Vz actual', linewidth=1.5)
            axes[plot_idx].set_title('Velocity Tracking')
            axes[plot_idx].set_xlabel('Time [s]')
            axes[plot_idx].set_ylabel('Velocity [m/s]')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            plot_idx += 1
    
    # XY trajectory
    if 'pos_setpoint' in data and 'pos_current' in data and plot_idx < len(axes):
        t_sp, x_sp, y_sp, z_sp = data['pos_setpoint']
        t_cur, x_cur, y_cur, z_cur = data['pos_current']
        
        if len(x_sp) > 0 and len(x_cur) > 0:
            axes[plot_idx].plot(x_sp, y_sp, 'b-', label='Setpoint trajectory', linewidth=2)
            axes[plot_idx].plot(x_cur, y_cur, 'r--', label='Actual trajectory', linewidth=1.5)
            axes[plot_idx].set_title('XY Trajectory')
            axes[plot_idx].set_xlabel('X [m]')
            axes[plot_idx].set_ylabel('Y [m]')
            axes[plot_idx].legend()
            axes[plot_idx].grid(True)
            axes[plot_idx].axis('equal')
            plot_idx += 1
    
    # Control outputs
    if 'control_outputs' in data and plot_idx < len(axes):
        t_ctrl, ctrl_data = data['control_outputs']
        if len(t_ctrl) > 0 and len(ctrl_data) > 0:
            ctrl_array = np.array(ctrl_data)
            n_cols = ctrl_array.shape[1]
            for i in range(min(n_cols, 4)):
                axes[plot_idx].plot(t_ctrl, ctrl_array[:, i], label=f'Output {i+1}', linewidth=1.5)
            axes[plot_idx].set_title('Control Outputs')
            axes[plot_idx].set_xlabel('Time [s]')
            axes[plot_idx].set_ylabel('Control Values')
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
    print("✅ Enhanced plotting completed successfully!")


if __name__ == "__main__":
    main()
