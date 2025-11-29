# #!/bin/bash

# echo "🧠 Starting MPC Cascade Demo..."
# echo "🔧 Features: Model Predictive Control with Body Rate Commands"

# # Set RMW implementation for consistent communication
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# # Configuration options (edit these as needed)
# TRAJECTORY_TYPE=${1:-"figure8"}         # First argument or default
# TAKEOFF_HEIGHT=${2:-"3.0"}             # Second argument or default

# # Display configuration
# echo "⚙️  Configuration:"
# echo "   📊 Trajectory: $TRAJECTORY_TYPE pattern"
# echo "   🎮 Control Mode: Body Rate + Thrust (bypasses PX4 attitude control)"
# echo "   🧠 Model: 12-state quadrotor dynamics"
# echo "   ⏱️  Prediction: 0.5s horizon (25 steps @ 0.02s)"
# echo "   🚁 Takeoff Height: ${TAKEOFF_HEIGHT}m"
# echo ""

# # Function to cleanup on exit
# cleanup() {
#     echo ""
#     echo "🧹 Cleaning up processes..."
#     pkill -f px4 > /dev/null 2>&1
#     pkill -f MicroXRCE > /dev/null 2>&1
#     pkill -f gz > /dev/null 2>&1
#     pkill -f ros2 > /dev/null 2>&1
#     exit 0
# }
# trap cleanup SIGINT SIGTERM

# # Kill any existing processes
# echo "🧹 Cleaning up existing processes..."
# pkill -f px4 > /dev/null 2>&1
# pkill -f MicroXRCE > /dev/null 2>&1
# pkill -f gz > /dev/null 2>&1
# sleep 2

# # Check dependencies
# echo "🔍 Checking dependencies..."
# if ! python3 -c "import casadi" 2>/dev/null; then
#     echo "❌ CasADi not found! Installing..."
#     pip install casadi
#     echo "✅ CasADi installed"
# else
#     echo "✅ CasADi available"
# fi

# # Start PX4 SITL
# echo "🛫 Starting PX4 SITL..."
# cd ~/PX4-Autopilot
# export PX4_NO_PREFLIGHT_CHECKS=1
# make px4_sitl gz_x500 &
# PX4_PID=$!

# # Wait for PX4 to initialize
# echo "⏳ Waiting for PX4 to initialize (30s)..."
# sleep 30

# # Start MicroXRCE Agent
# echo "🔗 Starting MicroXRCE-DDS Agent..."
# MicroXRCEAgent udp4 -p 8888 &
# AGENT_PID=$!

# # Wait for communication to establish
# echo "⏳ Waiting for ROS2-PX4 communication (10s)..."
# sleep 10

# # Navigate to workspace and build
# cd ~/ros2_px4_offboard_example_ws
# echo "🔧 Building workspace..."
# colcon build --packages-select px4_offboard --cmake-args -DCMAKE_BUILD_TYPE=Release

# # Source the workspace
# source install/setup.bash

# # Verify the MPC node exists
# if ! ros2 pkg executables px4_offboard | grep -q mpc; then
#     echo "❌ MPC node not found in executables!"
#     echo "📋 Available executables:"
#     ros2 pkg executables px4_offboard
#     echo ""
#     echo "🔧 Adding MPC entry point to setup.py..."
    
#     # Check if entry point exists in setup.py
#     if ! grep -q "mpc.*mpc:main" src/ROS2_PX4_Offboard_Example/px4_offboard/setup.py; then
#         echo "⚠️  MPC entry point missing from setup.py!"
#         echo "Please add this line to your setup.py entry_points:"
#         echo "'mpc = px4_offboard.mpc:main',"
#         echo ""
#         echo "Rebuilding with entry point fix..."
#         colcon build --packages-select px4_offboard --cmake-args -DCMAKE_BUILD_TYPE=Release
#         source install/setup.bash
#     fi
# fi

# # Start MPC Cascade Demo
# echo ""
# echo "🎯 Starting MPC Cascade Demo..."
# echo "📊 Control Flow:"
# echo "   Phase 1-2: Initialize & Arm (2-4s)"
# echo "   Phase 3:   Takeoff to ${TAKEOFF_HEIGHT}m using PX4 position control (4-8s)"
# echo "   Phase 4:   Stabilize at hover (8-20s)" 
# echo "   Phase 5:   MPC Figure-8 trajectory with body rate commands (20s+)"
# echo ""
# echo "⚠️  WARNING: Phase 5 bypasses PX4 attitude controller!"
# echo "   MPC sends direct roll/pitch/yaw rate + thrust commands"
# echo ""

# # Check if we can find the MPC executable
# if ros2 pkg executables px4_offboard | grep -q mpc; then
#     echo "🚀 Launching MPC controller..."
#     echo "📺 Monitor Gazebo to see the drone execute figure-8 pattern"
#     echo "📊 Use PlotJuggler to monitor /cascade_pid/* topics"
#     echo ""
    
#     # Run the MPC controller
#     ros2 run px4_offboard mpc
    
# else
#     echo "❌ Failed to find MPC executable!"
#     echo "🔍 Debug info:"
#     echo "Available executables:"
#     ros2 pkg executables px4_offboard
#     echo ""
#     echo "Package path:"
#     ros2 pkg prefix px4_offboard
#     echo ""
#     echo "🛠️  Manual run attempt (if entry point exists):"
#     echo "Try running: python3 src/ROS2_PX4_Offboard_Example/px4_offboard/px4_offboard/mpc.py"
# fi

# # Cleanup on exit
# cleanup

#!/bin/bash

echo "🧠 Starting MPC Cascade Demo with rosbag recording..."
echo "🔧 Features: Model Predictive Control with Body Rate Commands"

# Set RMW implementation for consistent communication
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Configuration options (edit these as needed)
TRAJECTORY_TYPE=${1:-"figure8"}         # First argument or default
TAKEOFF_HEIGHT=${2:-"3.0"}             # Second argument or default

# rosbag configuration
ENABLE_ROSBAG=${3:-"true"}             # Third argument or default
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_DIR="bags"
mkdir -p "$BAG_DIR"
BAG_NAME="${BAG_DIR}/mpc_flight_${TRAJECTORY_TYPE}_${TIMESTAMP}"

# Display configuration
echo "⚙️  Configuration:"
echo "   📊 Trajectory: $TRAJECTORY_TYPE pattern"
echo "   🎮 Control Mode: Body Rate + Thrust (bypasses PX4 attitude control)"
echo "   🧠 Model: 12-state quadrotor dynamics"
echo "   ⏱️  Prediction: 0.5s horizon (25 steps @ 0.02s)"
echo "   🚁 Takeoff Height: ${TAKEOFF_HEIGHT}m"
echo "   📦 rosbag Recording: $ENABLE_ROSBAG"
if [[ "$ENABLE_ROSBAG" == "true" ]]; then
    echo "   📁 Bag Name: $BAG_NAME"
fi
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🧹 Cleaning up processes..."
    
    # Stop rosbag recording if running
    if [[ -n "$ROSBAG_PID" ]]; then
        echo "🛑 Stopping rosbag recording..."
        kill $ROSBAG_PID 2>/dev/null
        wait $ROSBAG_PID 2>/dev/null
        echo "✅ rosbag saved: $BAG_NAME"
        
        # Show bag info
        echo "📊 Bag summary:"
        ros2 bag info $BAG_NAME 2>/dev/null || echo "   (Bag info not available)"
    fi
    
    pkill -f px4 > /dev/null 2>&1
    pkill -f MicroXRCE > /dev/null 2>&1
    pkill -f gz > /dev/null 2>&1
    pkill -f ros2 > /dev/null 2>&1
    exit 0
}
trap cleanup SIGINT SIGTERM

# Kill any existing processes
echo "🧹 Cleaning up existing processes..."
pkill -f px4 > /dev/null 2>&1
pkill -f MicroXRCE > /dev/null 2>&1
pkill -f gz > /dev/null 2>&1
pkill -f "ros2 bag" > /dev/null 2>&1  # Kill any existing rosbag
sleep 2

# Check dependencies
echo "🔍 Checking dependencies..."
if ! python3 -c "import casadi" 2>/dev/null; then
    echo "❌ CasADi not found! Installing..."
    pip install casadi
    echo "✅ CasADi installed"
else
    echo "✅ CasADi available"
fi

# Start PX4 SITL
echo "🛫 Starting PX4 SITL..."
cd ~/PX4-Autopilot
export PX4_NO_PREFLIGHT_CHECKS=1
make px4_sitl gz_x500 &
PX4_PID=$!

# Wait for PX4 to initialize
echo "⏳ Waiting for PX4 to initialize (30s)..."
sleep 30

# Start MicroXRCE Agent
echo "🔗 Starting MicroXRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!

# Wait for communication to establish
echo "⏳ Waiting for ROS2-PX4 communication (10s)..."
sleep 10

# Navigate to workspace and build
cd ~/ros2_px4_offboard_example_ws
echo "🔧 Building workspace..."
colcon build --packages-select px4_offboard --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Verify the MPC node exists
if ! ros2 pkg executables px4_offboard | grep -q mpc; then
    echo "❌ MPC node not found in executables!"
    echo "📋 Available executables:"
    ros2 pkg executables px4_offboard
    echo ""
    echo "🔧 Adding MPC entry point to setup.py..."
    
    # Check if entry point exists in setup.py
    if ! grep -q "mpc.*mpc:main" src/ROS2_PX4_Offboard_Example/px4_offboard/setup.py; then
        echo "⚠️  MPC entry point missing from setup.py!"
        echo "Please add this line to your setup.py entry_points:"
        echo "'mpc = px4_offboard.mpc:main',"
        echo ""
        echo "Rebuilding with entry point fix..."
        colcon build --packages-select px4_offboard --cmake-args -DCMAKE_BUILD_TYPE=Release
        source install/setup.bash
    fi
fi

# Start rosbag recording if enabled
ROSBAG_PID=""
if [[ "$ENABLE_ROSBAG" == "true" ]]; then
    echo "📦 Starting rosbag recording..."
    echo "   Recording topics for MPC analysis..."
    
    # Wait a moment for topics to be available
    sleep 3
    
    # Start rosbag recording in background with comprehensive topic list
    ros2 bag record \
        --output $BAG_NAME \
        --max-bag-size 1000000000 \
        --compression-mode file \
        --compression-format zstd \
        /cascade_pid/control_outputs \
        /cascade_pid/position_setpoint \
        /cascade_pid/position_current \
        /cascade_pid/velocity_setpoint \
        /cascade_pid/velocity_current \
        /cascade_pid/attitude_setpoint \
        /cascade_pid/attitude_current \
        /cascade_pid/position_error \
        /cascade_pid/velocity_error \
        /cascade_pid/attitude_error \
        /mpc/gp_metrics \
        /mpc/gp_training_data \
        /mpc/performance \
        /fmu/out/vehicle_local_position \
        /fmu/out/vehicle_attitude \
        /fmu/out/vehicle_status \
        /fmu/in/vehicle_rates_setpoint \
        /fmu/in/offboard_control_mode \
        /tf \
        /tf_static &
    
    ROSBAG_PID=$!
    
    # Give rosbag time to start
    sleep 3
    
    echo "✅ rosbag recording started (PID: $ROSBAG_PID)"
    echo "📁 Data will be saved to: $BAG_NAME"
else
    echo "⏭️  Skipping rosbag recording (disabled)"
fi

# Start MPC Cascade Demo
echo ""
echo "🎯 Starting MPC Cascade Demo..."
echo "📊 Control Flow:"
echo "   Phase 1-2: Initialize & Arm (2-4s)"
echo "   Phase 3:   Takeoff to ${TAKEOFF_HEIGHT}m using PX4 position control (4-8s)"
echo "   Phase 4:   Stabilize at hover (8-20s)" 
echo "   Phase 5:   MPC Figure-8 trajectory with body rate commands (20s+)"
echo ""
echo "⚠️  WARNING: Phase 5 bypasses PX4 attitude controller!"
echo "   MPC sends direct roll/pitch/yaw rate + thrust commands"
echo ""

# Check if we can find the MPC executable
if ros2 pkg executables px4_offboard | grep -q mpc; then
    echo "🚀 Launching MPC controller..."
    echo "📺 Monitor Gazebo to see the drone execute figure-8 pattern"
    echo "📊 Use PlotJuggler to monitor /cascade_pid/* topics"
    if [[ "$ENABLE_ROSBAG" == "true" ]]; then
        echo "📦 Flight data being recorded to: $BAG_NAME"
        echo "   Use 'ros2 bag play $BAG_NAME' to replay later"
        echo "   Use 'ros2 bag info $BAG_NAME' for bag details"
    fi
    echo ""
    echo "🛑 Press Ctrl+C to stop flight and save data"
    echo ""
    
    # Run the MPC controller
    ros2 run px4_offboard mpc
    
else
    echo "❌ Failed to find MPC executable!"
    echo "🔍 Debug info:"
    echo "Available executables:"
    ros2 pkg executables px4_offboard
    echo ""
    echo "Package path:"
    ros2 pkg prefix px4_offboard
    echo ""
    echo "🛠️  Manual run attempt (if entry point exists):"
    echo "Try running: python3 src/ROS2_PX4_Offboard_Example/px4_offboard/px4_offboard/mpc.py"
fi

# Cleanup on exit
cleanup