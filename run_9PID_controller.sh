# #!/bin/bash

# echo "🚀 Starting PX4 SITL with Gazebo and Velocity PID Demo..."
# echo "📝 This will:"
# echo "   1. Launch PX4 SITL with Gazebo visualization"
# echo "   2. Spawn x500 drone automatically"
# echo "   3. Start MicroXRCE-DDS agent"
# echo "   4. Run the velocity PID demo (advanced velocity control)"
# echo ""
# echo "🎯 PID Controller features:"
# echo "   • Proportional-Integral-Derivative control"
# echo "   • Real-time velocity feedback"
# echo "   • Figure-8 velocity pattern"
# echo "   • Precise velocity tracking"
# echo ""

# # Colors for output
# RED='\033[0;31m'
# GREEN='\033[0;32m'
# BLUE='\033[0;34m'
# YELLOW='\033[1;33m'
# NC='\033[0m' # No Color

# # Set up ROS2 environment
# echo -e "${BLUE}🔧 Setting up ROS2 environment...${NC}"
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# cd ~/ros2_px4_offboard_example_ws
# source install/setup.bash

# # Start PX4 SITL with Gazebo in background
# echo -e "${BLUE}🌎 Starting PX4 SITL with Gazebo (this will open the GUI)...${NC}"
# export PX4_NO_PREFLIGHT_CHECKS=1
# cd ~/PX4-Autopilot
# make px4_sitl gz_x500 &
# PX4_PID=$!

# # Wait for PX4 to initialize
# echo -e "${YELLOW}⏳ Waiting for PX4 to initialize...${NC}"
# sleep 10

# echo -e "${GREEN}✅ PX4 SITL with Gazebo is running!${NC}"

# # Start MicroXRCE-DDS Agent
# echo -e "${BLUE}🔗 Starting MicroXRCE-DDS Agent...${NC}"
# MicroXRCEAgent udp4 -p 8888 &
# AGENT_PID=$!

# # Wait for agent to start
# echo -e "${YELLOW}⏳ Waiting for MicroXRCE-DDS Agent...${NC}"
# sleep 3

# echo -e "${GREEN}✅ MicroXRCE-DDS Agent is running!${NC}"
# echo ""

# # Start the velocity PID demo
# echo -e "${BLUE}🔄 Starting velocity PID demo...${NC}"
# echo -e "${GREEN}👀 You should see:${NC}"
# echo -e "${GREEN}   - Gazebo window with x500 drone${NC}"
# echo -e "${GREEN}   - Drone will ARM, takeoff with velocity control${NC}"
# echo -e "${GREEN}   - PID controllers will track velocity setpoints${NC}"
# echo -e "${GREEN}   - Beautiful figure-8 velocity patterns${NC}"
# echo -e "${GREEN}   - Real-time PID performance logging${NC}"
# echo ""
# echo -e "${YELLOW}Press Ctrl+C to stop everything${NC}"
# echo ""

# cd ~/ros2_px4_offboard_example_ws
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# source install/setup.bash
# ros2 run px4_offboard cascade_pid_demo

# # Cleanup function
# cleanup() {
#     echo -e "\n${RED}🧹 Cleaning up processes...${NC}"
#     kill $PX4_PID 2>/dev/null
#     kill $AGENT_PID 2>/dev/null
#     pkill -f px4 2>/dev/null
#     pkill -f gz 2>/dev/null
#     pkill -f MicroXRCE 2>/dev/null
#     exit 0
# }

# # Set up signal handlers
# trap cleanup SIGINT SIGTERM

# # Wait for user to stop
# wait
#!/bin/bash

echo "Starting PX4 SITL with Gazebo and Velocity PID Demo (with rosbag)..."
echo "This will:"
echo "   1. Launch PX4 SITL with Gazebo visualization"
echo "   2. Spawn x500 drone automatically"
echo "   3. Start MicroXRCE-DDS agent"
echo "   4. Run the velocity PID demo (advanced velocity control)"
echo "   5. (Optionally) record a rosbag for analysis"
echo ""
echo "PID Controller features:"
echo "   • Proportional-Integral-Derivative control"
echo "   • Real-time velocity feedback"
echo "   • Figure-8 velocity pattern"
echo "   • Precise velocity tracking"
echo ""

# ---------------- Colors ----------------
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ---------------- Configuration ----------------
# 1st arg: enable/disable rosbag (default: true)
ENABLE_ROSBAG=${1:-"true"}

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_DIR="bags"
BAG_NAME="${BAG_DIR}/pid_cascade_${TIMESTAMP}"

echo -e "${BLUE}Configuration:${NC}"
echo -e "   rosbag recording: ${YELLOW}${ENABLE_ROSBAG}${NC}"
if [[ "$ENABLE_ROSBAG" == "true" ]]; then
    echo -e "   Bag name: ${GREEN}${BAG_NAME}${NC}"
fi
echo ""

# Make sure bag directory exists (relative to where ros2 bag will run)
mkdir -p "$BAG_DIR"

# ---------------- Cleanup ----------------
PX4_PID=""
AGENT_PID=""
ROSBAG_PID=""

cleanup() {
    echo -e "\n${RED}🧹 Cleaning up processes...${NC}"

    # Stop rosbag if running
    if [[ -n "$ROSBAG_PID" ]]; then
        echo -e "${BLUE}Stopping rosbag recording (PID: ${ROSBAG_PID})...${NC}"
        kill "$ROSBAG_PID" 2>/dev/null
        wait "$ROSBAG_PID" 2>/dev/null
        echo -e "${GREEN} rosbag saved: ${BAG_NAME}${NC}"

        echo -e "${BLUE}Bag summary:${NC}"
        ros2 bag info "$BAG_NAME" 2>/dev/null || echo "   (Bag info not available)"
    fi

    # Kill PX4 / Gazebo / MicroXRCE
    if [[ -n "$PX4_PID" ]]; then kill "$PX4_PID" 2>/dev/null; fi
    if [[ -n "$AGENT_PID" ]]; then kill "$AGENT_PID" 2>/dev/null; fi

    pkill -f px4 2>/dev/null
    pkill -f gz 2>/dev/null
    pkill -f MicroXRCE 2>/dev/null

    echo -e "${GREEN}Cleanup done. Bye!${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# ---------------- ROS2 env ----------------
echo -e "${BLUE}🔧 Setting up ROS2 environment...${NC}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cd ~/ros2_px4_offboard_example_ws
source install/setup.bash

# ---------------- Start PX4 SITL ----------------
echo -e "${BLUE}🌎 Starting PX4 SITL with Gazebo (this will open the GUI)...${NC}"
export PX4_NO_PREFLIGHT_CHECKS=1
cd ~/PX4-Autopilot
make px4_sitl gz_x500 &
PX4_PID=$!

echo -e "${YELLOW}⏳ Waiting for PX4 to initialize...${NC}"
sleep 10
echo -e "${GREEN}✅ PX4 SITL with Gazebo is running!${NC}"

# ---------------- Start MicroXRCE-DDS Agent ----------------
echo -e "${BLUE}🔗 Starting MicroXRCE-DDS Agent...${NC}"
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!

echo -e "${YELLOW}⏳ Waiting for MicroXRCE-DDS Agent...${NC}"
sleep 3
echo -e "${GREEN}✅ MicroXRCE-DDS Agent is running!${NC}"
echo ""

# ---------------- Start rosbag (optional) ----------------
cd ~/ros2_px4_offboard_example_ws
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash

if [[ "$ENABLE_ROSBAG" == "true" ]]; then
    echo -e "${BLUE}📦 Starting rosbag recording...${NC}"
    echo -e "   Recording PID and PX4 topics for offline analysis"

    # Let topics come up
    sleep 3

    ros2 bag record \
        --output "$BAG_NAME" \
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
        /fmu/out/vehicle_local_position \
        /fmu/out/vehicle_attitude \
        /fmu/out/vehicle_status \
        /fmu/in/vehicle_rates_setpoint \
        /fmu/in/offboard_control_mode \
        /tf \
        /tf_static &

    ROSBAG_PID=$!
    sleep 3
    echo -e "${GREEN}✅ rosbag recording started (PID: ${ROSBAG_PID})${NC}"
    echo -e "   📁 Data will be saved to: ${GREEN}${BAG_NAME}${NC}"
else
    echo -e "${YELLOW}⏭️  Skipping rosbag recording (disabled)${NC}"
fi

# ---------------- Run the velocity PID demo ----------------
echo -e "${BLUE}Starting velocity PID demo...${NC}"
echo -e "${GREEN}You should see:${NC}"
echo -e "${GREEN}   - Gazebo window with x500 drone${NC}"
echo -e "${GREEN}   - Drone will ARM, takeoff with velocity control${NC}"
echo -e "${GREEN}   - PID controllers will track velocity setpoints${NC}"
echo -e "${GREEN}   - Figure-8 velocity patterns${NC}"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop everything and close the bag.${NC}"
echo ""

ros2 run px4_offboard cascade_pid_demo

# If the node exits normally (no Ctrl+C), still cleanup and close bag
cleanup
