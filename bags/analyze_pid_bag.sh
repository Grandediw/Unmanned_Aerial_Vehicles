#!/bin/bash
#
# Convenient script to analyze PID rosbag data with enhanced plotting
# Usage: ./analyze_pid_bag.sh <bag_directory_name>
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if bag directory is provided
if [ $# -eq 0 ]; then
    echo -e "${RED}❌ Error: Please provide a PID bag directory name${NC}"
    echo -e "${BLUE}Usage: $0 <bag_directory_name>${NC}"
    echo ""
    echo -e "${YELLOW}Available PID bags in current directory:${NC}"
    # Adjust pattern to your actual PID bag naming if different
    ls -1 | grep -E "^pid_cascade.*_[0-9]{8}_[0-9]{6}$" | head -10 || \
        echo "   (no matching pid_cascade_* bags found here)"
    exit 1
fi

BAG_DIR="$1"

# Check if bag directory exists
if [ ! -d "$BAG_DIR" ]; then
    echo -e "${RED}❌ Error: Bag directory '$BAG_DIR' not found${NC}"
    echo ""
    echo -e "${YELLOW}Available PID bags in current directory:${NC}"
    ls -1 | grep -E "^pid_cascade.*_[0-9]{8}_[0-9]{6}$" | head -10 || \
        echo "   (no matching pid_cascade_* bags found here)"
    exit 1
fi

echo -e "${BLUE}🔍 Analyzing PID bag: $BAG_DIR${NC}"
echo ""

# Path to the PID plotting script
SCRIPT_PATH="../src/ROS2_PX4_Offboard_Example/px4_offboard/px4_offboard/enhanced_plot_pid_bag.py"

if [ ! -f "$SCRIPT_PATH" ]; then
    echo -e "${RED}❌ Error: PID enhanced plotting script not found at:${NC}"
    echo -e "    ${YELLOW}$SCRIPT_PATH${NC}"
    echo ""
    echo -e "${YELLOW}Tip:${NC} You can copy your MPC script and adapt it, e.g.:"
    echo -e "    cp enhanced_plot_mpc_bag.py enhanced_plot_pid_bag.py"
    exit 1
fi

echo -e "${GREEN}📊 Running enhanced PID analysis...${NC}"
python3 "$SCRIPT_PATH" "$BAG_DIR"

# Expected output plot file
PLOT_FILE="${BAG_DIR}_comprehensive_analysis.png"

if [ -f "$PLOT_FILE" ]; then
    echo ""
    echo -e "${GREEN}✅ PID analysis complete!${NC}"
    echo -e "${BLUE}📈 Plot saved as: $PLOT_FILE${NC}"
    
    # Show file size and info
    FILE_SIZE=$(ls -lh "$PLOT_FILE" | awk '{print $5}')
    echo -e "${YELLOW}📊 File size: $FILE_SIZE${NC}"
    
    # Optional: open with default image viewer
    # echo -e "${BLUE}🖼️  Opening plot...${NC}"
    # xdg-open "$PLOT_FILE" 2>/dev/null &
else
    echo -e "${RED}❌ Plot file not generated${NC}"
    exit 1
fi
