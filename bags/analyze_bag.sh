#!/bin/bash
#
# Convenient script to analyze rosbag data with enhanced plotting
# Usage: ./analyze_bag.sh <bag_directory_name>
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
    echo -e "${RED}❌ Error: Please provide a bag directory name${NC}"
    echo -e "${BLUE}Usage: $0 <bag_directory_name>${NC}"
    echo ""
    echo -e "${YELLOW}Available bags:${NC}"
    ls -1 | grep -E "^mpc_flight.*_[0-9]{8}_[0-9]{6}$" | head -10
    exit 1
fi

BAG_DIR="$1"

# Check if bag directory exists
if [ ! -d "$BAG_DIR" ]; then
    echo -e "${RED}❌ Error: Bag directory '$BAG_DIR' not found${NC}"
    echo ""
    echo -e "${YELLOW}Available bags in current directory:${NC}"
    ls -1 | grep -E "^mpc_flight.*_[0-9]{8}_[0-9]{6}$" | head -10
    exit 1
fi

echo -e "${BLUE}🔍 Analyzing bag: $BAG_DIR${NC}"
echo ""

# Run the enhanced plotting script
SCRIPT_PATH="../src/ROS2_PX4_Offboard_Example/px4_offboard/px4_offboard/enhanced_plot_mpc_bag.py"

if [ ! -f "$SCRIPT_PATH" ]; then
    echo -e "${RED}❌ Error: Enhanced plotting script not found at $SCRIPT_PATH${NC}"
    exit 1
fi

echo -e "${GREEN}📊 Running enhanced analysis...${NC}"
python3 "$SCRIPT_PATH" "$BAG_DIR"

# Show results
PLOT_FILE="${BAG_DIR}_comprehensive_analysis.png"
METRICS_FILE="${BAG_DIR}_metrics_summary.png"

if [ -f "$PLOT_FILE" ]; then
    echo ""
    echo -e "${GREEN}✅ Analysis complete!${NC}"
    echo -e "${BLUE}📈 Plot saved as: $PLOT_FILE${NC}"
    
    # Show file size and info
    FILE_SIZE=$(ls -lh "$PLOT_FILE" | awk '{print $5}')
    echo -e "${YELLOW}📊 File size: $FILE_SIZE${NC}"
    
    # Optional: Open with default image viewer (uncomment if desired)
    # echo -e "${BLUE}🖼️  Opening plot...${NC}"
    # xdg-open "$PLOT_FILE" 2>/dev/null &
else
    echo -e "${RED}❌ Plot file not generated${NC}"
    exit 1
fi