#!/bin/bash
# filepath: /home/grandediw/ros2_px4_offboard_example_ws/run_mpc_gp_enhanced.sh

echo "Starting GP-Enhanced MPC Demo..."

# Force GP mode
export GP_USE_MODEL=true
export GP_DATA_COLLECTION=false
export GP_MODEL_PATH="/home/grandediw/ros2_px4_offboard_example_ws/gp_models/gp_model_20251119_030043.pkl"

# Verify GP model exists
if [[ ! -f "$GP_MODEL_PATH" ]]; then
    echo "GP model not found: $GP_MODEL_PATH"
    exit 1
fi

echo "Using GP model: $(basename "$GP_MODEL_PATH")"

# Run the main script
exec "./run_mpc_velocity_control.sh" "$@"