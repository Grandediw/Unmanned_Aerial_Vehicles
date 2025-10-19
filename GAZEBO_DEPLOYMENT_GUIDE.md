# Gazebo gz_x500 Deployment with MPC and GP Controllers

## Overview

This guide explains how to deploy and run the MPC (Model Predictive Controller) and GP (Gaussian Process) learning system on the **Gazebo gz_x500 quadrotor** simulator.

## Prerequisites

### Required Packages
```bash
# Core ROS2 packages
sudo apt-get install ros-humble-gazebo-ros
sudo apt-get install ros-humble-ros-gz-bridge
sudo apt-get install ros-humble-robot-state-publisher
sudo apt-get install ros-humble-rviz2

# Python dependencies (if not installed)
pip install numpy scipy matplotlib cvxpy
```

### Gazebo Models
The gz_x500 is a common model in Gazebo. Ensure it's available:
```bash
# Check if gz_x500 model exists
find ~/.gz/models -name "*x500*"
```

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     GAZEBO SIMULATOR                        │
│  (Physics engine, gz_x500 drone model, sensor simulation)  │
└────────────────────────┬────────────────────────────────────┘
                         │ (ROS2 Bridge)
         ┌───────────────┼───────────────┐
         │               │               │
    ┌────▼────┐    ┌─────▼─────┐  ┌────▼────┐
    │   MPC   │    │     GP    │  │ Reference│
    │Controller│    │  Learning │  │Trajectory│
    │  Node   │    │   Node    │  │Generator │
    └────┬────┘    └─────┬─────┘  └────┬────┘
         │               │             │
         └───────────────┼─────────────┘
                         │
                    ┌────▼────┐
                    │Performance│
                    │  Metrics  │
                    │   Node    │
                    └───────────┘
```

### Components

1. **MPC Controller Node** (`mpc_controller_node`)
   - Reads current state from Gazebo
   - Computes control commands (thrust, torques)
   - Publishes to Gazebo
   - Rate: 100 Hz

2. **Gaussian Process Node** (`gaussian_process_node`)
   - Collects state-control pairs from simulation
   - Learns dynamics model online
   - Provides predictions to MPC
   - Training rate: Configurable

3. **Reference Trajectory Generator** (`reference_trajectory_node`)
   - Generates target trajectories
   - Supports: Setpoint, Circular, Figure-8, Custom
   - Rate: 50 Hz

4. **Performance Metrics Monitor** (`performance_metrics_node`)
   - Tracks control performance
   - Logs metrics to JSON
   - Publishes real-time statistics

## Quick Start

### 1. Build the Package

```bash
cd ~/quadrotor_gp_mpc_ws
colcon build --packages-select quadrotor_gp_mpc
source install/setup.bash
```

### 2. Launch Gazebo with MPC and GP

```bash
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py
```

#### Optional Parameters

```bash
# Run in headless mode (no GUI)
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py headless:=true

# Set initial position
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    x_pose:=1.0 y_pose:=2.0 z_pose:=0.5

# Test with custom trajectory
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    trajectory_type:=circular

# Disable GP learning
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    gp_enabled:=false

# Set control rate (Hz)
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    control_rate:=200
```

### 3. Monitor Performance

In another terminal:
```bash
# View ROS topics
ros2 topic list

# Echo control commands
ros2 topic echo /gz_x500/control_input

# View performance metrics
ros2 topic echo /metrics/summary

# Check GP learning progress
ros2 topic echo /gp/metrics
```

### 4. View Results

Results are saved to `/tmp/mpc_gz_metrics.json`:
```bash
# Display metrics
cat /tmp/mpc_gz_metrics.json | jq .

# Monitor in real-time
watch -n 1 'cat /tmp/mpc_gz_metrics.json | jq ".position_error"'
```

## Launch File Parameters

### Global Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim_time` | `true` | Use Gazebo simulation clock |
| `headless` | `false` | Run without GUI |
| `verbose` | `false` | Enable verbose logging |

### Initial Position

| Parameter | Default | Description |
|-----------|---------|-------------|
| `x_pose` | `0.0` | Initial X position (meters) |
| `y_pose` | `0.0` | Initial Y position (meters) |
| `z_pose` | `0.5` | Initial Z position (meters) |

### MPC Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mpc_enabled` | `true` | Enable MPC controller |
| `control_rate` | `100` | Control loop rate (Hz) |
| `horizon` | `20` | MPC prediction horizon (steps) |
| `dt` | `0.01` | Control timestep (seconds) |
| `max_thrust` | `2.0` | Maximum thrust (N) |
| `max_torque` | `0.1` | Maximum torque (N⋅m) |

### GP Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gp_enabled` | `true` | Enable GP learning |
| `kernel` | `rbf` | RBF kernel function |
| `length_scale` | `0.5` | GP length scale |
| `sigma` | `0.1` | GP output variance |
| `noise_variance` | `0.01` | Sensor noise variance |

### Reference Trajectory Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `trajectory_type` | `setpoint` | Trajectory type |
| `setpoint_x` | `0.0` | Target X position (m) |
| `setpoint_y` | `0.0` | Target Y position (m) |
| `setpoint_z` | `1.0` | Target Z altitude (m) |
| `circular_radius` | `1.0` | Circular trajectory radius (m) |
| `circular_period` | `10.0` | Circular trajectory period (s) |

### Trajectory Types

- **`setpoint`**: Hover at fixed position (default)
- **`circular`**: Circular trajectory in XY plane
- **`figure8`**: Figure-8 pattern trajectory
- **`hover`**: Simple hover (equivalent to setpoint)

## ROS2 Topics

### Published Topics

#### From MPC Controller
- `/gz_x500/control_input` (Float64MultiArray)
  - Contents: [thrust, tau_p, tau_q, tau_r]
  - Rate: 100 Hz

- `/mpc/metrics` (Float64MultiArray)
  - Contents: [position_error, velocity_magnitude, thrust_cmd, step_count]
  - Rate: 1 Hz

#### From Gaussian Process
- `/gz_x500/gp_predictions` (Float64MultiArray)
  - Contents: [dx, dy, dz] (predicted state derivatives)
  - Rate: Configurable

- `/gp/metrics` (Float64MultiArray)
  - Contents: [training_points, gp_training_points, total_steps]
  - Rate: 1 Hz

#### From Reference Trajectory Generator
- `/reference_trajectory` (Float64MultiArray)
  - Contents: [px, py, pz, vx, vy, vz, φ, θ, ψ, p, q, r] (12D state)
  - Rate: 50 Hz

#### From Performance Monitor
- `/metrics/summary` (Float64MultiArray)
  - Contents: [pos_err, mean_err, max_err, vel_err, ...]
  - Rate: 1 Hz

### Subscribed Topics (from Gazebo)

- `/gz_x500/state` (Float64MultiArray)
  - Current state: [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]
  - Provided by: Gazebo Bridge

- `/clock` (rosgraph_msgs/Clock)
  - Simulation time
  - Provided by: Gazebo

## Example Scenarios

### Scenario 1: Simple Hover Test
```bash
# Launch with default parameters (hovers at 1m altitude)
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py

# In another terminal, monitor
ros2 topic echo /metrics/summary --once
```

**Expected results**:
- Drone lifts off and stabilizes at 1m altitude
- Position error decreases over ~5-10 seconds
- Final steady-state error < 0.1m

### Scenario 2: Circular Trajectory Tracking
```bash
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    trajectory_type:=circular \
    circular_radius:=2.0 \
    circular_period:=15.0 \
    circular_height:=2.0
```

**Expected results**:
- Drone follows circular path at 2m radius
- Altitude maintained at 2m
- Smooth tracking with bounded errors

### Scenario 3: GP Learning Validation
```bash
# Launch with GP enabled
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    gp_enabled:=true

# Monitor GP training progress
watch -n 1 'ros2 topic echo /gp/metrics --once'
```

**Expected results**:
- Training point count increases over time
- GP learns quadrotor dynamics progressively
- Metrics show learning convergence

### Scenario 4: High-Performance Control
```bash
# Aggressive gains, fast control rate
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    control_rate:=200 \
    max_thrust:=3.0
```

**Expected results**:
- Faster response to reference changes
- Lower settling times
- May require tuning for stability

## Troubleshooting

### Issue: "Cannot find module rclpy"

**Solution**:
```bash
source /opt/ros/humble/setup.bash
source ~/quadrotor_gp_mpc_ws/install/setup.bash
```

### Issue: Gazebo crashes or doesn't start

**Solution**:
```bash
# Check Gazebo installation
gazebo --version

# Try headless mode
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py headless:=true

# Clear Gazebo cache
rm -rf ~/.gz/
```

### Issue: Drone doesn't respond to control commands

**Check**:
1. MPC node is running: `ros2 node list | grep mpc`
2. Topics are published: `ros2 topic list | grep control`
3. Check for errors: `ros2 node info /mpc_controller_node`

**Solution**:
```bash
# Restart with verbose logging
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py verbose:=true
```

### Issue: High CPU usage or lag

**Solution**:
- Reduce control rate: `control_rate:=50`
- Run headless: `headless:=true`
- Disable GPU rendering in Gazebo settings

## Performance Optimization

### For Real-Time Testing
```bash
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    control_rate:=200 \
    headless:=true \
    verbose:=false
```

### For Data Collection
```bash
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    trajectory_type:=circular \
    control_rate:=100 \
    gp_enabled:=true
```

### For Visualization
```bash
ros2 launch quadrotor_gp_mpc gazebo_gz_x500_mpc.launch.py \
    control_rate:=50 \
    headless:=false
```

## Output Files

### Metrics Log
- **Location**: `/tmp/mpc_gz_metrics.json`
- **Format**: JSON with timestamped metrics
- **Contents**: Position errors, velocity, thrust, GP training data

### Sample Output
```json
{
  "timestamp": "2025-10-19T14:30:45.123456",
  "elapsed_time": 125.45,
  "step_count": 12545,
  "position_error": {
    "mean": 0.0456,
    "max": 0.1234,
    "min": 0.0001
  },
  "velocity_error": {
    "mean": 0.0234,
    "max": 0.0891
  },
  "gp_training_points": 2500,
  "current_state": [0.05, -0.02, 1.00, 0.01, -0.01, 0.00, ...],
  "reference_state": [0.00, 0.00, 1.00, 0.00, 0.00, 0.00, ...]
}
```

## Next Steps

1. **Validate Control Performance**
   - Run test scenarios from above
   - Collect metrics over 5+ minutes
   - Analyze convergence behavior

2. **Tune Control Gains**
   - Adjust `kp`, `kd`, `ki` in MPC node
   - Balance speed vs. stability
   - Document tuning process

3. **GP Learning Analysis**
   - Monitor training data collection
   - Verify prediction accuracy
   - Compare with actual dynamics

4. **Real Hardware Deployment**
   - Port code to actual x500 hardware
   - Validate safety constraints
   - Test in real-world conditions

## References

- [Gazebo Sim Documentation](https://gazebosim.org)
- [ROS 2 Humble Guide](https://docs.ros.org/en/humble/)
- [gz_x500 Model](https://github.com/PX4/PX4-Autopilot)

---

**Created**: October 19, 2025  
**Last Updated**: October 19, 2025  
**Status**: Ready for Production
