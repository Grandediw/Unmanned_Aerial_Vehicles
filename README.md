# Quadrotor GP-MPC Workspace

This ROS 2 workspace contains a set of controllers and analysis tools for **quadrotor trajectory tracking in PX4 SITL**, including:

- a 9-loop **cascade PID** controller,
- an acceleration-level **linear MPC** controller based on a double integrator model,
- an experimental **Gaussian-Process-augmented MPC (GP-MPC)** that adds small residual corrections to MPC accelerations using a GP trained **offline** from flight data.

The focus is on **implementation and comparison** inside a realistic PX4/ROS 2 stack, not on full nonlinear modeling or complex obstacle avoidance.

---

## 1. System Overview

### 1.1 High-Level Architecture

The workspace is built around PX4 SITL (e.g. `gz_x500`) and ROS 2 Humble:

- PX4 handles **low-level attitude and rate control**.
- ROS 2 nodes implement:
  - a **trajectory generator** (circle, figure-eight),
  - a **cascade PID controller** that outputs `VehicleRatesSetpoint`,
  - a **linear MPC controller** that computes desired accelerations and yaw rate,
  - an optional **GP residual module** that refines MPC accelerations using a pre-trained Gaussian Process,
  - logging / plotting tools for analysis (rosbag + Python scripts).

Dynamics are provided by **PX4 SITL**, i.e. there is no separate full nonlinear simulator in this repo.

### 1.2 Main Components (Conceptual)

> Exact filenames/executables may differ slightly; see `quadrotor_gp_mpc/nodes/` and the launch files in your repo.

1. **Cascade PID Controller node**
   - 3-layer, 9-loop cascade:
     - Position PIDs → velocity setpoints  
     - Velocity PIDs → attitude + thrust setpoints  
     - Attitude PIDs → body-rate commands
   - Sends `VehicleRatesSetpoint` + normalized thrust to PX4 in offboard mode.
   - Tuned for conservative, “emergency-safe” behavior.

2. **MPC Controller node**
   - Linear **6-state double-integrator** model in position–velocity space:
     - State: \\(x = [x, y, z, v_x, v_y, v_z]^\top\\)
     - Input: \\(u = [a_x, a_y, a_z, r]^\top\\) (accelerations + yaw rate)
   - Finite-horizon quadratic cost with box constraints on position, velocity, and accelerations.
   - Implemented with **CasADi** and solved with a general-purpose NLP/QP solver (e.g. IPOPT).
   - Uses a **geometric allocation layer** to convert desired accelerations to thrust magnitude + roll/pitch/yaw commands, which are then passed through PX4 attitude/rate control.

3. **GP-Enhanced MPC node (GP-MPC)**
   - Wraps the same linear MPC controller.
   - Loads a **pre-trained Gaussian Process** model that approximates residual translational dynamics based on past flight data.
   - After each MPC solve, applies a **small, saturated correction** to the first acceleration command before geometric allocation:
     - GP input: concatenated \\([x_k; u_k]\\) (6-D state + 4-D control).
     - GP output: residual on acceleration/velocity dynamics.
     - Corrections are deliberately small to avoid destabilizing PX4.
   - GP is currently trained **offline** from SITL logs (CSV/rosbag), not online during flight.

4. **Tools and Utilities**
   - Rosbag parsing and plotting scripts (e.g. “comprehensive analysis” plots: XY tracks, errors, velocities, control outputs).
   - Scripts to extract data from rosbag and train a GP model from recorded trajectories.

---

## 2. Prerequisites

- **OS:** Ubuntu 22.04 LTS  
- **ROS 2:** Humble (desktop install)  
- **PX4 SITL:** PX4 Autopilot cloned and built (e.g. `px4_sitl gz_x500`)  
- **Python 3.x** with packages:
  - `numpy`
  - `scipy`
  - `matplotlib`
  - `casadi`
  - `pandas`
  - `scikit-learn` (for the GP model)
  - `rosbags` / `ros2bag` utilities as needed

Install typical dependencies:

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-rosdep2 python3-colcon-common-extensions

# Inside a virtualenv or user env:
pip install numpy scipy matplotlib casadi pandas scikit-learn
