Quadrotor GP-MPC Workspace

This ROS 2 workspace contains controllers, simulation tools, and analysis scripts for quadrotor trajectory tracking in PX4 SITL.

The project compares three control strategies:

* a 9-loop cascade PID controller,
* an acceleration-level linear Model Predictive Controller (MPC),
* an experimental Gaussian-Process-enhanced MPC (GP-MPC) trained offline from flight data.

The focus of this repository is the implementation, testing, and comparison of control approaches inside a realistic PX4 / ROS 2 / Gazebo SITL environment.

⸻

Project Documentation

This repository accompanies the final project on unmanned aerial vehicles and quadrotor control.

Document	Description
Final Report	Complete project report describing the methodology, controller design, implementation, experiments, and results.
Final Presentation	Final presentation summarizing the project, architecture, control strategies, and main results.

Recommended repository layout for documentation:

docs/
├── Unmanned_Aerial_Vehicles_Stefano_Tonini (4).pdf
├── Unmanned_Vehicles_Presentation.pdf
└── images/

⸻

Key Features

* ROS 2 Humble integration
* PX4 SITL simulation using the gz_x500 quadrotor model
* Offboard control through PX4 topics
* Cascade PID controller for conservative trajectory tracking
* Linear MPC based on a double-integrator model
* GP-MPC extension with offline Gaussian Process residual learning
* Geometric allocation from desired acceleration to attitude and thrust commands
* Rosbag-based logging and analysis
* Python scripts for plotting, comparison, and GP training

⸻

1. System Overview

1.1 High-Level Architecture

The workspace is built around PX4 SITL and ROS 2 Humble.

PX4 handles the low-level attitude and rate stabilization, while ROS 2 nodes provide trajectory generation, controller logic, data logging, and analysis.

Conceptually, the system is organized as follows:

Trajectory Generator
        ↓
Controller Node
(PID / MPC / GP-MPC)
        ↓
Geometric Allocation
        ↓
PX4 Offboard Interface
        ↓
PX4 SITL + Gazebo
        ↓
Vehicle Odometry / State Feedback

The main ROS 2 components are:

* a trajectory generator for circular and figure-eight references,
* a cascade PID controller,
* a linear MPC controller,
* an optional GP residual correction module,
* logging and plotting tools for post-flight analysis.

The vehicle dynamics are provided by PX4 SITL, meaning that this repository does not include a separate full nonlinear quadrotor simulator.

⸻

2. Controllers

2.1 Cascade PID Controller

The cascade PID controller is implemented as a 3-layer, 9-loop architecture:

Position PID
    ↓
Velocity setpoint
Velocity PID
    ↓
Attitude and thrust setpoint
Attitude PID
    ↓
Body-rate commands

The controller outputs VehicleRatesSetpoint messages and normalized thrust commands to PX4 in offboard mode.

This controller is designed to be conservative and stable, making it useful as a baseline and fallback controller.

⸻

2.2 Linear MPC Controller

The MPC controller uses a linear 6-state double-integrator model in position and velocity space.

The state is:

x = [x, y, z, v_x, v_y, v_z]^T

The control input is:

u = [a_x, a_y, a_z, r]^T

where:

* a_x, a_y, and a_z are desired accelerations,
* r is the yaw-rate command.

The MPC solves a finite-horizon optimization problem with:

* quadratic tracking cost,
* control effort penalty,
* position constraints,
* velocity constraints,
* acceleration constraints.

The optimization problem is implemented using CasADi and solved using a general-purpose solver such as IPOPT.

After solving, the desired acceleration is converted into attitude and thrust commands using a geometric allocation layer before being sent to PX4.

⸻

2.3 GP-Enhanced MPC Controller

The GP-MPC controller extends the linear MPC by adding a learned residual correction.

The Gaussian Process model is trained offline from SITL flight data. During execution, the GP predicts small residual corrections to the MPC acceleration command.

The GP input is the concatenation of the current state and control command:

[x_k; u_k]

where:

x_k \in \mathbb{R}^6

and

u_k \in \mathbb{R}^4

The GP output represents a residual correction to the translational dynamics.

To preserve stability and avoid aggressive behavior, the GP correction is deliberately limited using saturation before being applied to the MPC command.

The GP is currently trained offline from recorded rosbag or CSV flight data. It is not trained online during flight.

⸻

3. Tools and Utilities

This workspace includes scripts for:

* extracting data from rosbags,
* converting flight logs to CSV,
* plotting trajectories,
* comparing tracking errors,
* visualizing controller outputs,
* training Gaussian Process models,
* generating comprehensive analysis plots.

Typical analysis plots include:

* XY trajectory tracking,
* position error over time,
* velocity tracking,
* control input comparison,
* thrust and attitude command evolution.

Optional example image layout:

<p align="center">
  <img src="docs/images/xy_tracking.png" width="700">
</p>
<p align="center">
  <img src="docs/images/tracking_error.png" width="700">
</p>

⸻

4. Prerequisites

Operating System

* Ubuntu 22.04 LTS

ROS 2

* ROS 2 Humble

PX4

* PX4 Autopilot
* Gazebo simulation support
* Example SITL target: px4_sitl gz_x500

Python Dependencies

The main Python dependencies are:

* numpy
* scipy
* matplotlib
* casadi
* pandas
* scikit-learn
* rosbags or ROS 2 bag utilities

Install typical system dependencies:

sudo apt update
sudo apt install ros-humble-desktop python3-rosdep2 python3-colcon-common-extensions

Install Python packages:

pip install numpy scipy matplotlib casadi pandas scikit-learn rosbags

⸻

5. Installation

Clone the repository:

git clone <repository-url>
cd <repository-name>

Source ROS 2:

source /opt/ros/humble/setup.bash

Install dependencies:

rosdep update
rosdep install --from-paths src --ignore-src -r -y

Build the workspace:

colcon build

Source the workspace:

source install/setup.bash

⸻

6. Running the Simulation

Start PX4 SITL with the X500 model:

cd <PX4-Autopilot>
make px4_sitl gz_x500

In another terminal, source ROS 2 and the workspace:

source /opt/ros/humble/setup.bash
source install/setup.bash

Launch the desired controller or node according to the package structure.

Example:

ros2 launch <package_name> <launch_file>.py

⸻

7. Running the Controllers

Cascade PID

ros2 run <package_name> <pid_node>

Linear MPC

ros2 run <package_name> <mpc_node>

GP-MPC

ros2 run <package_name> <gp_mpc_node>

The exact package and node names may depend on the final repository structure.

⸻

8. Data Logging

Record flight data using rosbag:

ros2 bag record -a

Or record selected topics:

ros2 bag record \
  /fmu/out/vehicle_odometry \
  /fmu/in/vehicle_rates_setpoint \
  /fmu/in/vehicle_attitude_setpoint \
  /trajectory_reference

Logged data can then be used for:

* trajectory analysis,
* controller comparison,
* GP training,
* performance evaluation.

⸻

9. GP Training Workflow

The GP residual model is trained offline using recorded flight data.

Typical workflow:

1. Run a simulation using PID or MPC.
2. Record the relevant ROS 2 topics.
3. Extract position, velocity, and control data.
4. Compute residual dynamics from measured and predicted behavior.
5. Train a Gaussian Process model.
6. Save the trained model.
7. Load the model inside the GP-MPC controller.

The GP correction is intentionally kept small and saturated during control execution.

⸻

10. Results and Evaluation

The controllers are evaluated in PX4 SITL using reference trajectories such as:

* circular trajectories,
* figure-eight trajectories.

The comparison focuses on:

* trajectory tracking accuracy,
* position error,
* velocity tracking,
* control smoothness,
* robustness inside PX4 SITL,
* effect of GP residual correction on MPC performance.

The full discussion of methodology and results is available in the Final Report.

⸻

11. Repository Structure

A recommended structure for the repository is:

.
├── README.md
├── docs/
│   ├── Unmanned_Aerial_Vehicles_Stefano_Tonini (4).pdf
│   ├── Unmanned_Vehicles_Presentation.pdf
│   └── images/
├── src/
│   ├── trajectory_generator/
│   ├── cascade_pid_controller/
│   ├── mpc_controller/
│   ├── gp_mpc_controller/
│   └── analysis_tools/
├── scripts/
│   ├── plotting/
│   ├── rosbag_processing/
│   └── gp_training/
└── data/
    ├── rosbags/
    ├── csv/
    └── trained_models/

⸻

12. Limitations

This project focuses on controller implementation and comparison inside PX4 SITL.

Current limitations include:

* no full nonlinear custom quadrotor model,
* no online GP training,
* no complex obstacle avoidance,
* GP corrections are intentionally small for safety,
* controller performance depends on PX4 SITL configuration and tuning.

⸻

13. Future Work

Possible extensions include:

* online GP model adaptation,
* more advanced nonlinear MPC formulation,
* obstacle avoidance,
* wind disturbance testing,
* comparison with additional learning-based controllers,
* deployment on real hardware,
* improved controller tuning and benchmarking.

⸻

14. Author

Stefano Tonini

Project topic: Unmanned Aerial Vehicles, Quadrotor Control, MPC, and Gaussian Process-enhanced MPC

⸻

15. License

Add the chosen license for this repository here.

Example:

MIT License
