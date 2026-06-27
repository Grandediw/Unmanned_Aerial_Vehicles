Quadrotor GP-MPC Workspace

This repository contains a ROS 2 workspace for testing and comparing quadrotor trajectory tracking controllers in PX4 SITL.

The project focuses on three control approaches:

* Cascade PID controller
* Linear Model Predictive Controller
* Gaussian Process-enhanced MPC

The implementation is tested in a realistic PX4 / ROS 2 / Gazebo simulation environment.

⸻

Project Documents

The full report and final presentation are available here:

* Final Report
* Final Presentation

Place the files in the repository like this:

docs/
├── Unmanned_Aerial_Vehicles_Stefano_Tonini (4).pdf
└── Unmanned_Vehicles_Presentation.pdf

⸻

Overview

The workspace includes:

* a trajectory generator,
* a cascade PID controller,
* a linear MPC controller,
* a GP-MPC controller,
* rosbag logging tools,
* Python analysis and plotting scripts.

PX4 handles the low-level attitude and rate control.
The ROS 2 nodes generate trajectories, compute control commands, and analyze the flight data.

⸻

Controllers

Cascade PID

The cascade PID controller uses several control loops for:

* position control,
* velocity control,
* attitude control,
* body-rate command generation.

It is used as a simple and stable baseline controller.

Linear MPC

The MPC controller uses a double-integrator model with position and velocity states.

The state is:

x = [x, y, z, vx, vy, vz]

The input is:

u = [ax, ay, az, yaw_rate]

The MPC computes desired accelerations, which are converted into attitude and thrust commands.

GP-MPC

The GP-MPC controller extends the linear MPC with a Gaussian Process residual model.

The GP is trained offline from simulation data and provides small corrections to the MPC acceleration command.

The correction is limited to avoid unstable or aggressive behavior.

⸻

Requirements

* Ubuntu 22.04
* ROS 2 Humble
* PX4 Autopilot
* Gazebo
* Python 3

Python packages:

pip install numpy scipy matplotlib pandas casadi scikit-learn rosbags

⸻

Installation

Clone the repository:

git clone <repository-url>
cd <repository-name>

Source ROS 2:

source /opt/ros/humble/setup.bash

Build the workspace:

colcon build

Source the workspace:

source install/setup.bash

⸻

Running PX4 SITL

Start PX4 with the X500 model:

cd <PX4-Autopilot>
make px4_sitl gz_x500

In another terminal, source the workspace:

source /opt/ros/humble/setup.bash
source install/setup.bash

Then run the desired ROS 2 controller node.

Example:

ros2 run <package_name> <node_name>

⸻

Data Logging

To record all ROS 2 topics:

ros2 bag record -a

To record only selected topics:

ros2 bag record \
  /fmu/out/vehicle_odometry \
  /fmu/in/vehicle_rates_setpoint \
  /fmu/in/vehicle_attitude_setpoint

The recorded data can be used for plotting, analysis, and GP training.

⸻

Results

The controllers are tested on reference trajectories such as:

* circular trajectories,
* figure-eight trajectories.

The comparison focuses on:

* trajectory tracking error,
* velocity tracking,
* control effort,
* stability in PX4 SITL,
* effect of the GP correction.

More details are available in the Final Report.

⸻

Repository Structure

.
├── README.md
├── docs/
│   ├── Unmanned_Aerial_Vehicles_Stefano_Tonini (4).pdf
│   └── Unmanned_Vehicles_Presentation.pdf
├── src/
├── scripts/
└── data/

⸻

Limitations

This project is focused on simulation and controller comparison.

Current limitations:

* GP training is offline only,
* no real drone deployment,
* no obstacle avoidance,
* no custom nonlinear quadrotor simulator.

⸻

Author

Stefano Tonini

Project topic: Quadrotor trajectory tracking using PID, MPC, and GP-MPC

⸻
