# Quadrotor GP-MPC Workspace

A ROS 2 workspace for testing and comparing quadrotor trajectory tracking controllers in PX4 SITL.

This project compares three control approaches:

* Cascade PID
* Linear Model Predictive Control
* Gaussian Process-enhanced Model Predictive Control

The goal is to evaluate trajectory tracking performance in a realistic PX4, ROS 2, and Gazebo simulation environment.

---

## Project Documents

The full report and final presentation are included in the `docs` folder.

| Document           | File                                                                                                          |
| ------------------ | ------------------------------------------------------------------------------------------------------------- |
| Final Report       | [Unmanned_Aerial_Vehicles_Stefano_Tonini (4).pdf](docs/Unmanned_Aerial_Vehicles_Stefano_Tonini%20%284%29.pdf) |
| Final Presentation | [Unmanned_Vehicles_Presentation.pdf](docs/Unmanned_Vehicles_Presentation.pdf)                                 |

Expected structure:

```
docs/
├── Unmanned_Aerial_Vehicles_Stefano_Tonini (4).pdf
└── Unmanned_Vehicles_Presentation.pdf
```

---

## Overview

The workspace contains ROS 2 nodes and Python tools for quadrotor control, simulation, logging, and analysis.

PX4 is responsible for the low-level attitude and rate control. The ROS 2 nodes generate references, compute control commands, and process flight data.

Main components:

* Trajectory generation
* Cascade PID controller
* Linear MPC controller
* GP-MPC controller
* Rosbag logging
* Python analysis and plotting scripts
* Offline Gaussian Process training

---

## System Architecture

The control pipeline is organized as follows:

```
Trajectory Generator
        ↓
Controller
PID / MPC / GP-MPC
        ↓
Geometric Allocation
        ↓
PX4 Offboard Interface
        ↓
PX4 SITL + Gazebo
        ↓
Odometry Feedback
```

---

## Controllers

### Cascade PID

The cascade PID controller is used as a baseline controller.

It is organized into multiple control loops:

* Position control
* Velocity control
* Attitude control
* Body-rate command generation

The controller sends rate and thrust commands to PX4 in offboard mode.

### Linear MPC

The MPC controller uses a simplified double-integrator model.

State:

```
x = [x, y, z, vx, vy, vz]
```

Input:

```
u = [ax, ay, az, yaw_rate]
```

The MPC computes desired accelerations and yaw rate commands. These are converted into attitude and thrust commands before being sent to PX4.

The optimization problem includes:

* Tracking cost
* Control effort penalty
* Position constraints
* Velocity constraints
* Acceleration constraints

### GP-MPC

The GP-MPC controller extends the linear MPC with a Gaussian Process residual model.

The Gaussian Process is trained offline using flight data collected from PX4 SITL. During execution, the GP predicts a small correction to the MPC acceleration command.

The correction is saturated to keep the controller stable and avoid aggressive behavior.

---

## Requirements

* Ubuntu 22.04
* ROS 2 Humble
* PX4 Autopilot
* Gazebo
* Python 3
* CasADi
* NumPy
* SciPy
* Matplotlib
* Pandas
* scikit-learn

Install Python dependencies:

```
pip install numpy scipy matplotlib pandas casadi scikit-learn rosbags
```

---

## Installation

Clone the repository:

```
git clone <repository-url>
cd <repository-name>
```

Source ROS 2:

```
source /opt/ros/humble/setup.bash
```

Install dependencies:

```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```
colcon build
```

Source the workspace:

```
source install/setup.bash
```

---

## Running PX4 SITL

Start PX4 SITL with the X500 model:

```
cd <PX4-Autopilot>
make px4_sitl gz_x500
```

In a new terminal, source ROS 2 and the workspace:

```
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Run the desired ROS 2 node:

```
ros2 run <package_name> <node_name>
```

---

## Data Logging

Record all ROS 2 topics:

```
ros2 bag record -a
```

Or record selected topics:

```
ros2 bag record /fmu/out/vehicle_odometry /fmu/in/vehicle_rates_setpoint /fmu/in/vehicle_attitude_setpoint
```

The recorded data can be used for:

* Trajectory analysis
* Controller comparison
* Plot generation
* GP training

---

## Results

The controllers are tested on reference trajectories such as:

* Circular trajectory
* Figure-eight trajectory

The evaluation focuses on:

* Position tracking error
* Velocity tracking
* Control effort
* Stability in PX4 SITL
* Effect of the GP residual correction

For full results, see the [Final Report](docs/Unmanned_Aerial_Vehicles_Stefano_Tonini%20%284%29.pdf).

---

## Repository Structure

```
.
├── README.md
├── docs/
│   ├── Unmanned_Aerial_Vehicles_Stefano_Tonini (4).pdf
│   └── Unmanned_Vehicles_Presentation.pdf
├── src/
├── scripts/
└── data/
```

---

## Limitations

This project is focused on simulation and controller comparison.

Current limitations:

* GP training is offline only
* No real drone deployment
* No obstacle avoidance
* No online learning
* No custom nonlinear quadrotor simulator

---

## Author

Stefano Tonini

Project topic: Quadrotor trajectory tracking using PID, MPC, and GP-MPC
