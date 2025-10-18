# Quadrotor GP-MPC Workspace

This ROS2 workspace implements a complete system for quadrotor dynamics modeling with Gaussian Process learning and Model Predictive Control.

## System Architecture

### Components

1. **Quadrotor Dynamics** (`quadrotor_dynamics.py`)
   - Nonlinear quadrotor dynamics simulation
   - Aerodynamic effects modeling
   - Gaussian Process uncertainty integration
   - ROS2 interfaces for state/control

2. **Gaussian Process** (`gaussian_process.py`)
   - Online learning of model uncertainties
   - RBF kernel implementation
   - Hyperparameter optimization
   - Training data collection and management

3. **MPC Controller** (`mpc_controller.py`)
   - Nonlinear Model Predictive Control
   - Trajectory tracking
   - Obstacle avoidance
   - GP-enhanced predictions

4. **Main System** (`main.py`)
   - Coordinates all components
   - Data collection pipeline
   - Trajectory generation
   - Performance monitoring

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble Desktop
- Python packages: numpy, scipy, matplotlib, cvxpy

## Installation

1. **Install ROS2 Humble**:
   ```bash
   sudo apt update && sudo apt install ros-humble-desktop python3-rosdep2
   ```

2. **Install Python dependencies**:
   ```bash
   pip install numpy scipy matplotlib cvxpy
   ```

3. **Build the workspace**:
   ```bash
   cd ~/quadrotor_gp_mpc_ws
   colcon build --packages-select quadrotor_gp_mpc
   source install/setup.bash
   ```

## Usage

### Running Individual Components

1. **Quadrotor Dynamics Simulation**:
   ```bash
   ros2 run quadrotor_gp_mpc quadrotor_dynamics
   ```

2. **Gaussian Process Learning**:
   ```bash
   ros2 run quadrotor_gp_mpc gaussian_process
   ```

3. **MPC Controller**:
   ```bash
   ros2 run quadrotor_gp_mpc mpc_controller
   ```

### Running Complete System

For the integrated GP-MPC system:
```bash
ros2 run quadrotor_gp_mpc main_system
```

This will:
- Start all components in coordinated fashion
- Begin with hover trajectory (20s)
- Switch to circular trajectory (30s)
- Switch to figure-8 trajectory (30s)
- Collect GP training data throughout
- Save trained model at completion

### Visualization

To visualize the system in RViz:
```bash
rviz2
```
Add displays for:
- `/quadrotor/state` (Float64MultiArray)
- `/mpc/predicted_trajectory` (Path)
- `/tf` (TF transforms)

## System Parameters

### Quadrotor Physical Parameters
- Mass: 0.5 kg
- Inertia: Ixx=Iyy=0.0023, Izz=0.0046 kg⋅m²
- Drag coefficients: linear=0.25, angular=0.01

### MPC Parameters
- Prediction horizon: N=20 steps
- Time step: dt=0.1 s
- Control constraints: Thrust [0, 2g], Torque [-0.1, 0.1] N⋅m
- Cost weights: Position high, attitude moderate, control low

### GP Parameters
- Kernel: RBF with optimized hyperparameters
- Training data: max 1000 points (sliding window)
- Hyperparameter optimization: every 50 data points

## ROS2 Topics

### Published Topics
- `/quadrotor/state` (Float64MultiArray): Current state [x,y,z,vx,vy,vz,φ,θ,ψ,p,q,r]
- `/quadrotor/imu` (Imu): IMU measurements
- `/mpc/predicted_trajectory` (Path): MPC prediction horizon
- `/gp/prediction` (Float64MultiArray): GP uncertainty estimates
- `/gp/uncertainty` (Float64MultiArray): GP uncertainty variance

### Subscribed Topics
- `/quadrotor/control` (Float64MultiArray): Control inputs [T,τφ,τθ,τψ]
- `/mpc/reference` (Float64MultiArray): Reference trajectory
- `/gp/training_data` (Float64MultiArray): GP training samples
- `/gp/prediction_request` (Float64MultiArray): GP prediction requests

## Mathematical Formulation

### Quadrotor Dynamics
State: **x** = [x, y, z, vx, vy, vz, φ, θ, ψ, p, q, r]ᵀ
Control: **u** = [T, τφ, τθ, τψ]ᵀ

Dynamics: **ẋ** = f(**x**, **u**) + f_gp(**x**, **u**) + **w**

Where:
- f(**x**, **u**): Nominal dynamics
- f_gp(**x**, **u**): GP-learned uncertainty
- **w**: Process noise

### MPC Formulation
```
min Σ[k=0 to N-1] [||xk - xref||²Q + ||uk||²R] + ||xN - xref||²P

subject to:
    x[k+1] = f(x[k], u[k]) + f_gp(x[k], u[k])
    umin ≤ u[k] ≤ umax
    |φ|, |θ| ≤ π/4
    ||x[k] - xobs|| ≥ dsafe (obstacle avoidance)
```

### Gaussian Process
Learning residual: **r** = **ẋ**_true - f(**x**, **u**)
GP: f_gp(**x**, **u**) ~ GP(0, k(**x**,**u**; **x**',**u**'))
Kernel: k(x,x') = σ²f exp(-||x-x'||²/(2ℓ²))

## Performance Metrics

The system tracks:
- Position tracking error (RMS)
- Control effort
- GP prediction accuracy
- Computational performance

Typical performance:
- Tracking error: < 0.1m RMS
- GP training: ~1000 data points
- MPC solve time: < 50ms

## Troubleshooting

### Common Issues

1. **Import errors**: Ensure ROS2 environment is sourced
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/quadrotor_gp_mpc_ws/install/setup.bash
   ```

2. **cvxpy solver issues**: Install additional solvers
   ```bash
   pip install cvxopt osqp ecos
   ```

3. **Numerical instability**: Check GP hyperparameters and noise variance

4. **MPC infeasibility**: Verify control constraints and reference trajectory

### Debug Mode

For detailed logging, set environment variable:
```bash
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
```

## Extensions

### Adding New Trajectories
Modify `TrajectoryGenerator` class in `main.py`:
```python
def get_reference_state(self):
    if self.trajectory_type == "my_trajectory":
        # Implement your trajectory here
        ref_state[0:3] = [x_ref, y_ref, z_ref]
        ref_state[3:6] = [vx_ref, vy_ref, vz_ref]
```

### Custom Cost Functions
Modify `solve_mpc()` in `mpc_controller.py`:
```python
# Add custom cost terms
cost += custom_cost_function(x[k], u[k])
```

### Different GP Kernels
Implement new kernels in `gaussian_process.py`:
```python
class MyKernel:
    def __call__(self, X1, X2):
        # Implement kernel function
        return K_matrix
```

## References

1. "Gaussian Processes for Machine Learning" - Rasmussen & Williams
2. "Model Predictive Control: Theory and Design" - Rawlings et al.
3. "Learning-based Model Predictive Control for Aerial Robots" - Torrente et al.
4. "Gaussian Process-based Model Predictive Control" - Hewing et al.

## License

MIT License - see LICENSE file for details.

## Contributing

1. Fork the repository
2. Create feature branch
3. Add tests for new functionality
4. Submit pull request

## Contact

For questions or issues, please open a GitHub issue or contact the maintainer.
