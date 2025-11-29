#!/usr/bin/env python3
"""
Trajectory Definitions for MPC Testing

This module contains various trajectory patterns for testing MPC performance.
Each trajectory function returns position, velocity, and acceleration references
as functions of time.

Author: MPC Testing Framework
Date: November 2025
"""

import numpy as np
import math
from typing import Tuple, Callable


class TrajectoryGenerator:
    """Generate various trajectory patterns for MPC testing."""
    
    def __init__(self):
        """Initialize trajectory generator."""
        self.available_trajectories = {
            'figure_8': self.figure_8_trajectory,
            'circle': self.circular_trajectory,
            'square': self.square_trajectory,
            'lemniscate': self.lemniscate_trajectory,
            'spiral': self.spiral_trajectory,
            'waypoint_path': self.waypoint_trajectory,
            'hover': self.hover_trajectory,
            'sine_wave': self.sine_wave_trajectory,
            'oval': self.oval_trajectory,
            'cloverleaf': self.cloverleaf_trajectory
        }
    
    def get_trajectory(self, name: str) -> Callable:
        """Get trajectory function by name."""
        if name not in self.available_trajectories:
            raise ValueError(f"Unknown trajectory: {name}. Available: {list(self.available_trajectories.keys())}")
        return self.available_trajectories[name]
    
    def list_trajectories(self) -> list:
        """Return list of available trajectory names."""
        return list(self.available_trajectories.keys())
    
    # ======================== TRAJECTORY DEFINITIONS ========================
    
    def figure_8_trajectory(self, t: float, scale: float = 3.0, period: float = 20.0, 
                          center: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Classic figure-8 trajectory (infinity symbol).
        
        Args:
            t: Time in seconds
            scale: Size scaling factor
            period: Time to complete one full cycle
            center: Center position (x, y, z)
        
        Returns:
            position, velocity, acceleration as numpy arrays
        """
        omega = 2 * np.pi / period
        
        # Position
        x = center[0] + scale * np.sin(omega * t)
        y = center[1] + scale * np.sin(2 * omega * t) / 2
        z = center[2]
        
        # Velocity
        vx = scale * omega * np.cos(omega * t)
        vy = scale * omega * np.cos(2 * omega * t)
        vz = 0.0
        
        # Acceleration
        ax = -scale * omega**2 * np.sin(omega * t)
        ay = -2 * scale * omega**2 * np.sin(2 * omega * t)
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])
    
    def circular_trajectory(self, t: float, radius: float = 2.5, period: float = 15.0,
                          center: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Circular trajectory in XY plane.
        
        Args:
            t: Time in seconds
            radius: Circle radius
            period: Time to complete one circle
            center: Center position (x, y, z)
        """
        omega = 2 * np.pi / period
        
        # Position
        x = center[0] + radius * np.cos(omega * t)
        y = center[1] + radius * np.sin(omega * t)
        z = -center[2]  # Use correct NED altitude from center parameter
        #z = 2.0
        
        # Velocity
        vx = -radius * omega * np.sin(omega * t)
        vy = radius * omega * np.cos(omega * t)
        vz = 0.0
        
        # Acceleration
        ax = -radius * omega**2 * np.cos(omega * t)
        ay = -radius * omega**2 * np.sin(omega * t)
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])
    
    def square_trajectory(self, t: float, side_length: float = 4.0, period: float = 24.0,
                        center: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Square trajectory with smooth corners.
        
        Args:
            t: Time in seconds
            side_length: Length of square sides
            period: Time to complete full square
            center: Center position (x, y, z)
        """
        # Normalize time to [0, 1] for one complete cycle
        cycle_time = (t % period) / period
        half_side = side_length / 2
        
        if cycle_time < 0.25:  # Bottom edge (left to right)
            progress = cycle_time * 4
            x = center[0] + half_side * (2 * progress - 1)
            y = center[1] - half_side
            vx = 2 * half_side / (period / 4)
            vy = 0.0
        elif cycle_time < 0.5:  # Right edge (bottom to top)
            progress = (cycle_time - 0.25) * 4
            x = center[0] + half_side
            y = center[1] + half_side * (2 * progress - 1)
            vx = 0.0
            vy = 2 * half_side / (period / 4)
        elif cycle_time < 0.75:  # Top edge (right to left)
            progress = (cycle_time - 0.5) * 4
            x = center[0] + half_side * (1 - 2 * progress)
            y = center[1] + half_side
            vx = -2 * half_side / (period / 4)
            vy = 0.0
        else:  # Left edge (top to bottom)
            progress = (cycle_time - 0.75) * 4
            x = center[0] - half_side
            y = center[1] + half_side * (1 - 2 * progress)
            vx = 0.0
            vy = -2 * half_side / (period / 4)
        
        z = center[2]
        vz = 0.0
        
        # Simplified acceleration (could be smoothed)
        ax = 0.0
        ay = 0.0
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])
    
    def lemniscate_trajectory(self, t: float, scale: float = 3.0, period: float = 25.0,
                            center: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Lemniscate (Gerono's lemniscate) trajectory.
        
        Args:
            t: Time in seconds
            scale: Size scaling factor
            period: Time to complete one cycle
            center: Center position (x, y, z)
        """
        omega = 2 * np.pi / period
        
        # Position (parametric lemniscate)
        cos_t = np.cos(omega * t)
        sin_t = np.sin(omega * t)
        
        x = center[0] + scale * cos_t
        y = center[1] + scale * sin_t * cos_t
        z = center[2]
        
        # Velocity
        vx = -scale * omega * sin_t
        vy = scale * omega * (cos_t**2 - sin_t**2)
        vz = 0.0
        
        # Acceleration
        ax = -scale * omega**2 * cos_t
        ay = -4 * scale * omega**2 * sin_t * cos_t
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])
    
    def spiral_trajectory(self, t: float, max_radius: float = 3.0, period: float = 20.0, 
                        num_turns: float = 2.0, center: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Spiral trajectory expanding outward then contracting.
        
        Args:
            t: Time in seconds
            max_radius: Maximum spiral radius
            period: Time for complete spiral cycle
            num_turns: Number of spiral turns
            center: Center position (x, y, z)
        """
        cycle_time = (t % period) / period
        angle = 2 * np.pi * num_turns * cycle_time
        
        # Radius grows then shrinks
        if cycle_time < 0.5:
            radius = max_radius * (2 * cycle_time)
        else:
            radius = max_radius * (2 - 2 * cycle_time)
        
        # Position
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = center[2]
        
        # Velocity (simplified)
        omega = 2 * np.pi * num_turns / period
        vx = -radius * omega * np.sin(angle)
        vy = radius * omega * np.cos(angle)
        vz = 0.0
        
        # Acceleration (simplified)
        ax = -radius * omega**2 * np.cos(angle)
        ay = -radius * omega**2 * np.sin(angle)
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])
    
    def waypoint_trajectory(self, t: float, waypoints: list = None, segment_time: float = 8.0,
                          center: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Trajectory through predefined waypoints.
        
        Args:
            t: Time in seconds
            waypoints: List of (x, y) relative waypoint positions
            segment_time: Time to travel between waypoints
            center: Center position (x, y, z)
        """
        if waypoints is None:
            waypoints = [(2, 2), (-2, 2), (-2, -2), (2, -2)]  # Default square waypoints
        
        total_time = len(waypoints) * segment_time
        cycle_time = t % total_time
        
        # Find current segment
        segment_idx = int(cycle_time // segment_time)
        segment_progress = (cycle_time % segment_time) / segment_time
        
        # Current and next waypoints
        current_wp = waypoints[segment_idx]
        next_wp = waypoints[(segment_idx + 1) % len(waypoints)]
        
        # Linear interpolation between waypoints
        x = center[0] + current_wp[0] + (next_wp[0] - current_wp[0]) * segment_progress
        y = center[1] + current_wp[1] + (next_wp[1] - current_wp[1]) * segment_progress
        z = center[2]
        
        # Constant velocity between waypoints
        vx = (next_wp[0] - current_wp[0]) / segment_time
        vy = (next_wp[1] - current_wp[1]) / segment_time
        vz = 0.0
        
        # Zero acceleration (constant velocity)
        ax = 0.0
        ay = 0.0
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])
    
    def hover_trajectory(self, t: float, position: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Simple hover at fixed position.
        
        Args:
            t: Time in seconds (unused)
            position: Hover position (x, y, z)
        """
        pos = np.array(position)
        vel = np.array([0.0, 0.0, 0.0])
        acc = np.array([0.0, 0.0, 0.0])
        
        return pos, vel, acc
    
    def sine_wave_trajectory(self, t: float, amplitude: float = 2.0, frequency: float = 0.1,
                           axis: str = 'xy', center: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Sine wave trajectory along specified axis.
        
        Args:
            t: Time in seconds
            amplitude: Wave amplitude
            frequency: Wave frequency (Hz)
            axis: Wave axis ('x', 'y', 'xy', 'xyz')
            center: Center position (x, y, z)
        """
        omega = 2 * np.pi * frequency
        
        if axis == 'x':
            x = center[0] + amplitude * np.sin(omega * t)
            y = center[1]
            vx = amplitude * omega * np.cos(omega * t)
            vy = 0.0
            ax = -amplitude * omega**2 * np.sin(omega * t)
            ay = 0.0
        elif axis == 'y':
            x = center[0]
            y = center[1] + amplitude * np.sin(omega * t)
            vx = 0.0
            vy = amplitude * omega * np.cos(omega * t)
            ax = 0.0
            ay = -amplitude * omega**2 * np.sin(omega * t)
        elif axis == 'xy':
            x = center[0] + amplitude * np.sin(omega * t)
            y = center[1] + amplitude * np.sin(omega * t + np.pi/4)
            vx = amplitude * omega * np.cos(omega * t)
            vy = amplitude * omega * np.cos(omega * t + np.pi/4)
            ax = -amplitude * omega**2 * np.sin(omega * t)
            ay = -amplitude * omega**2 * np.sin(omega * t + np.pi/4)
        else:  # xyz
            x = center[0] + amplitude * np.sin(omega * t)
            y = center[1] + amplitude * np.sin(omega * t + np.pi/3)
            vx = amplitude * omega * np.cos(omega * t)
            vy = amplitude * omega * np.cos(omega * t + np.pi/3)
            ax = -amplitude * omega**2 * np.sin(omega * t)
            ay = -amplitude * omega**2 * np.sin(omega * t + np.pi/3)
        
        z = -center[2]
        vz = 0.0
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])
    
    def oval_trajectory(self, t: float, a: float = 3.0, b: float = 1.5, period: float = 18.0,
                      center: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Elliptical/oval trajectory.
        
        Args:
            t: Time in seconds
            a: Semi-major axis
            b: Semi-minor axis
            period: Time to complete one oval
            center: Center position (x, y, z)
        """
        omega = 2 * np.pi / period
        
        # Position
        x = center[0] + a * np.cos(omega * t)
        y = center[1] + b * np.sin(omega * t)
        z = center[2]
        
        # Velocity
        vx = -a * omega * np.sin(omega * t)
        vy = b * omega * np.cos(omega * t)
        vz = 0.0
        
        # Acceleration
        ax = -a * omega**2 * np.cos(omega * t)
        ay = -b * omega**2 * np.sin(omega * t)
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])
    
    def cloverleaf_trajectory(self, t: float, scale: float = 2.5, period: float = 30.0,
                            center: Tuple[float, float, float] = (0.0, 0.0, -2.0)) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Four-leaf clover trajectory.
        
        Args:
            t: Time in seconds
            scale: Size scaling factor
            period: Time to complete one clover
            center: Center position (x, y, z)
        """
        omega = 2 * np.pi / period
        
        # Cloverleaf parametric equations
        cos_t = np.cos(omega * t)
        sin_t = np.sin(omega * t)
        
        # Position
        r = scale * np.abs(np.cos(2 * omega * t))
        x = center[0] + r * cos_t
        y = center[1] + r * sin_t
        z = center[2]
        
        # Velocity (simplified approximation)
        vx = -scale * omega * sin_t * np.abs(np.cos(2 * omega * t))
        vy = scale * omega * cos_t * np.abs(np.cos(2 * omega * t))
        vz = 0.0
        
        # Acceleration (simplified)
        ax = -scale * omega**2 * cos_t * np.abs(np.cos(2 * omega * t))
        ay = -scale * omega**2 * sin_t * np.abs(np.cos(2 * omega * t))
        az = 0.0
        
        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az])


# ======================== TRAJECTORY CONFIGURATIONS ========================

TRAJECTORY_CONFIGS = {
    'easy_circle': {
        'name': 'circle',
        'params': {'radius': 6.0, 'period': 60.0}
    },
    'fast_circle': {
        'name': 'circle', 
        'params': {'radius': 6.5, 'period': 12.0}
    },
    'large_circle': {
        'name': 'circle',
        'params': {'radius': 10.0, 'period': 25.0}
    },
    'slow_figure8': {
        'name': 'figure_8',
        'params': {'scale': 5.5, 'period': 30.0}
    },
    'fast_figure8': {
        'name': 'figure_8',
        'params': {'scale': 3.0, 'period': 15.0}
    },
    'tight_figure8': {
        'name': 'figure_8',
        'params': {'scale': 1.5, 'period': 20.0}
    },
    'gentle_spiral': {
        'name': 'spiral',
        'params': {'max_radius': 2.5, 'period': 25.0, 'num_turns': 1.5}
    },
    'aggressive_spiral': {
        'name': 'spiral',
        'params': {'max_radius': 3.5, 'period': 18.0, 'num_turns': 3.0}
    },
    'square_path': {
        'name': 'square',
        'params': {'side_length': 10.0, 'period': 20.0}
    },
    'waypoint_square': {
        'name': 'waypoint_path',
        'params': {'waypoints': [(3, 3), (-3, 3), (-3, -3), (3, -3)], 'segment_time': 6.0}
    },
    'diamond_waypoints': {
        'name': 'waypoint_path', 
        'params': {'waypoints': [(0, 3), (3, 0), (0, -3), (-3, 0)], 'segment_time': 7.0}
    },
    'hover_test': {
        'name': 'hover',
        'params': {'position': (0.0, 0.0, -2.0)}
    },
    'sine_wave_x': {
        'name': 'sine_wave',
        'params': {'amplitude': 2.5, 'frequency': 0.08, 'axis': 'x'}
    },
    'oval_race': {
        'name': 'oval',
        'params': {'a': 3.5, 'b': 2.0, 'period': 22.0}
    },
    'clover_pattern': {
        'name': 'cloverleaf',
        'params': {'scale': 2.8, 'period': 35.0}
    }
}


def get_trajectory_function(config_name: str = 'slow_figure8'):
    """
    Get a trajectory function based on configuration name.
    
    Args:
        config_name: Name of trajectory configuration
        
    Returns:
        Trajectory function that takes time and returns (pos, vel, acc)
    """
    if config_name not in TRAJECTORY_CONFIGS:
        available = list(TRAJECTORY_CONFIGS.keys())
        raise ValueError(f"Unknown trajectory config: {config_name}. Available: {available}")
    
    config = TRAJECTORY_CONFIGS[config_name]
    generator = TrajectoryGenerator()
    trajectory_func = generator.get_trajectory(config['name'])
    params = config['params']
    
    def configured_trajectory(t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return trajectory_func(t, **params)
    
    return configured_trajectory


def list_available_trajectories():
    """Print all available trajectory configurations."""
    print("Available trajectory configurations:")
    print("=" * 50)
    
    categories = {
        'Circle Trajectories': ['easy_circle', 'fast_circle', 'large_circle'],
        'Figure-8 Trajectories': ['slow_figure8', 'fast_figure8', 'tight_figure8'],
        'Spiral Trajectories': ['gentle_spiral', 'aggressive_spiral'],
        'Geometric Paths': ['square_path', 'oval_race', 'clover_pattern'],
        'Waypoint Paths': ['waypoint_square', 'diamond_waypoints'],
        'Special Patterns': ['hover_test', 'sine_wave_x']
    }
    
    for category, configs in categories.items():
        print(f"\n{category}:")
        for config in configs:
            details = TRAJECTORY_CONFIGS[config]
            print(f"  • {config:18} -> {details['name']} with {details['params']}")


if __name__ == "__main__":
    # Demo usage
    list_available_trajectories()
    
    print("\n" + "="*50)
    print("Testing trajectory generation...")
    
    # Test a trajectory
    traj_func = get_trajectory_function('fast_figure8')
    pos, vel, acc = traj_func(5.0)  # t=5 seconds
    print(f"Fast Figure-8 at t=5s:")
    print(f"  Position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
    print(f"  Velocity: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}]")
    print(f"  Acceleration: [{acc[0]:.2f}, {acc[1]:.2f}, {acc[2]:.2f}]")