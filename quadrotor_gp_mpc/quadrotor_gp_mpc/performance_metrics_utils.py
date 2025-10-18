#!/usr/bin/env python3
"""
Utility script for integrating metrics collection into your ROS2 nodes.

This script provides helper functions and decorators to easily add metrics
collection to your GP and MPC nodes without much code modification.
"""

import functools
import time
from typing import Callable, Any, Dict
from quadrotor_gp_mpc.performance_metrics import GPMetrics, MPCMetrics
import numpy as np


class MetricsCollector:
    """Helper class for collecting and managing metrics in ROS2 nodes."""
    
    def __init__(self, node_type: str = 'gp'):
        """
        Initialize metrics collector.
        
        Args:
            node_type: 'gp' for Gaussian Process, 'mpc' for MPC Controller
        """
        self.node_type = node_type
        
        if node_type == 'gp':
            self.metrics = GPMetrics()
        elif node_type == 'mpc':
            self.metrics = MPCMetrics()
        else:
            raise ValueError(f"Unknown node type: {node_type}")
        
        self.start_time = time.time()
    
    def get_elapsed_time(self) -> float:
        """Get elapsed time since creation."""
        return time.time() - self.start_time
    
    def record_gp_step(self, n_training_points: int, pred_error: np.ndarray,
                      uncertainty: np.ndarray, hyperparams: Dict):
        """Record GP metrics for current step."""
        if self.node_type != 'gp':
            raise ValueError("Can only record GP metrics for 'gp' node type")
        
        self.metrics.add_metrics(
            n_data=n_training_points,
            pred_err=pred_error,
            uncertainty=uncertainty,
            hyperparams=hyperparams,
            timestamp=self.get_elapsed_time()
        )
    
    def record_mpc_step(self, reference: np.ndarray, actual: np.ndarray,
                       control: np.ndarray, tracking_error: np.ndarray,
                       solve_time: float, constraint_violated: bool = False):
        """Record MPC metrics for current step."""
        if self.node_type != 'mpc':
            raise ValueError("Can only record MPC metrics for 'mpc' node type")
        
        self.metrics.add_step(
            reference=reference,
            actual=actual,
            control=control,
            tracking_error=tracking_error,
            solve_time=solve_time,
            constraint_violated=constraint_violated,
            timestamp=self.get_elapsed_time()
        )
    
    def get_metrics(self):
        """Return the metrics object."""
        return self.metrics


def measure_time(func: Callable) -> Callable:
    """Decorator to measure function execution time."""
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        elapsed = time.time() - start_time
        return result, elapsed
    return wrapper


def track_gp_metrics(collector: MetricsCollector) -> Callable:
    """Decorator to automatically track GP metrics."""
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            result = func(self, *args, **kwargs)
            
            # Extract metrics from the node
            try:
                n_data = len(self.X_train) if hasattr(self, 'X_train') else 0
                pred_error = kwargs.get('pred_error', np.zeros(12))
                uncertainty = kwargs.get('uncertainty', np.ones(12))
                hyperparams = {
                    'length_scale': self.kernel.length_scale if hasattr(self, 'kernel') else 0,
                    'signal_variance': self.kernel.signal_variance if hasattr(self, 'kernel') else 0
                }
                
                collector.record_gp_step(n_data, pred_error, uncertainty, hyperparams)
            except Exception as e:
                print(f"Error recording GP metrics: {e}")
            
            return result
        return wrapper
    return decorator


def track_mpc_metrics(collector: MetricsCollector) -> Callable:
    """Decorator to automatically track MPC metrics."""
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            start_time = time.time()
            result = func(self, *args, **kwargs)
            solve_time = time.time() - start_time
            
            # Extract metrics from the node
            try:
                reference = self.reference_trajectory[0] if hasattr(self, 'reference_trajectory') else np.zeros(12)
                actual = self.current_state if hasattr(self, 'current_state') else np.zeros(12)
                control = kwargs.get('control', np.zeros(4))
                tracking_error = actual - reference
                constraint_violated = kwargs.get('constraint_violated', False)
                
                collector.record_mpc_step(reference, actual, control, tracking_error,
                                        solve_time, constraint_violated)
            except Exception as e:
                print(f"Error recording MPC metrics: {e}")
            
            return result
        return wrapper
    return decorator


class QuickMetricsPlotter:
    """Quick utility for plotting metrics with minimal code."""
    
    @staticmethod
    def plot_all(gp_metrics: 'GPMetrics' = None, mpc_metrics: 'MPCMetrics' = None,
                output_dir: str = './results'):
        """Plot all available metrics."""
        from quadrotor_gp_mpc.performance_metrics import PerformanceVisualizer
        import os
        
        os.makedirs(output_dir, exist_ok=True)
        visualizer = PerformanceVisualizer()
        
        if gp_metrics is not None:
            path = f"{output_dir}/gp_metrics.png"
            visualizer.plot_gp_metrics(gp_metrics, save_path=path)
            print(f"✓ GP metrics saved to {path}")
        
        if mpc_metrics is not None:
            path = f"{output_dir}/mpc_metrics.png"
            visualizer.plot_mpc_metrics(mpc_metrics, save_path=path)
            print(f"✓ MPC metrics saved to {path}")
        
        if gp_metrics is not None and mpc_metrics is not None:
            path = f"{output_dir}/comparison.png"
            visualizer.plot_comparison(gp_metrics, mpc_metrics, save_path=path)
            print(f"✓ Comparison plot saved to {path}")
    
    @staticmethod
    def save_all(gp_metrics: 'GPMetrics' = None, mpc_metrics: 'MPCMetrics' = None,
                output_dir: str = './results'):
        """Save all metrics to JSON."""
        from quadrotor_gp_mpc.performance_metrics import MetricsLogger
        
        logger = MetricsLogger(output_dir)
        
        if gp_metrics is not None:
            logger.save_gp_metrics(gp_metrics, 'gp_metrics.json')
            print(f"✓ GP metrics saved to {output_dir}/gp_metrics.json")
        
        if mpc_metrics is not None:
            logger.save_mpc_metrics(mpc_metrics, 'mpc_metrics.json')
            print(f"✓ MPC metrics saved to {output_dir}/mpc_metrics.json")


# Example usage patterns

def example_gp_node_integration():
    """Example: Integrating metrics into a GP node."""
    print("GP Node Integration Example:")
    print("-" * 50)
    
    code_example = '''
from quadrotor_gp_mpc.performance_metrics_utils import MetricsCollector

class MyGaussianProcess:
    def __init__(self):
        # ... your initialization code ...
        self.metrics = MetricsCollector(node_type='gp')
    
    def train(self):
        """Train the GP model."""
        # ... your training code ...
        
        # Record metrics
        pred_error = compute_prediction_error()
        uncertainty = compute_uncertainty()
        
        self.metrics.record_gp_step(
            n_training_points=len(self.X_train),
            pred_error=pred_error,
            uncertainty=uncertainty,
            hyperparams={
                'length_scale': self.kernel.length_scale,
                'signal_variance': self.kernel.signal_variance
            }
        )
    
    def get_metrics_plot(self):
        """Generate metrics plot."""
        from quadrotor_gp_mpc.performance_metrics_utils import QuickMetricsPlotter
        QuickMetricsPlotter.plot_all(gp_metrics=self.metrics.get_metrics())
    '''
    print(code_example)


def example_mpc_node_integration():
    """Example: Integrating metrics into an MPC node."""
    print("\nMPC Node Integration Example:")
    print("-" * 50)
    
    code_example = '''
from quadrotor_gp_mpc.performance_metrics_utils import MetricsCollector

class MyMPCController:
    def __init__(self):
        # ... your initialization code ...
        self.metrics = MetricsCollector(node_type='mpc')
    
    def control_step(self):
        """Execute one MPC control step."""
        import time
        start = time.time()
        
        # ... your MPC solving code ...
        control = solve_mpc_problem()
        
        solve_time = time.time() - start
        
        # Record metrics
        self.metrics.record_mpc_step(
            reference=self.reference_state,
            actual=self.current_state,
            control=control,
            tracking_error=self.current_state - self.reference_state,
            solve_time=solve_time,
            constraint_violated=False
        )
        
        return control
    
    def get_metrics_plot(self):
        """Generate metrics plot."""
        from quadrotor_gp_mpc.performance_metrics_utils import QuickMetricsPlotter
        QuickMetricsPlotter.plot_all(mpc_metrics=self.metrics.get_metrics())
    '''
    print(code_example)


if __name__ == "__main__":
    print("\nPerformance Metrics Utilities")
    print("=" * 60)
    print("\nUsage patterns:\n")
    
    example_gp_node_integration()
    example_mpc_node_integration()
    
    print("\n" + "=" * 60)
    print("See GP_MPC_METRICS_README.md for full integration guide")
