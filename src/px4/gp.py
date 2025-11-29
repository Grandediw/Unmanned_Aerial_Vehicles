#!/usr/bin/env python3
"""
Gaussian Process Dynamics Learning for MPC Enhancement
=====================================================
This module uses GPflow to learn residual quadrotor dynamics that improve
the simple double integrator model used in MPC. The GP learns:

Δẋ = f_GP(x, u) where the true dynamics are:
ẋ_true = f_nominal(x, u) + f_GP(x, u)

The learned GP model is then integrated into the MPC prediction model
for better trajectory tracking performance.
"""

import numpy as np
import tensorflow as tf
import gpflow
from gpflow.utilities import print_summary
import pickle
import os
from typing import Tuple, Optional, Dict, List
import time
from collections import deque

class QuadrotorGPDynamics:
    """
    Gaussian Process model for learning quadrotor dynamics residuals
    """
    
    def __init__(self, 
                 state_dim: int = 6, 
                 control_dim: int = 4,
                 output_dim: int = 6,
                 lengthscales: Optional[List[float]] = None,
                 variance: float = 1.0,
                 noise_variance: float = 0.01,
                 max_data_points: int = 2000):
        """
        Initialize GP dynamics model
        
        Args:
            state_dim: Dimension of state vector [x,y,z,vx,vy,vz]
            control_dim: Dimension of control vector [ax,ay,az,yaw_rate] 
            output_dim: Dimension of output (state derivatives)
            lengthscales: GP lengthscales for each input dimension
            variance: GP signal variance
            noise_variance: GP noise variance
            max_data_points: Maximum training data points to keep
        """
        self.state_dim = state_dim
        self.control_dim = control_dim
        self.output_dim = output_dim
        self.input_dim = state_dim + control_dim  # [state, control]
        self.max_data_points = max_data_points
        
        # Training data storage
        self.X_train = deque(maxlen=max_data_points)
        self.Y_train = deque(maxlen=max_data_points)
        
        # GP hyperparameters
        if lengthscales is None:
            # Default lengthscales: shorter for position/velocity, longer for accelerations
            lengthscales = [2.0, 2.0, 1.0,      # position [x,y,z]
                          1.0, 1.0, 0.5,        # velocity [vx,vy,vz]
                          0.5, 0.5, 0.3, 2.0]   # control [ax,ay,az,yaw_rate]
        
        self.lengthscales = lengthscales
        self.variance = variance
        self.noise_variance = noise_variance
        
        # GP model (will be created when first training data arrives)
        self.gp_model = None
        self.is_trained = False
        
        # Normalization parameters
        self.input_mean = None
        self.input_std = None
        self.output_mean = None
        self.output_std = None
        
        # Performance tracking
        self.training_history = []
        self.prediction_times = deque(maxlen=100)
        
        print("🧠 GP Dynamics Learner Initialized")
        print(f"   📊 Input dim: {self.input_dim} (state: {state_dim}, control: {control_dim})")
        print(f"   📈 Output dim: {output_dim}")
        print(f"   💾 Max data points: {max_data_points}")
        
    def add_training_data(self, state: np.ndarray, control: np.ndarray, 
                         state_next: np.ndarray, dt: float):
        """
        Add new training data point
        
        Args:
            state: Current state [x,y,z,vx,vy,vz]
            control: Applied control [ax,ay,az,yaw_rate]  
            state_next: Next observed state
            dt: Time step
        """
        # Input: [state, control]
        x_input = np.concatenate([state, control])
        
        # Output: residual dynamics (observed - nominal)
        # Nominal dynamics: double integrator
        state_nominal_next = self._nominal_dynamics(state, control, dt)
        residual = state_next - state_nominal_next
        
        # Store training data
        self.X_train.append(x_input.copy())
        self.Y_train.append(residual.copy())
        
        # Retrain periodically
        if len(self.X_train) % 50 == 0 and len(self.X_train) >= 100:
            self.train_gp()
            
    def _nominal_dynamics(self, state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """
        Nominal double integrator dynamics
        """
        x, y, z, vx, vy, vz = state
        ax, ay, az, yaw_rate = control
        
        # Double integrator: ẋ = [vx, vy, vz, ax, ay, az]
        state_dot = np.array([vx, vy, vz, ax, ay, az])
        
        # Euler integration
        state_next = state + dt * state_dot
        
        return state_next
        
    def train_gp(self):
        """
        Train GP model on collected data
        """
        if len(self.X_train) < 20:
            print("⚠️  Not enough data for GP training")
            return
            
        print(f"🎓 Training GP with {len(self.X_train)} data points...")
        
        # Convert to arrays
        X_array = np.array(list(self.X_train))
        Y_array = np.array(list(self.Y_train))
        
        # Normalize inputs and outputs
        self.input_mean = np.mean(X_array, axis=0)
        self.input_std = np.std(X_array, axis=0) + 1e-8  # Avoid division by zero
        
        self.output_mean = np.mean(Y_array, axis=0) 
        self.output_std = np.std(Y_array, axis=0) + 1e-8
        
        X_norm = (X_array - self.input_mean) / self.input_std
        Y_norm = (Y_array - self.output_mean) / self.output_std
        
        # Create GP model
        try:
            # Multi-output GP using independent GPs for each output dimension
            kernels = []
            for i in range(self.output_dim):
                kernel = gpflow.kernels.RBF(
                    lengthscales=self.lengthscales,
                    variance=self.variance
                )
                kernels.append(kernel)
            
            # Combine kernels for multi-output
            if self.output_dim > 1:
                kernel = gpflow.kernels.SeparateIndependent(kernels)
            else:
                kernel = kernels[0]
            
            # Create model
            self.gp_model = gpflow.models.GPR(
                data=(X_norm, Y_norm),
                kernel=kernel,
                noise_variance=self.noise_variance
            )
            
            # Optimize hyperparameters
            opt = gpflow.optimizers.Scipy()
            opt.minimize(
                self.gp_model.training_loss,
                self.gp_model.trainable_variables,
                options=dict(maxiter=100)
            )
            
            self.is_trained = True
            
            # Log performance
            log_likelihood = -self.gp_model.training_loss().numpy()
            self.training_history.append({
                'timestamp': time.time(),
                'data_points': len(self.X_train),
                'log_likelihood': log_likelihood
            })
            
            print(f"✅ GP training complete!")
            print(f"   📊 Log-likelihood: {log_likelihood:.3f}")
            print(f"   🎯 Kernel lengthscales: {[f'{ls:.3f}' for ls in kernel.lengthscales.numpy().flatten()]}")
            
        except Exception as e:
            print(f"❌ GP training failed: {e}")
            self.is_trained = False
            
    def predict_residual(self, state: np.ndarray, control: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict dynamics residual using trained GP
        
        Returns:
            mean: Predicted residual mean
            variance: Predicted residual variance
        """
        if not self.is_trained or self.gp_model is None:
            # Return zero residual if not trained
            return np.zeros(self.output_dim), np.zeros(self.output_dim)
            
        # Prepare input
        x_input = np.concatenate([state, control]).reshape(1, -1)
        
        # Normalize
        x_norm = (x_input - self.input_mean) / self.input_std
        
        try:
            start_time = time.time()
            
            # GP prediction
            mean_norm, var_norm = self.gp_model.predict_f(x_norm)
            
            # Denormalize
            mean = mean_norm.numpy().flatten() * self.output_std + self.output_mean
            variance = var_norm.numpy().flatten() * (self.output_std ** 2)
            
            # Track prediction time
            pred_time = time.time() - start_time
            self.prediction_times.append(pred_time)
            
            return mean, variance
            
        except Exception as e:
            print(f"⚠️  GP prediction failed: {e}")
            return np.zeros(self.output_dim), np.zeros(self.output_dim)
            
    def predict_enhanced_dynamics(self, state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """
        Predict enhanced dynamics: nominal + GP residual
        """
        # Nominal prediction
        state_nominal = self._nominal_dynamics(state, control, dt)
        
        # GP residual prediction
        residual_mean, _ = self.predict_residual(state, control)
        
        # Enhanced prediction
        state_enhanced = state_nominal + residual_mean
        
        return state_enhanced
        
    def get_uncertainty(self, state: np.ndarray, control: np.ndarray) -> float:
        """
        Get prediction uncertainty for adaptive MPC
        """
        if not self.is_trained:
            return 1.0  # High uncertainty if not trained
            
        _, variance = self.predict_residual(state, control)
        
        # Return average uncertainty across state dimensions
        return np.mean(np.sqrt(variance))
        
    def save_model(self, filepath: str):
        """Save trained GP model and data"""
        if not self.is_trained:
            print("⚠️  No trained model to save")
            return
            
        data = {
            'X_train': list(self.X_train),
            'Y_train': list(self.Y_train),
            'input_mean': self.input_mean,
            'input_std': self.input_std,
            'output_mean': self.output_mean,
            'output_std': self.output_std,
            'training_history': self.training_history,
            'hyperparameters': {
                'lengthscales': self.lengthscales,
                'variance': self.variance,
                'noise_variance': self.noise_variance
            }
        }
        
        with open(filepath, 'wb') as f:
            pickle.dump(data, f)
            
        print(f"💾 GP model saved to {filepath}")
        
    def load_model(self, filepath: str):
        """Load trained GP model and data"""
        if not os.path.exists(filepath):
            print(f"⚠️  Model file not found: {filepath}")
            return
            
        try:
            with open(filepath, 'rb') as f:
                data = pickle.load(f)
                
            # Restore data
            self.X_train = deque(data['X_train'], maxlen=self.max_data_points)
            self.Y_train = deque(data['Y_train'], maxlen=self.max_data_points)
            self.input_mean = data['input_mean']
            self.input_std = data['input_std']  
            self.output_mean = data['output_mean']
            self.output_std = data['output_std']
            self.training_history = data['training_history']
            
            # Retrain model
            self.train_gp()
            
            print(f"✅ GP model loaded from {filepath}")
            print(f"   📊 Data points: {len(self.X_train)}")
            
        except Exception as e:
            print(f"❌ Failed to load GP model: {e}")
            
    def get_stats(self) -> Dict:
        """Get GP performance statistics"""
        stats = {
            'is_trained': self.is_trained,
            'data_points': len(self.X_train),
            'avg_prediction_time': np.mean(self.prediction_times) if self.prediction_times else 0.0,
            'training_iterations': len(self.training_history)
        }
        
        if self.training_history:
            stats['latest_log_likelihood'] = self.training_history[-1]['log_likelihood']
            
        return stats
        
    def print_summary(self):
        """Print GP model summary"""
        print("\n🧠 GP Dynamics Model Summary")
        print("=" * 40)
        
        stats = self.get_stats()
        
        print(f"Status: {'✅ Trained' if stats['is_trained'] else '❌ Not Trained'}")
        print(f"Data Points: {stats['data_points']}/{self.max_data_points}")
        print(f"Training Iterations: {stats['training_iterations']}")
        
        if stats['avg_prediction_time'] > 0:
            print(f"Avg Prediction Time: {stats['avg_prediction_time']*1000:.2f} ms")
            
        if 'latest_log_likelihood' in stats:
            print(f"Latest Log-Likelihood: {stats['latest_log_likelihood']:.3f}")
            
        if self.is_trained and self.gp_model is not None:
            print("\nGP Model Details:")
            try:
                print_summary(self.gp_model)
            except:
                print("Model summary not available")
                
        print("=" * 40)


class GPEnhancedMPC:
    """
    MPC integration wrapper for GP-enhanced dynamics
    """
    
    def __init__(self, gp_dynamics: QuadrotorGPDynamics, confidence_threshold: float = 0.1):
        """
        Args:
            gp_dynamics: Trained GP dynamics model
            confidence_threshold: Uncertainty threshold for switching between models
        """
        self.gp_dynamics = gp_dynamics
        self.confidence_threshold = confidence_threshold
        
        # Usage statistics
        self.gp_usage_count = 0
        self.nominal_usage_count = 0
        
    def enhanced_dynamics_function(self, state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """
        Enhanced dynamics function for MPC integration
        
        Uses GP prediction when confidence is high, falls back to nominal otherwise
        """
        if not self.gp_dynamics.is_trained:
            self.nominal_usage_count += 1
            return self.gp_dynamics._nominal_dynamics(state, control, dt)
            
        # Get uncertainty
        uncertainty = self.gp_dynamics.get_uncertainty(state, control)
        
        if uncertainty < self.confidence_threshold:
            # High confidence: use GP-enhanced dynamics
            self.gp_usage_count += 1
            return self.gp_dynamics.predict_enhanced_dynamics(state, control, dt)
        else:
            # Low confidence: use nominal dynamics
            self.nominal_usage_count += 1
            return self.gp_dynamics._nominal_dynamics(state, control, dt)
            
    def get_usage_stats(self) -> Dict:
        """Get statistics on GP vs nominal usage"""
        total = self.gp_usage_count + self.nominal_usage_count
        
        if total == 0:
            return {'gp_usage': 0.0, 'nominal_usage': 0.0, 'total_predictions': 0}
            
        return {
            'gp_usage': self.gp_usage_count / total * 100,
            'nominal_usage': self.nominal_usage_count / total * 100,
            'total_predictions': total,
            'gp_count': self.gp_usage_count,
            'nominal_count': self.nominal_usage_count
        }


# Example usage and testing
if __name__ == "__main__":
    print("🧠 Testing GP Dynamics Learning...")
    
    # Create GP model
    gp_model = QuadrotorGPDynamics()
    
    # Generate some synthetic training data
    print("📊 Generating synthetic training data...")
    
    for i in range(200):
        # Random state and control
        state = np.random.randn(6) * [5, 5, 2, 2, 2, 1]  # [x,y,z,vx,vy,vz]
        control = np.random.randn(4) * [2, 2, 3, 0.5]     # [ax,ay,az,yaw_rate]
        
        # Simulate "true" dynamics with some nonlinearity
        dt = 0.02
        state_next_true = gp_model._nominal_dynamics(state, control, dt)
        
        # Add some nonlinear effects (drag, coupling, etc.)
        drag_effect = -0.1 * state[3:6] * np.abs(state[3:6])  # Quadratic drag
        coupling_effect = 0.05 * np.array([
            state[4] * control[3],  # yaw-rate coupling to vx
            -state[3] * control[3], # yaw-rate coupling to vy  
            0.0                     # no coupling to vz
        ])
        
        state_next_true[3:6] += dt * (drag_effect + coupling_effect)
        
        # Add to training data
        gp_model.add_training_data(state, control, state_next_true, dt)
        
    # Print final summary
    gp_model.print_summary()
    
    # Test prediction
    print("\n🔮 Testing GP predictions...")
    test_state = np.array([1.0, 2.0, 3.0, 0.5, -0.3, 0.1])
    test_control = np.array([0.2, -0.1, 1.0, 0.05])
    
    residual_mean, residual_var = gp_model.predict_residual(test_state, test_control)
    print(f"Predicted residual: {residual_mean}")
    print(f"Prediction variance: {residual_var}")
    
    enhanced_state = gp_model.predict_enhanced_dynamics(test_state, test_control, 0.02)
    print(f"Enhanced next state: {enhanced_state}")
    
    print("\n✅ GP Dynamics Learning test complete!")