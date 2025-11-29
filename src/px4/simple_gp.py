#!/usr/bin/env python3
"""
Simple Gaussian Process for MPC Enhancement
===========================================
Lightweight GP implementation without TensorFlow dependencies.
Uses scikit-learn's Gaussian Process Regressor for learning quadrotor dynamics.
"""

import numpy as np
from collections import deque
import time
import os

# from build.px4_offboard.build.lib.px4_offboard import gp

try:
    from sklearn.gaussian_process import GaussianProcessRegressor
    from sklearn.gaussian_process.kernels import RBF, WhiteKernel
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


class SimpleQuadrotorGP:
    """Simple GP for learning quadrotor dynamics residuals"""
    
    def __init__(self, max_data_points=1000):
        self.max_data_points = max_data_points
        
        # Training data storage
        self.X_train = deque(maxlen=max_data_points)
        self.Y_train = deque(maxlen=max_data_points)
        
        # GP model
        self.gp_model = None
        self.is_trained = False
        
        # Statistics
        self.training_count = 0
        self.prediction_count = 0
    
    def _to_numpy(self):
        """Return X, Y as numpy arrays."""
        if len(self.X_train) == 0:
            return np.empty((0, 10)), np.empty((0, 6))
        X = np.array(self.X_train)  # shape (N, 10)
        Y = np.array(self.Y_train)  # shape (N, 6)
        return X, Y
    
    def load_model(self, model_path):
        """Load pre-trained GP model from file"""
        try:
            if not os.path.exists(model_path):
                print(f"❌ Model file not found: {model_path}")
                return False
                
            import pickle
            with open(model_path, 'rb') as f:
                model_data = pickle.load(f)
            
            self.gp_model = model_data['gp_model']
            self.is_trained = True
            self.training_count = model_data.get('training_count', 0)
            
            print(f"📁 GP model loaded successfully!")
            print(f"   - Model file: {model_path}")
            print(f"   - Training samples: {model_data.get('data_points_used', 'unknown')}")
            print(f"   - Created: {model_data.get('timestamp', 'unknown')}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to load model: {e}")
            return False
    
    def save_dataset(self, csv_path, include_header=True, overwrite=True):
        """
        Save current training dataset (X_train, Y_train) to a CSV file.

        Columns:
        x,y,z,vx,vy,vz,ax,ay,az,yaw_rate,
        res_dx,res_dy,res_dz,res_dvx,res_dvy,res_dvz
        """
        X, Y = self._to_numpy()
        if X.shape[0] == 0:
            print("⚠️ No training data to save.")
            return

        data = np.hstack([X, Y])  # shape (N, 16)

        # Make sure directory exists
        os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)

        header = (
            "x,y,z,"
            "vx,vy,vz,"
            "ax,ay,az,yaw_rate,"
            "res_dx,res_dy,res_dz,"
            "res_dvx,res_dvy,res_dvz"
        )

        if overwrite or not os.path.exists(csv_path):
            # Write new file (with optional header)
            np.savetxt(
                csv_path,
                data,
                delimiter=",",
                header=header if include_header else "",
                comments="",
            )
            print(f"✅ GP dataset saved to {csv_path} (N={data.shape[0]})")
        else:
            # Append without header
            with open(csv_path, "ab") as f:
                np.savetxt(f, data, delimiter=",")
            print(f"✅ GP dataset appended to {csv_path} (N+={data.shape[0]})")

        
    def add_training_data(self, state, control, state_next, dt=0.02):
        """Add training data for GP with quality filtering"""
        if len(state) < 6 or len(state_next) < 6:
            return
            
        # Quality filter: Only train on stable, low-error data
        velocity_norm = np.linalg.norm(state[3:6])
        control_norm = np.linalg.norm(control[:3])  # acceleration commands
        
        # Skip training data if system is too aggressive or unstable
        if velocity_norm > 5.0 or control_norm > 3.0:  # Conservative thresholds
            return
            
        x_input = np.concatenate([state[:6], control[:4]])
        state_nominal_next = self._nominal_dynamics(state, control, dt)
        residual = state_next - state_nominal_next
        
        # Skip if residual is too large (likely noise or model failure)
        if np.linalg.norm(residual) > 2.0:
            return
            
        self.X_train.append(x_input.copy())
        self.Y_train.append(residual.copy())
        
        # # Train less frequently and require more data
        #if len(self.X_train) % 500 == 0 and len(self.X_train) >= 500:
        #     self.train_gp()

    
    def _nominal_dynamics(self, state, control, dt):
        """Nominal double integrator dynamics"""
        x, y, z, vx, vy, vz = state
        ax, ay, az, yaw_rate = control
        
        # ẋ = [vx, vy, vz, ax, ay, az]
        state_dot = np.array([vx, vy, vz, ax, ay, az])
        return state + dt * state_dot
    
    def train_gp(self):
        """Train GP model"""
        if not SKLEARN_AVAILABLE or len(self.X_train) < 30:
            return
            
        try:
            # Convert to arrays
            X = np.array(list(self.X_train))
            Y = np.array(list(self.Y_train))
            
            # Simple RBF kernel with better parameters
            kernel = RBF(length_scale=0.5) + WhiteKernel(noise_level=0.1)
            
            # Create and train GP
            self.gp_model = GaussianProcessRegressor(
                kernel=kernel,
                alpha=1e-4,  # Less regularization for more confident predictions
                normalize_y=True,
                n_restarts_optimizer=1  # Faster training
            )
            
            self.gp_model.fit(X, Y)
            self.is_trained = True
            self.training_count += 1
            
            print(f"✅ Simple GP trained with {len(X)} samples (iteration {self.training_count})")
            
        except Exception as e:
            print(f"⚠️ GP training failed: {e}")
            self.is_trained = False
    
    def predict_residual(self, state, control):
        """Predict dynamics residual"""
        if not self.is_trained or not SKLEARN_AVAILABLE:
            return np.zeros(6), np.ones(6)  # Zero residual, high uncertainty
            
        try:
            x_input = np.concatenate([state, control]).reshape(1, -1)
            mean, std = self.gp_model.predict(x_input, return_std=True)
            
            self.prediction_count += 1
            return mean.flatten(), std.flatten() ** 2  # Return variance
            
        except Exception as e:
            print(f"⚠️ GP prediction failed: {e}")
            return np.zeros(6), np.ones(6)
    
    def get_uncertainty(self, state, control):
        """Get prediction uncertainty"""
        _, variance = self.predict_residual(state, control)
        uncertainty = np.mean(np.sqrt(variance))
        return uncertainty
    
    def predict_enhanced_dynamics(self, state, control, dt):
        """Predict enhanced dynamics: nominal + GP residual"""
        state_nominal = self._nominal_dynamics(state, control, dt)
        residual_mean, _ = self.predict_residual(state, control)
        return state_nominal + residual_mean
    
    def get_stats(self):
        """Get statistics"""
        return {
            'is_trained': self.is_trained,
            'data_points': len(self.X_train),
            'training_iterations': self.training_count,
            'predictions_made': self.prediction_count,
            'sklearn_available': SKLEARN_AVAILABLE
        }


class SimpleGPEnhancedMPC:
    """Simple GP-enhanced MPC wrapper"""
    
    def __init__(self, gp_model, confidence_threshold=0.5):
        self.gp_model = gp_model
        self.confidence_threshold = confidence_threshold
        self.gp_usage = 0
        self.nominal_usage = 0
    
    def enhanced_dynamics_function(self, state, control, dt):
        """Enhanced dynamics with GP correction"""
        if not self.gp_model.is_trained:
            self.nominal_usage += 1
            return self.gp_model._nominal_dynamics(state, control, dt)
            
        uncertainty = self.gp_model.get_uncertainty(state, control)
        
        # Debug: Print uncertainty values occasionally
        if (self.gp_usage + self.nominal_usage) % 50 == 0:
            print(f"🔍 GP uncertainty: {uncertainty:.4f} vs threshold: {self.confidence_threshold}")
        
        # Use GP only when uncertainty is low (confident predictions)
        if uncertainty < self.confidence_threshold:
            self.gp_usage += 1
            return self.gp_model.predict_enhanced_dynamics(state, control, dt)
        else:
            self.nominal_usage += 1
            return self.gp_model._nominal_dynamics(state, control, dt)
    
    def get_usage_ratio(self):
        """Get GP vs nominal usage ratio"""
        total = self.gp_usage + self.nominal_usage
        if total == 0:
            return 0.0
        return self.gp_usage / total * 100