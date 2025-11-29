#!/usr/bin/env python3
"""
Offline GP Trainer
=================
Trains Gaussian Process models from collected flight data.
"""

import numpy as np
import pickle
import os
import glob
import time
from typing import List, Dict, Any, Tuple
from datetime import datetime

try:
    from sklearn.gaussian_process import GaussianProcessRegressor
    from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel
    from sklearn.preprocessing import StandardScaler
    from sklearn.model_selection import train_test_split
    from sklearn.metrics import mean_squared_error, r2_score
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False
    print("⚠️ Scikit-learn not available. Install with: pip install scikit-learn")

class GPTrainer:
    """Offline GP trainer for quadrotor dynamics"""
    
    def __init__(self, data_dir: str = "data/training_data", 
                 model_dir: str = "data/trained_models"):
        if not SKLEARN_AVAILABLE:
            raise ImportError("Scikit-learn is required for GP training")
            
        self.data_dir = data_dir
        self.model_dir = model_dir
        
        # Create model directory
        os.makedirs(model_dir, exist_ok=True)
        
        # GP models for each state dimension
        self.gp_models = {}
        self.scalers_X = {}
        self.scalers_y = {}
        
        # Training statistics
        self.training_stats = {}
        
    def load_training_data(self, max_samples: int = None) -> Tuple[np.ndarray, np.ndarray]:
        """Load all training data from files"""
        print("🔍 Loading training data...")
        
        # Find all data files
        data_files = glob.glob(os.path.join(self.data_dir, "flight_data_*.npz"))
        
        if not data_files:
            raise FileNotFoundError(f"No training data found in {self.data_dir}")
        
        all_X = []
        all_y = []
        
        for data_file in data_files:
            print(f"📂 Loading {os.path.basename(data_file)}")
            data = np.load(data_file)
            
            states_prev = data['states_prev']
            controls = data['controls']
            states_next = data['states_next']
            dt_values = data['dt_values']
            
            # Calculate residuals (actual change - nominal change)
            for i in range(len(states_prev)):
                state_prev = states_prev[i]
                control = controls[i]
                state_next = states_next[i]
                dt = dt_values[i]
                
                # Nominal dynamics (double integrator)
                nominal_next = self._nominal_dynamics(state_prev, control, dt)
                
                # Residual = actual_next - nominal_next
                residual = state_next - nominal_next
                
                # Input: [state_prev, control]
                X_sample = np.concatenate([state_prev, control])
                
                # Output: residual for each state dimension
                all_X.append(X_sample)
                all_y.append(residual)
        
        X = np.array(all_X)
        y = np.array(all_y)
        
        # Limit samples if requested
        if max_samples and len(X) > max_samples:
            indices = np.random.choice(len(X), max_samples, replace=False)
            X = X[indices]
            y = y[indices]
        
        print(f"✅ Loaded {len(X)} training samples")
        print(f"   Input dimension: {X.shape[1]} (6 states + 4 controls)")
        print(f"   Output dimension: {y.shape[1]} (6 residuals)")
        
        return X, y
    
    def _nominal_dynamics(self, state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """Nominal double integrator dynamics"""
        # state: [x, y, z, vx, vy, vz]
        # control: [ax, ay, az, yaw_rate]
        
        pos = state[:3]
        vel = state[3:6]
        accel = control[:3]
        
        # Double integrator: pos_next = pos + vel*dt, vel_next = vel + accel*dt
        pos_next = pos + vel * dt
        vel_next = vel + accel * dt
        
        return np.concatenate([pos_next, vel_next])
    
    def train_gp_models(self, X: np.ndarray, y: np.ndarray, 
                       test_size: float = 0.2) -> Dict[str, Any]:
        """Train GP models for each output dimension"""
        print("🧠 Training GP models...")
        
        # Split data
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=test_size, random_state=42
        )
        
        print(f"   Training samples: {len(X_train)}")
        print(f"   Test samples: {len(X_test)}")
        
        # Train a GP for each output dimension
        output_names = ['x_residual', 'y_residual', 'z_residual', 
                       'vx_residual', 'vy_residual', 'vz_residual']
        
        training_results = {}
        
        for i, output_name in enumerate(output_names):
            print(f"   Training GP for {output_name}...")
            
            # Extract target values for this dimension
            y_train_i = y_train[:, i]
            y_test_i = y_test[:, i]
            
            # Skip if no significant variation
            if np.std(y_train_i) < 1e-6:
                print(f"     Skipping {output_name} - no significant variation")
                continue
            
            # Scale input and output data
            scaler_X = StandardScaler()
            scaler_y = StandardScaler()
            
            X_train_scaled = scaler_X.fit_transform(X_train)
            y_train_scaled = scaler_y.fit_transform(y_train_i.reshape(-1, 1)).flatten()
            
            X_test_scaled = scaler_X.transform(X_test)
            y_test_scaled = scaler_y.transform(y_test_i.reshape(-1, 1)).flatten()
            
            # Define GP kernel
            kernel = (ConstantKernel(1.0, constant_value_bounds="fixed") * 
                     RBF(length_scale=[1.0] * X.shape[1], 
                         length_scale_bounds=(0.1, 10.0)) + 
                     WhiteKernel(noise_level=0.01, noise_level_bounds=(1e-5, 1e1)))
            
            # Create and train GP
            gp = GaussianProcessRegressor(
                kernel=kernel,
                n_restarts_optimizer=3,
                alpha=1e-6,
                normalize_y=False  # We're doing our own normalization
            )
            
            gp.fit(X_train_scaled, y_train_scaled)
            
            # Evaluate on test set
            y_pred_scaled = gp.predict(X_test_scaled)
            y_pred = scaler_y.inverse_transform(y_pred_scaled.reshape(-1, 1)).flatten()
            
            # Calculate metrics
            mse = mean_squared_error(y_test_i, y_pred)
            r2 = r2_score(y_test_i, y_pred)
            
            # Store model and scalers
            self.gp_models[output_name] = gp
            self.scalers_X[output_name] = scaler_X
            self.scalers_y[output_name] = scaler_y
            
            # Store statistics
            training_results[output_name] = {
                'mse': mse,
                'rmse': np.sqrt(mse),
                'r2': r2,
                'kernel': str(gp.kernel_),
                'log_marginal_likelihood': gp.log_marginal_likelihood()
            }
            
            print(f"     RMSE: {np.sqrt(mse):.6f}, R²: {r2:.4f}")
        
        self.training_stats = training_results
        print("✅ GP training completed!")
        
        return training_results
    
    def save_models(self, model_name: str = None) -> str:
        """Save trained GP models"""
        if model_name is None:
            model_name = f"gp_model_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        model_path = os.path.join(self.model_dir, f"{model_name}.pkl")
        
        model_data = {
            'gp_models': self.gp_models,
            'scalers_X': self.scalers_X,
            'scalers_y': self.scalers_y,
            'training_stats': self.training_stats,
            'model_name': model_name,
            'creation_time': time.time()
        }
        
        with open(model_path, 'wb') as f:
            pickle.dump(model_data, f)
        
        print(f"✅ Saved GP models to {model_path}")
        return model_path
    
    def load_models(self, model_path: str):
        """Load trained GP models"""
        with open(model_path, 'rb') as f:
            model_data = pickle.load(f)
        
        self.gp_models = model_data['gp_models']
        self.scalers_X = model_data['scalers_X']
        self.scalers_y = model_data['scalers_y']
        self.training_stats = model_data['training_stats']
        
        print(f"✅ Loaded GP models from {model_path}")
        return model_data['training_stats']