#!/usr/bin/env python3
"""
Pre-trained GP Model Loader
===========================
Loads and uses offline-trained GP models.
"""

import numpy as np
import pickle
import os
from typing import Tuple, Dict, Any

class PreTrainedGP:
    """Wrapper for pre-trained GP models"""
    
    def __init__(self, model_path: str):
        self.model_path = model_path
        self.gp_models = {}
        self.scalers_X = {}
        self.scalers_y = {}
        self.training_stats = {}
        self.is_loaded = False
        
        # Load models
        self.load_models()
    
    def load_models(self):
        """Load pre-trained GP models"""
        if not os.path.exists(self.model_path):
            print(f"❌ GP model file not found: {self.model_path}")
            return False
        
        try:
            with open(self.model_path, 'rb') as f:
                model_data = pickle.load(f)
            
            self.gp_models = model_data['gp_models']
            self.scalers_X = model_data['scalers_X']
            self.scalers_y = model_data['scalers_y']
            self.training_stats = model_data['training_stats']
            
            self.is_loaded = True
            print(f"✅ Loaded pre-trained GP models from {self.model_path}")
            print(f"   Available models: {list(self.gp_models.keys())}")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to load GP models: {e}")
            return False
    
    def predict_residual(self, state: np.ndarray, control: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Predict dynamics residual using trained GPs"""
        if not self.is_loaded:
            return np.zeros(6), np.ones(6) * 1e6
        
        # Prepare input
        state_6d = state[:6] if len(state) >= 6 else state
        control_4d = control[:4] if len(control) >= 4 else control
        X_input = np.concatenate([state_6d, control_4d]).reshape(1, -1)
        
        residual_mean = np.zeros(6)
        residual_std = np.zeros(6)
        
        output_names = ['x_residual', 'y_residual', 'z_residual',
                       'vx_residual', 'vy_residual', 'vz_residual']
        
        for i, output_name in enumerate(output_names):
            if output_name in self.gp_models:
                try:
                    # Scale input
                    X_scaled = self.scalers_X[output_name].transform(X_input)
                    
                    # Predict
                    y_pred_scaled, y_std_scaled = self.gp_models[output_name].predict(
                        X_scaled, return_std=True
                    )
                    
                    # Unscale output
                    y_pred = self.scalers_y[output_name].inverse_transform(
                        y_pred_scaled.reshape(-1, 1)
                    ).flatten()[0]
                    
                    # Approximate unscaled standard deviation
                    y_std = y_std_scaled[0] * self.scalers_y[output_name].scale_[0]
                    
                    residual_mean[i] = y_pred
                    residual_std[i] = abs(y_std)
                    
                except Exception as e:
                    print(f"⚠️ GP prediction failed for {output_name}: {e}")
                    residual_mean[i] = 0.0
                    residual_std[i] = 1e6
            else:
                residual_mean[i] = 0.0
                residual_std[i] = 1e6
        
        return residual_mean, residual_std
    
    def get_uncertainty(self, state: np.ndarray, control: np.ndarray) -> float:
        """Get average prediction uncertainty"""
        _, residual_std = self.predict_residual(state, control)
        return float(np.mean(residual_std))
    
    def get_stats(self) -> Dict[str, Any]:
        """Get model statistics"""
        return {
            'is_loaded': self.is_loaded,
            'model_path': self.model_path,
            'available_models': list(self.gp_models.keys()),
            'training_stats': self.training_stats
        }