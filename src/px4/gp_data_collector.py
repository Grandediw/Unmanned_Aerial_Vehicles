#!/usr/bin/env python3
"""
GP Data Collector for Offline Training
=====================================
Collects flight data during normal operations for offline GP training.
"""

import numpy as np
import pickle
import os
import time
from datetime import datetime
from typing import List, Dict, Any

class GPDataCollector:
    """Collects and manages flight data for GP training"""
    
    def __init__(self, max_samples: int = 10000, data_dir: str = "data/training_data"):
        self.max_samples = max_samples
        self.data_dir = data_dir
        
        # Create data directory if it doesn't exist
        os.makedirs(data_dir, exist_ok=True)
        
        # Data storage
        self.states_prev = []      # Previous states [x,y,z,vx,vy,vz]
        self.controls = []         # Control inputs [ax,ay,az,yaw_rate]
        self.states_next = []      # Next states [x,y,z,vx,vy,vz]
        self.timestamps = []       # Timestamps
        self.dt_values = []        # Time steps
        
        # Metadata
        self.flight_session = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.collection_active = True
        
    def add_sample(self, state_prev: np.ndarray, control: np.ndarray, 
                   state_next: np.ndarray, dt: float):
        """Add a training sample"""
        if not self.collection_active or len(self.states_prev) >= self.max_samples:
            return False
            
        # Extract 6D states (position + velocity)
        state_6d_prev = state_prev[:6] if len(state_prev) >= 6 else state_prev
        state_6d_next = state_next[:6] if len(state_next) >= 6 else state_next
        control_4d = control[:4] if len(control) >= 4 else control
        
        # Store data
        self.states_prev.append(state_6d_prev.copy())
        self.controls.append(control_4d.copy())
        self.states_next.append(state_6d_next.copy())
        self.timestamps.append(time.time())
        self.dt_values.append(dt)
        
        return True
    
    def save_data(self, filename: str = None) -> str:
        """Save collected data to file"""
        if filename is None:
            filename = f"flight_data_{self.flight_session}.npz"
        
        filepath = os.path.join(self.data_dir, filename)
        
        # Convert lists to numpy arrays
        data = {
            'states_prev': np.array(self.states_prev),
            'controls': np.array(self.controls),
            'states_next': np.array(self.states_next),
            'timestamps': np.array(self.timestamps),
            'dt_values': np.array(self.dt_values),
            'flight_session': self.flight_session,
            'collection_time': datetime.now().isoformat()
        }
        
        np.savez_compressed(filepath, **data)
        print(f"✅ Saved {len(self.states_prev)} samples to {filepath}")
        
        return filepath
    
    def get_stats(self) -> Dict[str, Any]:
        """Get collection statistics"""
        return {
            'num_samples': len(self.states_prev),
            'max_samples': self.max_samples,
            'collection_active': self.collection_active,
            'flight_session': self.flight_session,
            'fill_percentage': (len(self.states_prev) / self.max_samples) * 100
        }
    
    def stop_collection(self):
        """Stop data collection"""
        self.collection_active = False
        
    def clear_data(self):
        """Clear collected data"""
        self.states_prev.clear()
        self.controls.clear()
        self.states_next.clear()
        self.timestamps.clear()
        self.dt_values.clear()