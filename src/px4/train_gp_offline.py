#!/usr/bin/env python3
"""
Offline GP Training from Flight Data
===================================
Train GP models from CSV data collected during PX4 simulation flights.
"""

import os
import glob
import argparse
import sys
import pandas as pd
import numpy as np
from datetime import datetime

# Add package to path
sys.path.append('/home/grandediw/ros2_px4_offboard_example_ws/src/ROS2_PX4_Offboard_Example/px4_offboard')

from px4_offboard.simple_gp import SimpleQuadrotorGP


def load_csv_data_simple(gp, csv_file_path):
    """Load CSV data using the existing simple_gp format"""
    try:
        df = pd.read_csv(csv_file_path)
        print(f"📊 CSV columns found: {list(df.columns)}")
        
        # Expected format from your simple_gp.save_dataset():
        # x,y,z,vx,vy,vz,ax,ay,az,yaw_rate,res_dx,res_dy,res_dz,res_dvx,res_dvy,res_dvz
        
        required_cols = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'yaw_rate', 
                        'res_dx', 'res_dy', 'res_dz', 'res_dvx', 'res_dvy', 'res_dvz']
        
        # Check if all required columns exist
        missing_cols = [col for col in required_cols if col not in df.columns]
        if missing_cols:
            print(f"❌ Missing columns: {missing_cols}")
            return 0
        
        print(f"📊 Loading {len(df)} samples from CSV...")
        
        samples_loaded = 0
        for _, row in df.iterrows():
            try:
                # Input: [x, y, z, vx, vy, vz, ax, ay, az, yaw_rate] (10D)
                x_input = np.array([
                    row['x'], row['y'], row['z'], 
                    row['vx'], row['vy'], row['vz'],
                    row['ax'], row['ay'], row['az'], 
                    row['yaw_rate']
                ])
                
                # Output: residual [res_dx, res_dy, res_dz, res_dvx, res_dvy, res_dvz] (6D)
                y_output = np.array([
                    row['res_dx'], row['res_dy'], row['res_dz'],
                    row['res_dvx'], row['res_dvy'], row['res_dvz']
                ])
                
                # Filter out NaN/inf values
                if np.isfinite(x_input).all() and np.isfinite(y_output).all():
                    # Check if residual is reasonable (not too large)
                    if np.linalg.norm(y_output) < 5.0:  # Reasonable residual threshold
                        gp.X_train.append(x_input)
                        gp.Y_train.append(y_output)
                        samples_loaded += 1
                        
            except Exception as e:
                print(f"⚠️ Error processing row: {e}")
                continue
        
        print(f"✅ Successfully loaded {samples_loaded} valid samples")
        return samples_loaded
        
    except Exception as e:
        print(f"❌ Failed to load CSV: {e}")
        return 0


def main():
    parser = argparse.ArgumentParser(description='Train GP from collected flight data')
    parser.add_argument('--data_dir', type=str, default='/home/grandediw/ros2_px4_offboard_example_ws/gp_datasets',
                       help='Directory with CSV flight data')
    parser.add_argument('--output_dir', type=str, default='/home/grandediw/ros2_px4_offboard_example_ws/gp_models',
                       help='Directory to save trained models')
    parser.add_argument('--model_name', type=str, default=None,
                       help='Custom model name')
    parser.add_argument('--pattern', type=str, default='*.csv',
                       help='CSV file pattern to match')
    
    args = parser.parse_args()
    
    # Find flight data files
    data_dir = os.path.expanduser(args.data_dir)
    csv_pattern = os.path.join(data_dir, args.pattern)
    csv_files = glob.glob(csv_pattern)
    
    if not csv_files:
        print(f"❌ No flight data found in {data_dir}")
        print(f"   Pattern searched: {args.pattern}")
        print(f"   Full path: {csv_pattern}")
        
        # Show what files exist
        all_files = os.listdir(data_dir) if os.path.exists(data_dir) else []
        if all_files:
            print(f"   Files found in directory:")
            for f in all_files:
                print(f"     - {f}")
        else:
            print(f"   Directory does not exist: {data_dir}")
        return 1
    
    print(f"🔍 Found {len(csv_files)} flight data files in {data_dir}:")
    total_size = 0
    for csv_file in sorted(csv_files):
        file_size = os.path.getsize(csv_file) / 1024  # KB
        total_size += file_size
        print(f"   - {os.path.basename(csv_file)} ({file_size:.1f} KB)")
    print(f"   Total data size: {total_size:.1f} KB")
    
    print(f"\n🎓 Starting offline GP training...")
    
    # Initialize GP for training using your existing constructor
    print("🔧 Initializing SimpleQuadrotorGP...")
    gp = SimpleQuadrotorGP(max_data_points=10000)  # Use your existing constructor
    
    # Load data from all CSV files
    total_samples_loaded = 0
    successful_files = 0
    
    for csv_file in csv_files:
        print(f"\n📊 Loading data from: {os.path.basename(csv_file)}")
        samples_loaded = load_csv_data_simple(gp, csv_file)
        
        if samples_loaded > 0:
            total_samples_loaded += samples_loaded
            successful_files += 1
            print(f"   ✅ Loaded {samples_loaded} samples")
        else:
            print(f"   ❌ Failed to load {csv_file}")
    
    if total_samples_loaded == 0:
        print(f"❌ No training data could be loaded from any CSV file!")
        print(f"   Check that CSV files have the correct format")
        return 1
    
    print(f"\n📦 Successfully loaded {total_samples_loaded} samples from {successful_files}/{len(csv_files)} files")
    print(f"📊 Final GP training data size: {len(gp.X_train)} samples")
    
    # Train the GP model
    print(f"\n🧠 Training Gaussian Process...")
    
    # Call train_gp() - your method doesn't return True/False, so we check is_trained instead
    gp.train_gp()
    
    # Check if training was successful by looking at is_trained flag
    if gp.is_trained and gp.gp_model is not None:
        # Save the trained model
        output_dir = os.path.expanduser(args.output_dir)
        os.makedirs(output_dir, exist_ok=True)
        
        if args.model_name:
            model_path = os.path.join(output_dir, f"{args.model_name}.pkl")
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            model_path = os.path.join(output_dir, f"gp_model_{timestamp}.pkl")
        
        # Try using your existing save method first, fall back to manual save
        try:
            # Check if your GP has a save_model method
            if hasattr(gp, 'save_model'):
                saved_path = gp.save_model(model_path)
                if saved_path:
                    print(f"\n✅ GP training completed successfully!")
                    print(f"📁 Model saved using GP save_model(): {saved_path}")
                    model_path = saved_path
                else:
                    raise Exception("GP save_model() returned False/None")
            else:
                raise Exception("No save_model() method found")
                
        except Exception as save_error:
            print(f"⚠️ GP save_model() failed: {save_error}")
            print(f"🔄 Trying manual pickle save...")
            
            # Manual save as fallback
            try:
                import pickle
                model_data = {
                    'gp_model': gp.gp_model,
                    'training_count': gp.training_count,
                    'data_points_used': len(gp.X_train),
                    'timestamp': datetime.now().isoformat(),
                    'is_trained': gp.is_trained
                }
                
                with open(model_path, 'wb') as f:
                    pickle.dump(model_data, f)
                
                print(f"✅ Manual pickle save successful!")
                print(f"📁 Model saved: {model_path}")
                
            except Exception as pickle_error:
                print(f"❌ Manual pickle save failed: {pickle_error}")
                return 1
        
        # Create latest symlink
        try:
            latest_path = os.path.join(output_dir, "gp_model_latest.pkl")
            if os.path.exists(latest_path):
                os.remove(latest_path)
            os.symlink(os.path.basename(model_path), latest_path)
            print(f"🔗 Latest model symlink created: {latest_path}")
        except Exception as symlink_error:
            print(f"⚠️ Could not create symlink: {symlink_error}")
        
        # Print training statistics
        stats = gp.get_stats()
        print(f"\n📈 Training Statistics:")
        print(f"   - Training samples: {stats['data_points']}")
        print(f"   - Training iterations: {stats['training_iterations']}")
        print(f"   - sklearn available: {stats.get('sklearn_available', 'unknown')}")
        print(f"   - Model trained: {gp.is_trained}")
        print(f"   - Model file size: {os.path.getsize(model_path) / 1024:.1f} KB")
        
        # Test a quick prediction to verify model works
        try:
            test_state = np.array([0, 0, -3, 0, 0, 0])  # hover state
            test_control = np.array([0, 0, 0, 0])       # zero control
            residual, variance = gp.predict_residual(test_state, test_control)
            print(f"   - Test prediction successful: residual_norm={np.linalg.norm(residual):.4f}")
        except Exception as test_error:
            print(f"   ⚠️ Test prediction failed: {test_error}")
        
        print(f"\n🚀 Next steps:")
        print(f"   1. Test model: python3 test_gp_model.py {model_path}")
        print(f"   2. Run GP-enhanced MPC with GP_MODEL_PATH={model_path}")
        print(f"   3. Or export GP_MODEL_PATH={model_path}")
        
        return 0
        
    else:
        print(f"❌ GP training failed!")
        print(f"   - is_trained: {gp.is_trained}")
        print(f"   - gp_model exists: {gp.gp_model is not None}")
        print(f"   - Training samples: {len(gp.X_train)}")
        print(f"   - sklearn available: {getattr(gp, 'SKLEARN_AVAILABLE', 'unknown')}")
        return 1


if __name__ == '__main__':
    sys.exit(main())