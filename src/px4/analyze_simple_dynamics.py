#!/usr/bin/env python3
"""
Analyze Simple Dynamics Data
============================
Process the data collected by simple_dynamics_collector.py to identify system dynamics
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit
import glob
import os

def load_latest_data():
    """Load the most recent dynamics data file"""
    data_files = glob.glob("/tmp/simple_dynamics_data_*.npz")
    if not data_files:
        print("❌ No data files found!")
        return None
        
    latest_file = max(data_files, key=os.path.getctime)
    print(f"📂 Loading: {latest_file}")
    
    data = np.load(latest_file)
    return {key: data[key] for key in data.keys()}

def first_order_response(t, K, tau, delay):
    """First order system response: K * (1 - exp(-(t-delay)/tau))"""
    response = np.zeros_like(t)
    mask = t >= delay
    response[mask] = K * (1 - np.exp(-(t[mask] - delay) / tau))
    return response

def second_order_response(t, K, wn, zeta, delay):
    """Second order system response"""
    response = np.zeros_like(t)
    mask = t >= delay
    
    if zeta < 1:  # Underdamped
        wd = wn * np.sqrt(1 - zeta**2)
        response[mask] = K * (1 - np.exp(-zeta * wn * (t[mask] - delay)) * 
                             (np.cos(wd * (t[mask] - delay)) + 
                              (zeta * wn / wd) * np.sin(wd * (t[mask] - delay))))
    else:  # Overdamped or critically damped
        if zeta == 1:  # Critically damped
            response[mask] = K * (1 - (1 + wn * (t[mask] - delay)) * 
                                 np.exp(-wn * (t[mask] - delay)))
        else:  # Overdamped
            r1 = -wn * (zeta - np.sqrt(zeta**2 - 1))
            r2 = -wn * (zeta + np.sqrt(zeta**2 - 1))
            A = K * r2 / (r2 - r1)
            B = K * r1 / (r1 - r2)
            response[mask] = K - A * np.exp(r1 * (t[mask] - delay)) - B * np.exp(r2 * (t[mask] - delay))
    
    return response

def analyze_step_response(time, input_signal, output_signal, axis_name):
    """Analyze step response for one axis"""
    print(f"\n🔬 Analyzing {axis_name}-axis dynamics...")
    
    # Find step responses
    diff_input = np.diff(input_signal)
    step_indices = np.where(np.abs(diff_input) > 0.1)[0]
    
    if len(step_indices) == 0:
        print(f"❌ No steps found in {axis_name}-axis")
        return None
        
    results = []
    
    for i, step_idx in enumerate(step_indices):
        # Define analysis window (3 seconds after step)
        start_idx = step_idx
        end_idx = min(step_idx + 150, len(time))  # ~3 seconds at 50Hz
        
        if end_idx - start_idx < 50:  # Need at least 1 second of data
            continue
            
        t_step = time[start_idx:end_idx] - time[start_idx]
        u_step = input_signal[start_idx:end_idx]
        y_step = output_signal[start_idx:end_idx]
        
        # Remove NaN values
        valid_mask = ~(np.isnan(u_step) | np.isnan(y_step))
        if np.sum(valid_mask) < 20:
            print(f"  ⚠️  Step {i+1}: Insufficient valid data")
            continue
            
        t_step = t_step[valid_mask]
        u_step = u_step[valid_mask]
        y_step = y_step[valid_mask]
        
        # Calculate step magnitude
        step_magnitude = np.mean(u_step[10:]) - np.mean(u_step[:5]) if len(u_step) > 15 else np.mean(u_step)
        
        if abs(step_magnitude) < 0.01:
            continue
            
        # Normalize output by input
        y_normalized = y_step / step_magnitude if step_magnitude != 0 else y_step
        
        print(f"  📊 Step {i+1}: magnitude={step_magnitude:.3f}, data points={len(t_step)}")
        
        # Try first-order fit
        try:
            # Initial guess: K=final_value, tau=time_to_63%, delay=0.1s
            final_val = np.mean(y_normalized[-10:]) if len(y_normalized) > 10 else y_normalized[-1]
            tau_guess = 0.5
            
            popt1, pcov1 = curve_fit(first_order_response, t_step, y_normalized, 
                                   p0=[final_val, tau_guess, 0.1], 
                                   bounds=([0, 0.01, 0], [10, 5, 1]),
                                   maxfev=2000)
            
            y_fit1 = first_order_response(t_step, *popt1)
            r2_1 = 1 - np.sum((y_normalized - y_fit1)**2) / np.sum((y_normalized - np.mean(y_normalized))**2)
            
            print(f"    1st order: K={popt1[0]:.3f}, τ={popt1[1]:.3f}s, delay={popt1[2]:.3f}s, R²={r2_1:.3f}")
            
        except Exception as e:
            print(f"    1st order fit failed: {e}")
            popt1, r2_1 = None, 0
        
        # Try second-order fit
        try:
            wn_guess = 2.0
            zeta_guess = 0.7
            
            popt2, pcov2 = curve_fit(second_order_response, t_step, y_normalized,
                                   p0=[final_val, wn_guess, zeta_guess, 0.1],
                                   bounds=([0, 0.1, 0, 0], [10, 20, 2, 1]),
                                   maxfev=2000)
            
            y_fit2 = second_order_response(t_step, *popt2)
            r2_2 = 1 - np.sum((y_normalized - y_fit2)**2) / np.sum((y_normalized - np.mean(y_normalized))**2)
            
            print(f"    2nd order: K={popt2[0]:.3f}, ωₙ={popt2[1]:.3f}, ζ={popt2[2]:.3f}, delay={popt2[3]:.3f}s, R²={r2_2:.3f}")
            
        except Exception as e:
            print(f"    2nd order fit failed: {e}")
            popt2, r2_2 = None, 0
        
        # Store best result
        if r2_1 > r2_2 and popt1 is not None:
            best_fit = {
                'type': '1st_order',
                'params': popt1,
                'r2': r2_1,
                'step_magnitude': step_magnitude,
                'time': t_step,
                'output': y_normalized,
                'fit': y_fit1
            }
        elif popt2 is not None:
            best_fit = {
                'type': '2nd_order', 
                'params': popt2,
                'r2': r2_2,
                'step_magnitude': step_magnitude,
                'time': t_step,
                'output': y_normalized,
                'fit': y_fit2
            }
        else:
            continue
            
        results.append(best_fit)
    
    return results

def plot_analysis_results(data, x_results, y_results, z_results):
    """Create comprehensive analysis plots"""
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('PX4 System Dynamics Analysis', fontsize=16)
    
    # Time vectors
    valid_indices = ~np.isnan(data['time'])
    time = data['time'][valid_indices]
    
    if len(time) == 0:
        print("❌ No valid time data for plotting")
        return
    
    # Normalize time to start from 0
    time = time - time[0]
    
    # Plot raw data
    axes[0,0].set_title('Acceleration Commands vs Time')
    for key, label, color in [('cmd_ax', 'X-axis', 'red'), ('cmd_ay', 'Y-axis', 'green'), ('cmd_az', 'Z-axis', 'blue')]:
        if key in data and len(data[key]) == len(time):
            axes[0,0].plot(time, data[key][:len(time)], label=label, color=color, alpha=0.7)
    axes[0,0].set_xlabel('Time [s]')
    axes[0,0].set_ylabel('Acceleration [m/s²]')
    axes[0,0].legend()
    axes[0,0].grid(True)
    
    # Plot position responses
    axes[0,1].set_title('Position Responses vs Time')
    for key, label, color in [('pos_x', 'X position', 'red'), ('pos_y', 'Y position', 'green'), ('pos_z', 'Z position', 'blue')]:
        if key in data:
            pos_data = data[key][:len(time)]
            valid_pos = ~np.isnan(pos_data)
            if np.any(valid_pos):
                axes[0,1].plot(time[valid_pos], pos_data[valid_pos], label=label, color=color, alpha=0.7)
    axes[0,1].set_xlabel('Time [s]')
    axes[0,1].set_ylabel('Position [m]')
    axes[0,1].legend()
    axes[0,1].grid(True)
    
    # Plot step response fits
    axis_results = [('X-axis', x_results, 'red'), ('Y-axis', y_results, 'green'), ('Z-axis', z_results, 'blue')]
    
    for i, (axis_name, results, color) in enumerate(axis_results):
        if i >= 2:  # Only plot first 2 axes to fit in 3x2 grid
            break
            
        ax = axes[i+1, 0]
        ax.set_title(f'{axis_name} Step Response Analysis')
        
        if results:
            for j, result in enumerate(results):
                ax.plot(result['time'], result['output'], 'o', color=color, alpha=0.6, 
                       markersize=3, label=f'Data {j+1}')
                ax.plot(result['time'], result['fit'], '-', color=color, linewidth=2,
                       label=f"Fit {j+1} ({result['type']}, R²={result['r2']:.3f})")
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Normalized Response')
        ax.legend()
        ax.grid(True)
        
        # Summary text
        if i < 2:  # Only create text panels for first 2 axes
            ax_text = axes[i+1, 1]
            ax_text.axis('off')
            ax_text.set_title(f'{axis_name} Parameters')
            
            if results:
                text_lines = []
                for j, result in enumerate(results):
                    text_lines.append(f"Step {j+1} ({result['type']}):")
                    if result['type'] == '1st_order':
                        K, tau, delay = result['params']
                        text_lines.append(f"  K = {K:.3f}")
                        text_lines.append(f"  τ = {tau:.3f} s")
                        text_lines.append(f"  delay = {delay:.3f} s")
                    else:  # 2nd_order
                        K, wn, zeta, delay = result['params']
                        text_lines.append(f"  K = {K:.3f}")
                        text_lines.append(f"  ωₙ = {wn:.3f} rad/s")
                        text_lines.append(f"  ζ = {zeta:.3f}")
                        text_lines.append(f"  delay = {delay:.3f} s")
                    text_lines.append(f"  R² = {result['r2']:.3f}")
                    text_lines.append("")
                
                ax_text.text(0.05, 0.95, '\n'.join(text_lines), transform=ax_text.transAxes, 
                            fontfamily='monospace', fontsize=10, verticalalignment='top')
            else:
                ax_text.text(0.05, 0.95, f"No valid {axis_name.lower()} step responses found", 
                            transform=ax_text.transAxes, fontsize=12, verticalalignment='top')
    
    plt.tight_layout()
    
    # Save plot
    plot_filename = f"/tmp/dynamics_analysis_{int(time[0] if len(time) > 0 else 0)}.png"
    plt.savefig(plot_filename, dpi=150, bbox_inches='tight')
    print(f"📊 Analysis plot saved: {plot_filename}")
    
    plt.show()

def generate_mpc_recommendations(x_results, y_results, z_results):
    """Generate MPC tuning recommendations based on identified dynamics"""
    print("\n" + "="*60)
    print("🎯 MPC TUNING RECOMMENDATIONS")
    print("="*60)
    
    all_results = []
    if x_results:
        all_results.extend([(r, 'X') for r in x_results])
    if y_results:
        all_results.extend([(r, 'Y') for r in y_results])
    if z_results:
        all_results.extend([(r, 'Z') for r in z_results])
    
    if not all_results:
        print("❌ No dynamics identified - cannot provide recommendations")
        return
    
    print(f"📊 Analyzed {len(all_results)} step responses")
    
    # Analyze delays
    delays = [r['params'][-1] for r, axis in all_results]  # delay is always last parameter
    avg_delay = np.mean(delays)
    max_delay = np.max(delays)
    
    print(f"\n🕐 CONTROL DELAYS:")
    print(f"  Average delay: {avg_delay:.3f} s")
    print(f"  Maximum delay: {max_delay:.3f} s")
    print(f"  Recommendation: Include {max_delay:.3f}s delay in MPC prediction")
    
    # Analyze time constants
    time_constants = []
    natural_frequencies = []
    
    for result, axis in all_results:
        if result['type'] == '1st_order':
            K, tau, delay = result['params']
            time_constants.append(tau)
            print(f"\n🔧 {axis}-AXIS (1st order):")
            print(f"  Time constant τ = {tau:.3f} s")
            print(f"  Bandwidth ≈ {1/tau:.2f} rad/s")
            
        else:  # 2nd_order
            K, wn, zeta, delay = result['params']
            natural_frequencies.append(wn)
            time_constants.append(1/wn)  # Approximate
            print(f"\n🔧 {axis}-AXIS (2nd order):")
            print(f"  Natural frequency ωₙ = {wn:.3f} rad/s")
            print(f"  Damping ratio ζ = {zeta:.3f}")
            print(f"  Bandwidth ≈ {wn:.2f} rad/s")
            if zeta < 0.7:
                print(f"  ⚠️  Low damping - expect overshoot!")
    
    # MPC recommendations
    if time_constants:
        avg_time_constant = np.mean(time_constants)
        min_time_constant = np.min(time_constants)
        
        print(f"\n🎯 MPC PARAMETER RECOMMENDATIONS:")
        print(f"  System time constant: {avg_time_constant:.3f} s")
        
        # Prediction horizon
        pred_horizon = max(10 * avg_time_constant, 2.0)  # At least 2 seconds
        print(f"  Prediction horizon: {pred_horizon:.1f} s ({int(pred_horizon * 50)} steps at 50Hz)")
        
        # Control horizon  
        ctrl_horizon = min(pred_horizon / 3, 1.0)
        print(f"  Control horizon: {ctrl_horizon:.1f} s ({int(ctrl_horizon * 50)} steps at 50Hz)")
        
        # Constraints
        print(f"\n⚡ SUGGESTED CONSTRAINTS:")
        print(f"  Max acceleration: Based on PX4 MPC_ACC_HOR_MAX parameter")
        print(f"  Max velocity: Based on mission requirements")
        print(f"  Consider rate limiting: {1/min_time_constant:.1f} m/s² per step")
        
        # Model recommendations
        print(f"\n🔄 MODEL IMPROVEMENTS:")
        print(f"  Current model: ẍ = u (double integrator)")
        print(f"  Suggested model: Include {avg_delay:.3f}s delay")
        if natural_frequencies:
            avg_wn = np.mean(natural_frequencies)
            print(f"  Consider 2nd order model: G(s) = K*ωₙ²/(s² + 2*ζ*ωₙ*s + ωₙ²)")
            print(f"  With ωₙ ≈ {avg_wn:.2f} rad/s")
    
    print("\n" + "="*60)

def main():
    print("🔬 Analyzing Simple Dynamics Data...")
    
    # Load data
    data = load_latest_data()
    if data is None:
        return
    
    print(f"📊 Data summary:")
    for key, arr in data.items():
        valid_count = np.sum(~np.isnan(arr)) if arr.dtype == np.float64 else len(arr)
        print(f"  {key}: {len(arr)} points ({valid_count} valid)")
    
    # Extract valid position data for analysis
    if 'time' not in data or len(data['time']) == 0:
        print("❌ No time data found!")
        return
    
    # Analyze each axis
    x_results = analyze_step_response(data['time'], 
                                    data.get('cmd_ax', np.zeros_like(data['time'])),
                                    data.get('pos_x', np.full_like(data['time'], np.nan)),
                                    'X')
    
    y_results = analyze_step_response(data['time'],
                                    data.get('cmd_ay', np.zeros_like(data['time'])), 
                                    data.get('pos_y', np.full_like(data['time'], np.nan)),
                                    'Y')
    
    z_results = analyze_step_response(data['time'],
                                    data.get('cmd_az', np.zeros_like(data['time'])),
                                    data.get('pos_z', np.full_like(data['time'], np.nan)), 
                                    'Z')
    
    # Plot results
    plot_analysis_results(data, x_results, y_results, z_results)
    
    # Generate recommendations
    generate_mpc_recommendations(x_results, y_results, z_results)

if __name__ == "__main__":
    main()