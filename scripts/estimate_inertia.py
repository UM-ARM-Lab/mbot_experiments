#!/usr/bin/env python3
"""
Reads latest log file from sysID experiment and estimates mass and inertia of the robot (by doing linear regression).
Returns the values on the terminal and saves to .pt file.
"""

import argparse
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple, Optional
import os
import sys
import glob
import lcm
import torch
from scipy import signal
from collections import defaultdict

# Add paths for imports
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from exp_utils import import_lcm_messages
from config import MOTION_CMD_CHANNEL, DEFAULT_VELOCITY_CHANNEL, MOTION_STATUS_CHANNEL

# Import LCM message types globally
import_lcm_messages()
global motion_command_t, motion_status_t, twist2D_t, vicon_twist_t
try:
    from mbot_lcm_msgs.motion_command_t import motion_command_t
    from mbot_lcm_msgs.motion_status_t import motion_status_t
    from mbot_lcm_msgs.twist2D_t import twist2D_t
    from vicon_msgs.vicon_twist_t import vicon_twist_t
except ImportError:
    print("Warning: Could not import LCM message types. Make sure you're running on desktop with proper paths.")

from exp_utils import extract_motion_segments, find_latest_log_file, read_lcm_experiment_log

def estimate_parameters_from_segment(segment: Dict) -> Dict:
    """
    Estimate mass and inertia using linear fitting over entire segment.
    No numerical differentiation - use slope of velocity vs time directly.
    """
    test_name = segment['test_name']
    command = segment['command']
    measured_twists = segment['measured_twists']
    status = segment['status']
    
    print(f"📊 Analyzing segment: {test_name}")
    
    # Only process completed segments
    if status != 'COMPLETED':
        print(f"   ⚠️ Segment not completed (status: {status}), skipping")
        return {}
    
    if len(measured_twists) < 10:
        print(f"   ⚠️ Insufficient data points ({len(measured_twists)})")
        return {}
    
    # Extract time series data
    times = np.array([t for t, _, _, _ in measured_twists])
    vx_data = np.array([vx for _, vx, _, _ in measured_twists])
    wz_data = np.array([wz for _, _, _, wz in measured_twists])
    
    # Classify motion type based on command
    ax_cmd = command['ax']
    az_cmd = command['az']
    
    result = {
        'test_name': test_name,
        'command': command,
        'data_points': len(measured_twists),
        'status': status,
        'ax_cmd': ax_cmd,
        'az_cmd': az_cmd
    }
    
    # Forward motion analysis (ax != 0)
    if abs(ax_cmd) > 0.01:
        if len(times) >= 3:
            # Linear fit: vx(t) = vx0 + a_measured * t
            slope, intercept = np.polyfit(times, vx_data, 1)
            result['measured_ax'] = slope
            result['vx0_fit'] = intercept
            result['motion_type'] = 'forward'
            print(f"   Forward motion: ax_cmd={ax_cmd:.3f}, ax_measured={slope:.3f} m/s²")
        else:
            result['motion_type'] = 'forward_insufficient'
    
    # Angular motion analysis (az != 0)
    if abs(az_cmd) > 0.01:
        if len(times) >= 3:
            # Linear fit: wz(t) = wz0 + alpha_measured * t
            slope, intercept = np.polyfit(times, wz_data, 1)
            result['measured_az'] = slope
            result['wz0_fit'] = intercept
            result['motion_type'] = 'angular' if 'motion_type' not in result else 'combined'
            print(f"   Angular motion: az_cmd={az_cmd:.3f}, az_measured={slope:.3f} rad/s²")
        else:
            result['motion_type'] = 'angular_insufficient'
    
    # Stationary or insufficient command
    if abs(ax_cmd) <= 0.01 and abs(az_cmd) <= 0.01:
        result['motion_type'] = 'stationary'
        print(f"   Stationary motion (ax={ax_cmd:.3f}, az={az_cmd:.3f})")
    
    print(f"   ✓ Analysis complete: {result.get('motion_type', 'unknown')} motion")
    return result


def estimate_mass_from_segments(segments: List[Dict]) -> Dict:
    """
    Estimate robot mass using linear regression over forward motion segments.
    
    Physics: F_actual = m * ax_measured
    We assume F_actual is proportional to the commanded acceleration: F_actual = k_f * ax_cmd
    Therefore: k_f * ax_cmd = m * ax_measured
    So: m ≈ (ax_cmd_avg / ax_measured_avg) assuming k_f ≈ 1
    """
    forward_segments = []
    
    for segment in segments:
        if segment.get('motion_type') == 'forward' and 'measured_ax' in segment:
            forward_segments.append(segment)
    
    if len(forward_segments) < 2:
        print(f"⚠️ Need at least 2 forward segments for mass estimation, got {len(forward_segments)}")
        return {}
    
    print(f"📏 Estimating mass from {len(forward_segments)} forward motion segments...")
    
    # Extract commanded and measured accelerations
    ax_commanded = np.array([seg['ax_cmd'] for seg in forward_segments])
    ax_measured = np.array([seg['measured_ax'] for seg in forward_segments])
    
    # Linear regression for visualization and R² calculation
    if len(ax_commanded) >= 2 and np.std(ax_commanded) > 0.001:
        slope, intercept = np.polyfit(ax_commanded, ax_measured, 1)
        
        # Mass estimation: m ≈ ax_cmd_avg / ax_measured_avg (assuming k_f ≈ 1)
        estimated_mass = np.mean(ax_commanded) / np.mean(ax_measured) if np.mean(ax_measured) > 0.001 else None
        
        result = {
            'estimated_mass_kg': estimated_mass,
            'slope': slope,
            'intercept': intercept,
            'n_segments': len(forward_segments),
            'commanded_accelerations': ax_commanded.tolist(),
            'measured_accelerations': ax_measured.tolist(),
            'r_squared': np.corrcoef(ax_commanded, ax_measured)[0,1]**2 if len(ax_commanded) > 1 else 0
        }
        
        print(f"   Mass estimate: {estimated_mass:.3f} kg")
        print(f"   Linear fit: ax_measured = {slope:.3f} * ax_cmd + {intercept:.3f}")
        print(f"   R² = {result['r_squared']:.3f}")
        print(f"   Note: Using m ≈ ax_cmd_avg / ax_measured_avg = {np.mean(ax_commanded):.3f} / {np.mean(ax_measured):.3f} = {estimated_mass:.3f} kg")
        
        return result
    else:
        print("⚠️ Insufficient variation in commanded accelerations for mass estimation")
        return {}


# def estimate_inertia_from_segments(segments: List[Dict]) -> Dict:
#     """
#     Estimate robot moment of inertia using linear regression over angular motion segments.
    
#     Physics: τ_actual = I * az_measured
#     We assume τ_actual is proportional to the commanded angular acceleration: τ_actual = k_τ * az_cmd
#     Therefore: k_τ * az_cmd = I * az_measured
#     So: I ≈ (az_cmd_avg / az_measured_avg) assuming k_τ ≈ 1
#     """
#     angular_segments = []
    
#     for segment in segments:
#         if segment.get('motion_type') == 'angular' and 'measured_az' in segment:
#             angular_segments.append(segment)
    
#     if len(angular_segments) < 2:
#         print(f"⚠️ Need at least 2 angular segments for inertia estimation, got {len(angular_segments)}")
#         return {}
    
#     print(f"📏 Estimating inertia from {len(angular_segments)} angular motion segments...")
    
#     # Extract commanded and measured angular accelerations
#     az_commanded = np.array([seg['az_cmd'] for seg in angular_segments])
#     az_measured = np.array([seg['measured_az'] for seg in angular_segments])
    
#     # Linear regression for visualization and R² calculation
#     if len(az_commanded) >= 2 and np.std(az_commanded) > 0.001:
#         slope, intercept = np.polyfit(az_commanded, az_measured, 1)
        
#         # Inertia estimation: I ≈ az_cmd_avg / az_measured_avg (assuming k_τ ≈ 1)
#         estimated_inertia = np.mean(az_commanded) / np.mean(az_measured) if np.mean(az_measured) > 0.001 else None
        
#         result = {
#             'estimated_inertia_kg_m2': estimated_inertia,
#             'slope': slope,
#             'intercept': intercept,
#             'n_segments': len(angular_segments),
#             'commanded_accelerations': az_commanded.tolist(),
#             'measured_accelerations': az_measured.tolist(),
#             'r_squared': np.corrcoef(az_commanded, az_measured)[0,1]**2 if len(az_commanded) > 1 else 0
#         }
        
#         print(f"   Inertia estimate: {estimated_inertia:.6f} kg⋅m²")
#         print(f"   Linear fit: az_measured = {slope:.3f} * az_cmd + {intercept:.3f}")
#         print(f"   R² = {result['r_squared']:.3f}")
#         print(f"   Note: Using I ≈ az_cmd_avg / az_measured_avg = {np.mean(az_commanded):.3f} / {np.mean(az_measured):.3f} = {estimated_inertia:.6f} kg⋅m²")
        
#         return result
#     else:
#         print("⚠️ Insufficient variation in commanded angular accelerations for inertia estimation")
#         return {}


def get_reference_parameters(radius: float = 0.08) -> Dict:
    """
    Get hardcoded reference parameters for comparison.
    
    Args:
        radius: Robot radius in meters (default 0.11m for MBot)
        
    Returns:
        Dictionary with reference mass and inertia values
    """
    reference_mass = 1.093  # kg (Weighted in August 2025)
    reference_inertia = (0.5) * reference_mass * (radius**2)  # kg⋅m²
    wheel_diameter = 0.084 # https://www.pololu.com/product/3275
    
    result = {
        'reference_mass_kg': reference_mass,
        'reference_inertia_kg_m2': reference_inertia,
        'radius_m': radius,
        'wheel_diameter_m': wheel_diameter,
        'calculation': f"I = (1/12) * m * (2 * r²) = (1/12) * {reference_mass} * (2 * {radius}²)"
    }
    
    print(f"📚 Reference parameters:")
    print(f"   Mass: {reference_mass:.3f} kg")
    print(f"   Inertia: {reference_inertia:.6f} kg⋅m²")
    print(f"   Radius: {radius:.3f} m")
    
    return result


def plot_individual_segment_velocities(segments: List[Dict], output_dir: str = "analysis_plots"):
    """Create individual plots for each segment showing expected vs observed velocities"""
    output_dir = os.path.join(script_dir, "..", "real_data", "sysID", output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"📊 Creating individual segment velocity plots...")
    
    for i, segment in enumerate(segments):
        if segment['status'] != 'COMPLETED' or not segment.get('measured_twists'):
            continue
            
        test_name = segment['test_name']
        command = segment['command']
        measured_twists = segment['measured_twists']
        
        # Extract time series data
        times = np.array([t for t, _, _, _ in measured_twists])
        vx_data = np.array([vx for _, vx, _, _ in measured_twists])
        wz_data = np.array([wz for _, _, _, wz in measured_twists])
        
        # Calculate expected velocities based on command
        vx_expected = command['vx0'] + command['ax'] * times
        wz_expected = command['wz0'] + command['az'] * times
        
        # Create subplot figure
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Plot linear velocity
        ax1.plot(times, vx_data, 'b-', label='Observed vx', linewidth=2)
        ax1.plot(times, vx_expected, 'r--', label='Expected vx', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Linear Velocity (m/s)')
        ax1.set_title(f'Linear Velocity - {test_name}\nax={command["ax"]:.3f} m/s²')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot angular velocity  
        ax2.plot(times, wz_data, 'b-', label='Observed ωz', linewidth=2)
        ax2.plot(times, wz_expected, 'r--', label='Expected ωz', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angular Velocity (rad/s)')
        ax2.set_title(f'Angular Velocity - {test_name}\naz={command["az"]:.3f} rad/s²')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        plt.suptitle(f'Segment {i+1}: {test_name} (Duration: {segment["actual_duration"]:.2f}s)', 
                     fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        # Save individual plot
        plot_path = os.path.join(output_dir, f'segment_{i+1}_{test_name.replace("/", "_")}.png')
        plt.savefig(plot_path, dpi=150, bbox_inches='tight')
        plt.close(fig)  # Close to save memory
        
        print(f"   Saved: {plot_path}")


def plot_analysis_results(segments: List[Dict], mass_result: Dict, inertia_result: Dict, output_dir: str = "analysis_plots"):
    """Create plots showing the analysis results and linear fits"""
    output_dir = os.path.join(script_dir, "..", "real_data", "sysID", output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    # Create individual segment plots first
    plot_individual_segment_velocities(segments, output_dir)
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('MBot System Identification Results', fontsize=16, fontweight='bold')
    
    # Plot 1: Commanded vs Measured Accelerations (Mass)
    if mass_result:
        ax = axes[0, 0]
        ax_cmd = np.array(mass_result['commanded_accelerations'])
        ax_meas = np.array(mass_result['measured_accelerations'])
        
        ax.scatter(ax_cmd, ax_meas, alpha=0.7, s=50, label='Data points')
        ax.plot(ax_cmd, mass_result['slope'] * ax_cmd + mass_result['intercept'], 
                'r--', label=f'Linear fit (R²={mass_result["r_squared"]:.3f})')
        ax.set_xlabel('Commanded Acceleration (m/s²)')
        ax.set_ylabel('Measured Acceleration (m/s²)')
        ax.set_title(f'Mass Estimation: {mass_result["estimated_mass_kg"]:.3f} kg')
        ax.grid(True, alpha=0.3)
        ax.legend()
    else:
        axes[0, 0].text(0.5, 0.5, 'No mass estimation data', ha='center', va='center', transform=axes[0, 0].transAxes)
        axes[0, 0].set_title('Mass Estimation: No data')
    
    # Plot 2: Commanded vs Measured Angular Accelerations (Inertia)
    if inertia_result:
        ax = axes[0, 1]
        az_cmd = np.array(inertia_result['commanded_accelerations'])
        az_meas = np.array(inertia_result['measured_accelerations'])
        
        ax.scatter(az_cmd, az_meas, alpha=0.7, s=50, label='Data points')
        ax.plot(az_cmd, inertia_result['slope'] * az_cmd + inertia_result['intercept'], 
                'r--', label=f'Linear fit (R²={inertia_result["r_squared"]:.3f})')
        ax.set_xlabel('Commanded Angular Acceleration (rad/s²)')
        ax.set_ylabel('Measured Angular Acceleration (rad/s²)')
        ax.set_title(f'Inertia Estimation: {inertia_result["estimated_inertia_kg_m2"]:.6f} kg⋅m²')
        ax.grid(True, alpha=0.3)
        ax.legend()
    else:
        axes[0, 1].text(0.5, 0.5, 'No inertia estimation data', ha='center', va='center', transform=axes[0, 1].transAxes)
        axes[0, 1].set_title('Inertia Estimation: No data')
    
    # Plot 3: Segment Analysis Summary
    ax = axes[1, 0]
    motion_types = {}
    for seg in segments:
        mot_type = seg.get('motion_type', 'unknown')
        motion_types[mot_type] = motion_types.get(mot_type, 0) + 1
    
    if motion_types:
        ax.bar(motion_types.keys(), motion_types.values(), alpha=0.7)
        ax.set_xlabel('Motion Type')
        ax.set_ylabel('Count')
        ax.set_title('Segment Analysis Summary')
        ax.tick_params(axis='x', rotation=45)
    
    # Plot 4: Parameter Comparison
    ax = axes[1, 1]
    reference_params = get_reference_parameters()
    
    params = ['Mass (kg)', 'Inertia (kg⋅m²)']
    estimated_vals = []
    reference_vals = []
    
    if mass_result:
        estimated_vals.append(mass_result['estimated_mass_kg'])
        reference_vals.append(reference_params['reference_mass_kg'])
    else:
        estimated_vals.append(0)
        reference_vals.append(reference_params['reference_mass_kg'])
    
    if inertia_result:
        estimated_vals.append(inertia_result['estimated_inertia_kg_m2'])
        reference_vals.append(reference_params['reference_inertia_kg_m2'])
    else:
        estimated_vals.append(0)
        reference_vals.append(reference_params['reference_inertia_kg_m2'])
    
    x = np.arange(len(params))
    width = 0.35
    
    ax.bar(x - width/2, estimated_vals, width, label='Estimated', alpha=0.7)
    ax.bar(x + width/2, reference_vals, width, label='Reference', alpha=0.7)
    ax.set_xlabel('Parameters')
    ax.set_ylabel('Value')
    ax.set_title('Estimated vs Reference Parameters')
    ax.set_xticks(x)
    ax.set_xticklabels(params)
    ax.legend()
    
    plt.tight_layout()
    plot_path = os.path.join(output_dir, 'system_identification_results.png')
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.show()
    
    print(f"📊 Analysis plots saved to: {plot_path}")


def main():
    """Main analysis entry point"""
    parser = argparse.ArgumentParser(description="Estimate mass and inertia from LCM experiment logs using linear regression")
    parser.add_argument("--log_file", help="Path to LCM log file (uses latest sysID log if not specified)")
    parser.add_argument("--plot", action="store_true", help="Generate analysis plots")
    
    args = parser.parse_args()
    
    # Use latest log file if not specified
    log_file = args.log_file
    if log_file is None:
        log_file = find_latest_log_file(script_dir, "sysID")
        if log_file is None:
            print("❌ No log file specified and no sysID logs found")
            return 1
        print(f"📁 Using latest log file: {log_file}")
    
    print("🔬 MBot Mass and Inertia Estimation (Linear Regression)")
    print("=" * 60)
    print(f"📁 Log file: {log_file}")
    print(f"📊 Method: Physics-based linear regression")
    print()
    
    # Read and analyze log file
    print("📖 Reading LCM log file...")
    log_data = read_lcm_experiment_log(log_file)
    
    if not log_data:
        print("❌ Failed to read log file")
        return 1
    
    # Extract motion segments for analysis
    segments = extract_motion_segments(log_data)
    
    if not segments:
        print("❌ No usable motion segments found")
        return 1
    
    # Analyze each segment
    print(f"\n🔬 Analyzing {len(segments)} motion segments...")
    segment_results = []
    
    for segment in segments:
        result = estimate_parameters_from_segment(segment)
        if result:
            segment_results.append(result)
    
    # Estimate parameters using physics-based regression
    print(f"\n📊 Physics-based Parameter Estimation:")
    mass_result = estimate_mass_from_segments(segment_results)
    # inertia_result = estimate_inertia_from_segments(segment_results)
    inertia_result = None

    # Get reference parameters
    print(f"\n📚 Reference Parameters:")
    reference_params = get_reference_parameters()
    
    # Prepare results summary
    print(f"\n📊 Analysis Summary:")
    print(f"   Total segments processed: {len(segment_results)}")
    
    if mass_result:
        estimated_mass = mass_result['estimated_mass_kg']
        reference_mass = reference_params['reference_mass_kg']
        mass_error = abs(estimated_mass - reference_mass) / reference_mass * 100 if estimated_mass else None
        print(f"   📏 Mass:")
        print(f"      Estimated: {estimated_mass:.3f} kg")
        print(f"      Reference: {reference_mass:.3f} kg")
        if mass_error:
            print(f"      Error: {mass_error:.1f}%")
    else:
        print(f"   ⚠️ No mass estimate available")
    
    if inertia_result:
        estimated_inertia = inertia_result['estimated_inertia_kg_m2']
        reference_inertia = reference_params['reference_inertia_kg_m2']
        inertia_error = abs(estimated_inertia - reference_inertia) / reference_inertia * 100 if estimated_inertia else None
        print(f"   📏 Inertia:")
        print(f"      Estimated: {estimated_inertia:.6f} kg⋅m²")
        print(f"      Reference: {reference_inertia:.6f} kg⋅m²")
        if inertia_error:
            print(f"      Error: {inertia_error:.1f}%")
    else:
        print(f"   ⚠️ No inertia estimate available")
    
    # Save parameters to .pt files
    print(f"\n💾 Saving parameters...")
    
    # Save estimated parameters
    if mass_result or inertia_result:
        estimated_params = {
            'timestamp': log_file,
            'analysis_method': 'linear_regression',
            'mass_result': mass_result,
            'inertia_result': inertia_result,
            'segment_count': len(segment_results)
        }
        
        estimated_file = os.path.join(script_dir, "..", "real_data", "sysID", "estimated_parameters.pt")
        torch.save(estimated_params, estimated_file)
        print(f"   Estimated parameters saved to: {estimated_file}")
    
    # Save reference parameters
    reference_file = os.path.join(script_dir, "..", "real_data", "sysID", "reference_parameters.pt")
    torch.save(reference_params, reference_file)
    print(f"   Reference parameters saved to: {reference_file}")
    
    # Generate plots if requested
    if args.plot:
        plot_analysis_results(segment_results, mass_result, inertia_result)
    
    print(f"\n✅ Analysis complete!")
    return 0


if __name__ == "__main__":
    sys.exit(main())