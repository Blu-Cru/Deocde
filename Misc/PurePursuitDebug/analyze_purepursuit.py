#!/usr/bin/env python3
"""
PurePursuit Debug Log Analyzer
Visualizes logged data from PurePursuitDebugTeleOp to diagnose path following issues
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from pathlib import Path

def load_log_file(filepath):
    """Load CSV log file, skipping comment lines"""
    return pd.read_csv(filepath, comment='#')

def plot_trajectory(df, ax):
    """Plot 2D trajectory showing actual path vs target points"""
    ax.plot(df['robot_x'], df['robot_y'], 'b-', linewidth=2, label='Actual Path', alpha=0.7)
    ax.scatter(df['robot_x'].iloc[0], df['robot_y'].iloc[0],
              c='green', s=100, marker='o', label='Start', zorder=5)
    ax.scatter(df['robot_x'].iloc[-1], df['robot_y'].iloc[-1],
              c='red', s=100, marker='X', label='End', zorder=5)

    # Plot target points
    ax.scatter(df['target_x'], df['target_y'],
              c='orange', s=20, alpha=0.3, label='Target Points')

    # Draw intended path (straight line if targets are linear)
    # Find unique target points to show waypoints
    targets = df[['target_x', 'target_y']].drop_duplicates()
    if len(targets) > 1:
        ax.plot(targets['target_x'], targets['target_y'],
               'r--', linewidth=1, alpha=0.5, label='Intended Path')

    ax.set_xlabel('X Position (inches)')
    ax.set_ylabel('Y Position (inches)')
    ax.set_title('Robot Trajectory')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    ax.legend()

def plot_position_error(df, ax):
    """Plot position errors over time"""
    time_sec = df['timestamp_ms'] / 1000.0

    # Calculate distance from intended path (assuming straight line along X axis)
    # For straight line tests, Y should stay near 0
    ax.plot(time_sec, df['robot_y'], 'r-', linewidth=2, label='Y Position (Lateral Drift)')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3, label='Ideal Y=0')

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Y Position (inches)')
    ax.set_title('Lateral Drift from Straight Path')
    ax.grid(True, alpha=0.3)
    ax.legend()

def plot_heading(df, ax):
    """Plot heading and heading error over time"""
    time_sec = df['timestamp_ms'] / 1000.0

    ax.plot(time_sec, df['robot_heading_deg'], 'b-', linewidth=2, label='Actual Heading')
    ax.plot(time_sec, df['heading_target_deg'], 'g--', linewidth=1.5, label='Target Heading')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Heading (degrees)')
    ax.set_title('Heading Control')
    ax.grid(True, alpha=0.3)
    ax.legend()

def plot_heading_error(df, ax):
    """Plot heading error over time"""
    time_sec = df['timestamp_ms'] / 1000.0

    ax.plot(time_sec, df['heading_err_deg'], 'r-', linewidth=2)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Heading Error (degrees)')
    ax.set_title('Heading Error Over Time')
    ax.grid(True, alpha=0.3)

def plot_linear_pid(df, ax):
    """Plot linear PID terms over time"""
    time_sec = df['timestamp_ms'] / 1000.0

    ax.plot(time_sec, df['linear_error'], 'b-', linewidth=1.5, label='Error', alpha=0.7)
    ax.plot(time_sec, df['linear_p'], 'g-', linewidth=1.5, label='P Term', alpha=0.7)
    ax.plot(time_sec, df['linear_d'], 'r-', linewidth=1.5, label='D Term', alpha=0.7)
    ax.plot(time_sec, df['linear_out'], 'k-', linewidth=2, label='Output', alpha=0.9)

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Value')
    ax.set_title('Linear Velocity PD Control')
    ax.grid(True, alpha=0.3)
    ax.legend()

def plot_heading_pid(df, ax):
    """Plot heading PID terms over time"""
    time_sec = df['timestamp_ms'] / 1000.0

    ax.plot(time_sec, df['heading_err_deg'], 'b-', linewidth=1.5, label='Error (deg)', alpha=0.7)
    ax.plot(time_sec, df['heading_p'], 'g-', linewidth=1.5, label='P Term', alpha=0.7)
    ax.plot(time_sec, df['heading_d'], 'r-', linewidth=1.5, label='D Term', alpha=0.7)
    ax.plot(time_sec, df['heading_out'], 'k-', linewidth=2, label='Output', alpha=0.9)

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Value')
    ax.set_title('Heading PD Control')
    ax.grid(True, alpha=0.3)
    ax.legend()

def plot_velocity(df, ax):
    """Plot robot velocity over time"""
    time_sec = df['timestamp_ms'] / 1000.0

    # Calculate total velocity magnitude
    vel_mag = np.sqrt(df['vel_x']**2 + df['vel_y']**2)

    ax.plot(time_sec, df['vel_x'], 'b-', linewidth=1.5, label='X Velocity', alpha=0.7)
    ax.plot(time_sec, df['vel_y'], 'r-', linewidth=1.5, label='Y Velocity', alpha=0.7)
    ax.plot(time_sec, vel_mag, 'k-', linewidth=2, label='Total Velocity', alpha=0.9)

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Velocity (in/s)')
    ax.set_title('Robot Velocity')
    ax.grid(True, alpha=0.3)
    ax.legend()

def plot_distance_remaining(df, ax):
    """Plot distance remaining to end of path"""
    time_sec = df['timestamp_ms'] / 1000.0

    ax.plot(time_sec, df['dist_remaining'], 'b-', linewidth=2)

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Distance (inches)')
    ax.set_title('Distance Remaining to Path End')
    ax.grid(True, alpha=0.3)

def print_statistics(df):
    """Print summary statistics about the run"""
    print("\n" + "="*60)
    print("PUREPURSUIT DEBUG LOG ANALYSIS")
    print("="*60)

    print(f"\nTest Duration: {df['timestamp_ms'].iloc[-1] / 1000.0:.2f} seconds")
    print(f"Number of Samples: {len(df)}")
    print(f"Average Sample Rate: {len(df) / (df['timestamp_ms'].iloc[-1] / 1000.0):.1f} Hz")

    print(f"\n--- Path Following ---")
    print(f"Start Position: ({df['robot_x'].iloc[0]:.2f}, {df['robot_y'].iloc[0]:.2f})")
    print(f"End Position: ({df['robot_x'].iloc[-1]:.2f}, {df['robot_y'].iloc[-1]:.2f})")
    print(f"Distance Traveled: {np.sum(np.sqrt(np.diff(df['robot_x'])**2 + np.diff(df['robot_y'])**2)):.2f} inches")

    print(f"\n--- Lateral Drift (Y Position) ---")
    print(f"Maximum Drift: {df['robot_y'].abs().max():.3f} inches")
    print(f"Average Drift: {df['robot_y'].abs().mean():.3f} inches")
    print(f"Final Drift: {df['robot_y'].iloc[-1]:.3f} inches")

    print(f"\n--- Heading ---")
    print(f"Start Heading: {df['robot_heading_deg'].iloc[0]:.2f}°")
    print(f"End Heading: {df['robot_heading_deg'].iloc[-1]:.2f}°")
    print(f"Max Heading Error: {df['heading_err_deg'].abs().max():.2f}°")
    print(f"Average Heading Error: {df['heading_err_deg'].abs().mean():.2f}°")

    print(f"\n--- Velocity ---")
    vel_mag = np.sqrt(df['vel_x']**2 + df['vel_y']**2)
    print(f"Average Velocity: {vel_mag.mean():.2f} in/s")
    print(f"Max Velocity: {vel_mag.max():.2f} in/s")

    print(f"\n--- Control Output ---")
    print(f"Linear Output Range: [{df['linear_out'].min():.3f}, {df['linear_out'].max():.3f}]")
    print(f"Heading Output Range: [{df['heading_out'].min():.3f}, {df['heading_out'].max():.3f}]")

    if df['backwards'].any():
        backwards_pct = df['backwards'].sum() / len(df) * 100
        print(f"\nBackwards Driving: {backwards_pct:.1f}% of time")

    print("\n" + "="*60)

def analyze_log(filepath, save_plots=True):
    """Main analysis function"""
    print(f"\nLoading log file: {filepath}")
    df = load_log_file(filepath)

    # Print statistics
    print_statistics(df)

    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))

    # Create 3x3 grid of subplots
    ax1 = plt.subplot(3, 3, 1)  # Trajectory
    ax2 = plt.subplot(3, 3, 2)  # Position error
    ax3 = plt.subplot(3, 3, 3)  # Heading
    ax4 = plt.subplot(3, 3, 4)  # Heading error
    ax5 = plt.subplot(3, 3, 5)  # Linear PID
    ax6 = plt.subplot(3, 3, 6)  # Heading PID
    ax7 = plt.subplot(3, 3, 7)  # Velocity
    ax8 = plt.subplot(3, 3, 8)  # Distance remaining
    ax9 = plt.subplot(3, 3, 9)  # Reserved for future use

    # Generate plots
    plot_trajectory(df, ax1)
    plot_position_error(df, ax2)
    plot_heading(df, ax3)
    plot_heading_error(df, ax4)
    plot_linear_pid(df, ax5)
    plot_heading_pid(df, ax6)
    plot_velocity(df, ax7)
    plot_distance_remaining(df, ax8)

    # Add overall title
    log_filename = os.path.basename(filepath)
    fig.suptitle(f'PurePursuit Debug Analysis: {log_filename}',
                fontsize=14, fontweight='bold')

    plt.tight_layout()

    # Save plots if requested
    if save_plots:
        output_path = filepath.replace('.csv', '_analysis.png')
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved to: {output_path}")

    plt.show()

def find_latest_log(log_dir="/sdcard/FIRST/purePursuit_logs/"):
    """Find the most recent log file"""
    # Check if directory exists
    if not os.path.exists(log_dir):
        return None

    # Find all CSV files
    csv_files = list(Path(log_dir).glob("pp_debug_*.csv"))

    if not csv_files:
        return None

    # Return most recent file
    return str(max(csv_files, key=os.path.getmtime))

def main():
    """Main entry point"""
    print("="*60)
    print("PurePursuit Debug Log Analyzer")
    print("="*60)

    # Determine which log file to analyze
    if len(sys.argv) > 1:
        # Log file specified as command line argument
        log_file = sys.argv[1]
    else:
        # Try to find latest log file
        log_file = find_latest_log()

        if log_file is None:
            print("\nNo log file specified and couldn't find logs in default directory.")
            print("\nUsage:")
            print("  python analyze_purepursuit.py <log_file.csv>")
            print("\nOr place this script in the project directory and it will")
            print("automatically find the latest log file.")
            sys.exit(1)

        print(f"\nNo log file specified, using latest: {os.path.basename(log_file)}")

    # Check if file exists
    if not os.path.exists(log_file):
        print(f"\nError: Log file not found: {log_file}")
        sys.exit(1)

    # Analyze the log
    analyze_log(log_file, save_plots=True)

if __name__ == "__main__":
    main()
