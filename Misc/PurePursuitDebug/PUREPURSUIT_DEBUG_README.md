# PurePursuit Debug System

A comprehensive debugging system for diagnosing PurePursuit path following issues.

## Quick Start

### 1. Run the Debug TeleOp

1. Build and deploy the code to your robot
2. On the driver station, select **"PurePursuit Debug"** from the TeleOp list (in "test" group)
3. Press **Y** to reset robot position to (0, 0, 0°)
4. Press **A** to start following a test path (logging starts automatically)
5. Watch the telemetry for real-time debug information
6. Press **B** to stop (or let it complete automatically)

### 2. Get the Log Files

Log files are saved to `/sdcard/FIRST/purePursuit_logs/` on the robot Control Hub.

**To retrieve them:**
1. Open Android Studio
2. Go to **View → Tool Windows → Device File Explorer**
3. Navigate to `/sdcard/FIRST/purePursuit_logs/`
4. Right-click the log file → **Save As...**
5. Save to your computer

### 3. Analyze the Data

#### Install Python Dependencies

```bash
pip install -r requirements_analysis.txt
```

Or manually:
```bash
pip install pandas matplotlib numpy
```

#### Run the Analysis Script

**Option 1: Specify log file**
```bash
python analyze_purepursuit.py path/to/pp_debug_20260110_143022.csv
```

**Option 2: Auto-detect latest (if logs are in default location)**
```bash
python analyze_purepursuit.py
```

The script will:
- Print detailed statistics to the console
- Generate a 3x3 grid of diagnostic plots
- Save the plots as a PNG file next to the CSV

## Controls

| Button | Action |
|--------|--------|
| **A** | Start following selected test path and begin logging |
| **B** | Stop path following and save log |
| **X** | Cycle through test paths |
| **Y** | Reset robot position to (0, 0, 0°) |
| **DPad Up** | Start new log file (while following) |
| **Left/Right Sticks** | Manual drive when not following path |

## Test Paths

1. **Straight Line Short** - 40 inches forward, basic straight-line tracking
2. **Straight Line Long** - 100 inches forward, tests drift over distance
3. **Straight Line Multi-Segment** - 90 inches in 3 segments, tests transitions
4. **Simple Curve** - Gentle curve, tests heading control

## What the Analysis Shows

The Python script generates 8 diagnostic plots:

### 1. Robot Trajectory (Top Left)
- **2D path visualization**
- Shows actual path vs intended path
- Green = start, Red = end, Orange dots = target points
- **Look for:** Deviations from intended straight line

### 2. Lateral Drift (Top Middle)
- **Y position over time** (should be ~0 for straight paths)
- **Look for:** Accumulating drift, oscillations

### 3. Heading (Top Right)
- **Actual vs target heading**
- **Look for:** Large deviations, oscillations

### 4. Heading Error (Middle Left)
- **Heading error over time**
- **Look for:** Persistent bias, oscillations

### 5. Linear PD Control (Middle Center)
- **Error, P term, D term, and output**
- **Look for:**
  - P and D terms oscillating together (possible instability)
  - D term too small (underdamped)
  - P term too small (slow response)

### 6. Heading PD Control (Middle Right)
- **Error, P term, D term, and output**
- **Look for:** Same as linear control

### 7. Velocity (Bottom Left)
- **X, Y, and total velocity**
- **Look for:** Excessive Y velocity (lateral drift)

### 8. Distance Remaining (Bottom Middle)
- **Distance to path end over time**
- **Look for:** Jumps at segment transitions, incorrect calculations

## Console Statistics

The script prints detailed statistics including:
- Test duration and sample rate
- Start/end positions and distance traveled
- Maximum, average, and final lateral drift
- Heading errors
- Velocity ranges
- PID output ranges
- Backwards driving percentage

## Diagnosing Common Issues

### Issue: Robot drifts sideways on straight paths

**What to check:**
1. **Lateral Drift plot** - Is Y position accumulating?
2. **Heading plot** - Is heading drifting from 0°?
3. **Heading PD Control** - Is heading correction adequate?

**Possible causes:**
- Heading controller P gain too low (increase `SixWheelPID.pR`)
- Heading controller D gain too low (increase `SixWheelPID.dR`)
- Odometry calibration issue (check encoder directions/positions)
- Robot mechanical issue (wheel alignment, friction)

### Issue: Oscillation around path

**What to check:**
1. **Trajectory** - Snake-like pattern?
2. **Heading Error** - Oscillating?
3. **PD Control plots** - P and D terms fighting?

**Possible causes:**
- P gain too high (decrease `pR` or `pXY`)
- D gain too low (increase `dR` or `dXY`)
- Loop time too slow

### Issue: Robot overshoots or can't reach target

**What to check:**
1. **Distance Remaining** - Decreasing smoothly?
2. **Linear PD Control** - Appropriate response?
3. **Velocity** - Too fast or too slow?

**Possible causes:**
- Linear P gain too low/high (adjust `pXY`)
- `STOP_DISTANCE` too large (robot stops too early)
- Distance calculation error

### Issue: Path following stutters or jerks

**What to check:**
1. **Velocity** - Sudden changes?
2. **Distance Remaining** - Jumps at segment transitions?
3. **Target Point (Trajectory)** - Discontinuous?

**Possible causes:**
- Segment transition logic issue
- Lookahead distance too small
- Loop time inconsistent

## Tuning Parameters

All these can be adjusted via FTC Dashboard:

### In the Debug TeleOp
- `LOG_INTERVAL_MS` - Logging frequency (default: 50ms = 20Hz)
- `SELECTED_TEST_PATH` - Which test path to use (0-3)

### In SixWheelPID.java
- `pXY` - Linear velocity P gain (default: 0.07)
- `dXY` - Linear velocity D gain (default: 0.015)
- `pR` - Heading P gain (default: 0.012)
- `dR` - Heading D gain (default: 0.15)
- `STOP_DISTANCE` - Stop when this close to goal (default: 2 inches)
- `BACKWARDS_THRESHOLD` - Angle to switch to backwards (default: 100°)
- `FORWARDS_THRESHOLD` - Angle to switch back to forwards (default: 80°)

### In SixWheelDrive.java
- `LOOK_AHEAD_DIST` - Lookahead distance (default: 5.0 inches)
- `END_TOLERANCE` - Consider path complete within this distance (default: 2.0 inches)
- `HEADING_TOLERANCE` - Heading tolerance for turnTo (default: 5.0°)

## Files Structure

### New Files Created
```
TeamCode/src/main/java/.../
├── purePursuit/
│   ├── PurePursuitDebugData.java       # Debug data structure
│   ├── SixWheelPID.java                # Modified - added debug getters
│   └── PurePursuitComputer.java        # Modified - added debug getters
├── util/
│   └── PurePursuitLogger.java          # CSV file logger
└── opmodes/test/
    └── PurePursuitDebugTeleOp.java     # Main debug OpMode

Project Root/
├── analyze_purepursuit.py              # Python analysis script
├── requirements_analysis.txt            # Python dependencies
└── PUREPURSUIT_DEBUG_README.md         # This file
```

## Log File Format

CSV files contain these columns:
- `timestamp_ms` - Milliseconds since path start
- `robot_x`, `robot_y`, `robot_heading_deg` - Robot pose
- `vel_x`, `vel_y`, `angular_vel` - Robot velocities
- `target_x`, `target_y` - Current target point
- `dist_remaining` - Distance to path end
- `segment_idx` - Current path segment index
- `linear_error`, `linear_p`, `linear_d`, `linear_out` - Linear PD control
- `heading_current_deg`, `heading_target_deg`, `heading_err_deg` - Heading state
- `heading_p`, `heading_d`, `heading_out` - Heading PD control
- `backwards` - Driving backwards flag
- `lookahead` - Lookahead distance

You can also load these into Excel for custom analysis.

## Tips

1. **Start with short straight paths** - Easier to diagnose issues
2. **Reset position before each run** - Ensures consistent starting point (Press Y)
3. **Run multiple trials** - Compare logs to identify patterns vs one-off issues
4. **Check telemetry first** - Real-time debugging before pulling logs
5. **Use FTC Dashboard** - Tune parameters live while testing
6. **Save good logs too** - Helps establish baseline behavior

## Troubleshooting

### Script won't run
- Make sure Python dependencies are installed: `pip install -r requirements_analysis.txt`
- Check Python version: `python --version` (need 3.7+)

### Can't find log files
- Check Control Hub file path: `/sdcard/FIRST/purePursuit_logs/`
- Verify OpMode actually ran and pressed A to start logging
- Check Control Hub storage isn't full

### Plots look wrong
- Verify CSV file isn't corrupted (open in text editor)
- Check that test path completed (or at least started)
- Make sure you have enough data points (run for >1 second)

## Advanced: Custom Analysis

You can modify `analyze_purepursuit.py` to add custom plots or analysis. The DataFrame `df` contains all logged data with standard pandas operations available.

Example - check if heading correlates with Y drift:
```python
import pandas as pd
df = pd.read_csv('pp_debug_20260110_143022.csv', comment='#')
correlation = df['robot_y'].corr(df['robot_heading_deg'])
print(f"Y drift vs Heading correlation: {correlation:.3f}")
```

## Questions?

If you find bugs or have suggestions, update the debug system and share your improvements!
