# 🧠 Localization Logic Explained: Limelight + Pinpoint + Kalman

This document explains how our robot knows where it is on the field, even when the camera is blocked.

## 🔄 Hybrid Localization Strategy

Our system fuses two data sources to get the best of both worlds:
1.  **Limelight 3A (MegaTag 2)**: Provides **Absolute Position** (corrects drift).
2.  **GoBilda Pinpoint (Odometry)**: Provides **Continuous Position** (handles fast moves & no-vision).

### ⚙️ How it Works (Logic Flow)

Every loop cycle (`read()` method):

1.  **Step 1: Read Odometry (Always)**
    *   We first read the **Pinpoint**. This gives us a baseline position immediately, even if the camera sees nothing.

2.  **Step 2: Check Vision**
    *   **IF Limelight sees an AprilTag:**
        1.  **Get Raw Pose**: We get the MegaTag 2 pose (which uses IMU data to be stable).
        2.  **Smooth It**: We run the X, Y, and Heading through a **Kalman Filter**. This removes "jitter" or camera noise.
        3.  **Correct Odometry**: We force the Pinpoint to match this filtered vision pose (`imuLocalizer.setPosition(visionPose)`). This kills any drift that accumulated.
        4.  **Result**: `Current Pose = Filtered Vision Pose`.
    *   **IF Limelight sees NOTHING:**
        1.  **Fallback**: We ignore the camera.
        2.  **Result**: `Current Pose = Pinpoint Pose`.
        3.  **Sync Filters**: We update the Kalman filters with the Odometry pose so they don't "jump" when the camera comes back.

---

## 📉 The Kalman Filter & Tuning Guide

The filter smooths out camera noise. You need to tune **Q** and **R**.

*   **R (Measurement Noise)**: Trust in the **Camera**. (Increase = Smoother/Slower).
*   **Q (Process Noise)**: Trust in the **Physics**. (Increase = Faster/Noisier).

### 🛠️ How to Choose: R vs Q?

Since changing R or Q often does the same thing, how do you decide? Use this simple test:

#### 1. The Stationary Test (Tune R)
Park the robot in front of a tag and **don't move**.
*   **Symptom**: Does the `Limelight Pose` numbers jitter or flicker?
*   **Action**: **Increase R**.
    *   Keep increasing R until the numbers are stable when stopped.
    *   *Reason*: This filters out the static sensor noise.

#### 2. The Driving Test (Tune Q)
After R is tuned, drive the robot back and forth quickly.
*   **Symptom**: Does the `Limelight Pose` feel "laggy" or trail behind the real robot?
*   **Action**: **Increase Q**.
    *   Keep increasing Q until the lag is gone.
    *   *Reason*: This tells the filter "my robot is agile, trust the new data more."

### Summary Table

| Problem | Symptom | Solution |
| :--- | :--- | :--- |
| **Too Jittery** | Robot shakes on screen when stopped. | **Increase R** |
| **Too Laggy** | Robot position trails behind real movement. | **Increase Q** |
| **Drift** | Position slides away when stopped. | Check **IMU Calibration** or Camera Offsets. |

**Current Defaults:**
*   `Q = 0.1`
*   `R = 0.3`

---

## 🚨 Troubleshooting

*   **Robot "Teleports"**: This happens if the Vision pose is very different from Odometry. Check that the **Camera Pose** (offsets) in Limelight web interface are correct!
*   **Drift**: If the robot drifts over time, it means it's not seeing tags. Check exposure settings.
