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

## 📉 The Kalman Filter

We added a **Kalman Filter** to smooth out the camera data. Vision data can be "noisy" (inputs jumping around slightly).

*   **Q (Process Noise)**: How much we trust our "prediction" (physics). Higher = Faster reaction.
*   **R (Measurement Noise)**: How much we trust the "sensor" (camera). Higher = More smoothing.

**Current Tuning:**
*   `Q = 0.1`
*   `R = 0.3`
*   *Result*: A smooth, stable position that doesn't jitter, but still reacts relatively quickly to movement.

---

## 🚨 Troubleshooting

*   **Robot "Teleports"**: This happens if the Vision pose is very different from Odometry. Check that the **Camera Pose** (offsets) in Limelight web interface are correct!
*   **Drift**: If the robot drifts over time, it means it's not seeing tags. Check exposure settings.
