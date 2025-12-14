# 🤖 Limelight 3A + MegaTag 2 Setup & Testing Guide

This guide covers the setup, configuration, and testing of the Limelight 3A localization system using MegaTag 2 (MT2) with Pinpoint IMU fusion.

## 📋 Prerequisites
- **Limelight 3A** Camera (FTC Legal)
- **GoBilda Pinpoint** Odometry Computer (for IMU data)
- **Control Hub** (Limelight connected via USB-C)
- **Field Map**: "Into the Deep" (Pre-loaded on Limelight 3A)

---

## 🛠️ Step 1: Implementation Check

The code has already been updated to support MegaTag 2.
- **DriveBase.java**: Instantiates `Pinpoint` and passes it to `LimelightLocalizer`.
- **LimelightLocalizer.java**: handle `getBotpose_MT2()` and fuses IMU data automatically.

**No code changes are needed** unless you changed hardware port names.
- Limelight Name in Config: `"limelight"`
- Pinpoint Name in Config: `"pinpoint"`

---

## ⚙️ Step 2: Limelight Web Interface Setup (CRITICAL)

You **MUST** configure the camera properties for localization to work.

1.  **Connect to Limelight**:
    - Plug Limelight into Control Hub via USB-C.
    - Connect Laptop to Control Hub WiFi.
    - Open browser to `http://172.29.0.1:5801` (or `http://limelight.local:5801`).

2.  **Configure AprilTag Pipeline**:
    - Go to **Pipeline** tab.
    - Ensure **Pipeline 0** is selected.
    - Set **Pipeline Type** to `AprilTag`.
    - **Tracking**:
        - **3D Tracking**: Enabled / "Full 3D".
        - **Family**: Standard FTC tags (usually 36h11).

3.  **Set Camera Pose (REQUIRED for MegaTag 2)**:
    - Go to **Settings** -> **Camera Pose**.
    - Measure the distance from the **center of the robot** to the **camera lens**.
    - Enter values (in meters):
        - **Forward (X)**: Distance forward (+) or backward (-) from center.
        - **Right (Y)**: Distance right (+) or left (-) from center.
        - **Up (Z)**: Height of lens from floor.
        - **Roll/Pitch/Yaw**: Orientation of the camera.
          - *Example*: If camera is facing straight forward level, all are 0.

4.  **Verify Field Map**:
    - Go to **Settings** -> **Field Map**.
    - Ensure **"FTC Into The Deep"** is selected/loaded.

---

## 🚀 Step 3: Deployment & Testing

1.  **Deploy Code**:
    - Connect Control Hub to PC.
    - Run `./gradlew.bat build` or deploy from Android Studio.

2.  **Run the OpMode**:
    - Select your standard TeleOp or `LimelightRelocalizationTest` (if available).
    - **Initialize** the robot. Don't move it yet.

3.  **Check Telemetry (Driver Station)**:
    - Look for **"Limelight Mode"**.
        - ✅ **Correct**: `MegaTag 2 (MT2)` (Means Pinpoint IMU is connected and working).
        - ⚠️ **Fallback**: `MegaTag 1` (Means Pinpoint is not detected; checking connections).
    - Look for **"Limelight Pose"**.
        - Verify X, Y, and Heading match the robot's physical position on the field.

---

## 🧪 Verification Checklist

| Check | Action | Expected Result |
| :--- | :--- | :--- |
| **IMU Fusion** | Rotate robot in place | Limelight Heading should match Pinpoint Heading smoothly. |
| **Localization** | Move robot to a known field coordinate | Telemetry X/Y should match field position (within ~1-2 inches). |
| **Robustness** | Cover camera lens briefly | Pose should remain stable (Pinpoint takes over). |
| **Recovery** | Uncover lens | Localization should snap back to absolute field position immediately. |

---

## 🔧 Troubleshooting

- **Telemetry says "MegaTag 1"**:
  - Check Pinpoint cable connection.
  - Verify "pinpoint" name in Driver Station config.

- **Pose is wildly wrong or mirrored**:
  - Check **Camera Pose** settings in Limelight Web Interface.
  - Verify **Roll/Pitch/Yaw** are correct (especially Yaw if camera is mounted backwards or sideways).

- **"No Data Available"**:
  - Ensure AprilTag pipeline is active.
  - Ensure Field Map matches the current tags being viewed.
