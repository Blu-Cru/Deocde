# LimelightLocalizer MegaTag 2 Update

## Summary
Successfully modified `LimelightLocalizer.java` to use **MegaTag 2 (MT2)** for improved localization accuracy. MegaTag 2 fuses IMU data with AprilTag detections to eliminate pose ambiguity and provide more accurate position estimates.

## Changes Made

### 1. **Added MegaTag 2 Support**
- Added new constructor that accepts a `RobotLocalizer` parameter (e.g., Pinpoint) to provide IMU heading data
- Maintained backward compatibility with legacy constructor (uses standard MegaTag 1)

### 2. **Updated `read()` Method**
- **IMU Fusion**: Calls `limelight.updateRobotOrientation()` to provide current heading to Limelight
- **Smart Mode Selection**: 
  - Uses `getBotpose_MT2()` when IMU localizer is available (MegaTag 2)
  - Falls back to `getBotpose()` when no IMU is provided (MegaTag 1)
- **Cleaner Code**: Simplified comments and removed redundant explanations

### 3. **Enhanced Telemetry**
- Added "Limelight Mode" telemetry to show which mode is active:
  - "MegaTag 2 (MT2)" when using IMU fusion
  - "MegaTag 1" when using standard localization

## Usage Examples

### Using MegaTag 2 (Recommended)
```java
// Create a Pinpoint localizer for IMU data
Pinpoint pinpoint = new Pinpoint("pinpoint");

// Create LimelightLocalizer with MegaTag 2 support
LimelightLocalizer limelight = new LimelightLocalizer("limelight", pinpoint);
```

### Using MegaTag 1 (Legacy)
```java
// Create LimelightLocalizer without IMU fusion
LimelightLocalizer limelight = new LimelightLocalizer("limelight");
```

## MegaTag 2 Requirements

Before using MegaTag 2, ensure the following are configured in the Limelight web interface:

1. ✅ **AprilTag Pipeline**: Enable "Full 3D" functionality
2. ✅ **Camera Pose**: Configure camera position/orientation relative to robot center
3. ✅ **Field Map**: Upload `.fmap` file with AprilTag locations for your field
4. ✅ **IMU Data**: Provide IMU heading via `updateRobotOrientation()` (handled automatically)

## Benefits of MegaTag 2

- **Higher Accuracy**: Fuses IMU data with vision for better pose estimates
- **Eliminates Ambiguity**: Resolves pose ambiguity issues that can occur with single AprilTag detections
- **More Robust**: Better performance when only partial AprilTag views are available
- **Continuous Tracking**: Maintains accurate pose even during brief vision losses

## Build Status

✅ **Build Successful** - All changes compiled without errors
- Modified file: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/localization/LimelightLocalizer.java`
- Build time: 26 seconds
- 174 tasks executed successfully

## Next Steps

1. **Configure Limelight**: Ensure all MegaTag 2 requirements are met in the web interface
2. **Upload Field Map**: Get the appropriate `.fmap` file for your competition field
3. **Update OpModes**: Modify your OpModes to use the new constructor with IMU localizer
4. **Test**: Verify MegaTag 2 is working by checking telemetry shows "MegaTag 2 (MT2)"

## Example OpMode Update

```java
// Before (MegaTag 1)
LimelightLocalizer limelight = new LimelightLocalizer("limelight");

// After (MegaTag 2)
Pinpoint pinpoint = new Pinpoint("pinpoint");
LimelightLocalizer limelight = new LimelightLocalizer("limelight", pinpoint);
```
