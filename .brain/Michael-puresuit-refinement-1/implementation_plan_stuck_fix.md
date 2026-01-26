# Fix Segment Transition and Verify Heading Units

The robot gets stuck at the end of a segment with "distance remaining 2". This is likely due to the `isDone()` threshold in `PurePursuitSegment` being identical to the `STOP_DISTANCE` in `SixWheelPID` (both are 2.0).

## Proposed Changes

### [PurePursuitSegment](file:///c:/Users/micha/Documents/GitHub/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/pathing/PurePursuitSegment.java)

#### [MODIFY] [PurePursuitSegment.java](file:///c:/Users/micha/Documents/GitHub/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/pathing/PurePursuitSegment.java)

- Increase the `isDone()` distance threshold to **2.5** (currently **2**) to ensure the segment transitions even if the robot stalls slightly before reaching the absolute stop distance of 2.0.

### [PurePursuitComputer](file:///c:/Users/micha/Documents/GitHub/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/PurePursuitComputer.java)

#### [VERIFY] [PurePursuitComputer.java](file:///c:/Users/micha/Documents/GitHub/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/PurePursuitComputer.java)

- Confirm `deltaAngle` is in **degrees**. (Found: `robotHeading = Math.toDegrees(robotPose.getH())` and `targetHeadingDeg` is also in degrees, so `deltaAngle` is degrees).

## Verification Plan

### Automated Tests
- Build verification using `/build`.

### Manual Verification
- Run `PurePursuitAutoPath`.
- Verify that the robot proceeds past the first segment to the next one.
- Verify that the telemetry "Dist" shows a transition once it goes below 2.5.
