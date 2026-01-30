# Path Following Improvements for Pure Pursuit

The user is experiencing Y-axis deviation and heading misalignment at the end of paths. I will implement Cross-Track Error (CTE) correction and Path Tangent Alignment to address these issues.

## Proposed Changes

### [SixWheelPID](file:///c:/Users/micha/Documents/GitHub/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/SixWheelPID.java)

#### [MODIFY] [SixWheelPID.java](file:///c:/Users/micha/Documents/GitHub/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/SixWheelPID.java)

- Add `Kp_CTE` (Cross-Track Error gain) to the `@Config` parameters.
- Add `TANGENT_BLEND_DISTANCE` to define when the robot should start prioritizing alignment with the path tangent over pointing at the look-ahead point.

### [PurePursuitComputer](file:///c:/Users/micha/Documents/GitHub/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/PurePursuitComputer.java)

#### [MODIFY] [PurePursuitComputer.java](file:///c:/Users/micha/Documents/GitHub/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/PurePursuitComputer.java)

- In `computeRotAndXY()`:
  - Calculate the current segment's tangent heading.
  - Calculate the Cross-Track Error (CTE) of the robot relative to the current segment.
  - Implement a blending logic:
    - As the robot approaches the end of the path (within `TANGENT_BLEND_DISTANCE`), smoothly transition the target heading from "Point at Look-ahead" to "Segment Tangent".
    - Use the CTE to add a corrective term to the heading target, forcing the robot back to the path more aggressively than standard Pure Pursuit when look-ahead distances are large.

## Verification Plan

### Automated Tests
- Build verification using `/build`.

### Manual Verification
- Run "Pure Pursuit Tuner" Stage 1.
- Introduce a manual Y deviation and observe if the robot corrects back to Y=0 more quickly.
- Verify that at the end of the straight path, the robot is perfectly aligned with the path's heading (e.g., θ=0).
- Test on curved paths (Stage 2) to ensure the tangent blending doesn't cause instability.
