# Pure Pursuit Implementation Fixes

The current Pure Pursuit implementation has several fundamental issues that affect its reliability and performance. This plan outlines fixes for the D-term calculation, distance-along-path calculation, and lookahead logic.

## User Review Required

> [!IMPORTANT]
> The current system uses a "Position-to-Power" PD controller. For true Pure Pursuit, we would ideally have a "Velocity Controller" (PID on motor encoders) underneath. This plan improves the current "Position-to-Power" logic but does not add a motor-level velocity controller as it would require significant hardware-specific tuning.

## Proposed Changes

### Drivetrain Subsystem

#### [MODIFY] [SixWheelPID.java](file:///c:/Users/dong_/source/repos/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/SixWheelPID.java)

- Fix the linear D-term calculation to use the projected velocity towards the goal instead of raw robot speed. This correctly dampens movement even when moving sideways or away.
- Improve the angular D-term to use the true derivative of the heading error.

---

#### [MODIFY] [PurePursuitComputer.java](file:///c:/Users/dong_/source/repos/Deocde/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/PurePursuitComputer.java)

- Simplify and robustify the `dist` calculation (distance along path).
- Refine the `getLineIntersections` and `findOptimalGoToPoint` logic to more reliably pick the point furthest along the path.
- Remove redundant or fragile checks that might cause segment skipping.

## Verification Plan

### Automated Tests
- Since this is a physical robot control system, unit tests for the math functions will be added to verify logic without a robot.
- [NEW] `PurePursuitMathTest.java` to test:
    - `getLineIntersections` with various robot poses and segments.
    - `findOptimalGoToPoint` behavior on edge cases (at end of path, off-path).
    - `SixWheelPID` velocity projections.

### Manual Verification
- The user should run `PurePursuitTuner` or `PurePursuitDebugTeleOp` to observe the robot's behavior.
- Use FTC Dashboard to monitor `Target Point`, `Dist to End`, and `Powers` to ensure they are smooth and logical.
- Test with both forward and backward driving paths.
