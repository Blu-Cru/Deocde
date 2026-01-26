# Combining PurePursuit Modifications

The goal is to merge the improvements from the current branch (`purepursuit-jason-refinement-1`) and Michael's branch (`Michael-purepursuit-1`). These changes are complementary and will result in a more robust path-following system.

## Proposed Changes

### [Component Name] Path Following & PID
The changes will be combined into a new branch: `combined-purepursuit-refinement`.

#### [MODIFY] [PurePursuitComputer.java](file:///TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/PurePursuitComputer.java)
- Incorporate Michael's **Cross-Track Error (CTE)** correction logic.
- Incorporate Michael's **Tangent Blending** for arrivals.
- Retain the updated `pid.getLinearVel` call that includes `robotPose` and `goalPoint`.

#### [MODIFY] [SixWheelPID.java](file:///TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/SixWheelPID.java)
- Port the **Projected Velocity** logic from the current branch.
- Port the **Cosine Squared Scaling** logic from the current branch.
- Adopt Michael's tuned PID gains and new path-following constants (`Kp_CTE`, `TANGENT_BLEND_DISTANCE`).
- Include Michael's new `getHeadingVelToTargetTurnTo` method.

#### [NEW] [Michael's Drive Subsystems](file:///TeamCode/src/main/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/)
- Bring in `SixWheelDrive.java` and `SixWheelDriveBase.java` from Michael's branch.
- Bring in `PurePursuitTuner.java` from Michael's branch.

#### [NEW] [Refined Tests](file:///TeamCode/src/test/java/org/firstinspires/ftc/teamcode/blucru/common/subsytems/drivetrain/sixWheelDrive/purePursuit/PurePursuitMathTest.java)
- Retain the new test file from the current branch to verify the combined logic.

## Verification Plan

### Automated Tests
- Run `PurePursuitMathTest.java` to ensure the core mathematics (distance, projected velocity, CTE) are working as expected.
```bash
./gradlew :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit.PurePursuitMathTest"
```

### Manual Verification
- Use `PurePursuitTuner.java` on the robot to verify the behavior in real-time.
- Observe CTE correction: the robot should stay closer to the path line.
- Observe Tangent Blending: the robot should arrive at the end of the path with the correct final heading.
- Observe Projected Velocity: the robot should handle cornering smoother without unnecessary deceleration.
