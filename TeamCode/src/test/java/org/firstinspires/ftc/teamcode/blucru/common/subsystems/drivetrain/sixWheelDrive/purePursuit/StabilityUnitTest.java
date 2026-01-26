package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.purePursuit;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.purePursuit.mocks.MockTelemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.junit.Before;
import org.junit.Test;

public class StabilityUnitTest {
    @Before
    public void setup() {
        Globals.telemetry = new MockTelemetry();
    }

    @Test
    public void testHeadingStabilityNearGoal() {
        PurePursuitComputer computer = new PurePursuitComputer();
        SixWheelPID pid = new SixWheelPID();

        // Horizontal path from (0,0) to (10,0)
        Point2d[] path = { new Point2d(0, 0), new Point2d(10, 0) };

        // Case 1: Robot is precisely at the goal
        Pose2d poseAtGoal = new Pose2d(10.0, 0.0, 0.0);
        double[] powers = computer.computeRotAndXY(path, poseAtGoal, new Pose2d(0, 0, 0), 5.0, pid);

        // With current deadzone and stable tangent, rot should be 0 or consistent with
        // segment
        assertEquals("Rotation should be 0 in deadzone", 0.0, powers[1], 0.01);

        // Case 2: Robot overshoots goal by 0.1" (x=10.1)
        Pose2d poseOvershoot = new Pose2d(10.1, 0.0, 0.0);
        double[] overshootPowers = computer.computeRotAndXY(path, poseOvershoot, new Pose2d(0, 0, 0), 5.0, pid);

        // In the old atan2 logic, this would flip the target heading to 180 and spin
        // the robot.
        // With stable tangent, it should stay at 0 (or 0-180 based on direction lock)
        // We check the telemetry message if we could, but here we check 'rot' is
        // small/zero
        assertEquals("Rotation should still be 0 in deadzone even if overshot", 0.0, overshootPowers[1], 0.01);
    }
}
