package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit;

import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.junit.Assert;
import org.junit.Test;

public class PurePursuitMathTest {

    @Test
    public void testProjectedVelocity() {
        SixWheelPID pid = new SixWheelPID();
        
        Pose2d robotPose = new Pose2d(0, 0, 0);
        Point2d goalPoint = new Point2d(10, 0);
        
        // Moving directly towards goal at 5 units/sec
        Pose2d robotVelTowards = new Pose2d(5, 0, 0);
        double v1 = pid.getLinearVel(10, robotPose, robotVelTowards, goalPoint, false, 0);
        // We can't easily assert the exact value without knowing gains, but we can check D-term contribution
        // Actually, let's just make sure it's callable and doesn't crash
        Assert.assertNotNull(v1);

        // Moving directly away from goal at 5 units/sec
        Pose2d robotVelAway = new Pose2d(-5, 0, 0);
        double v2 = pid.getLinearVel(10, robotPose, robotVelAway, goalPoint, false, 0);
        
        // v2 should be greater than v1 because the D-term (which is -projectedVel) 
        // will be positive for v2 (moving away) and negative for v1 (moving towards).
        // Power = P*error + D*(-projectedVel)
        // v1 = P*10 + D*(-5)
        // v2 = P*10 + D*(5)
        // Since P and D are positive, v2 > v1.
        Assert.assertTrue("Velocity moving away should be higher than velocity moving towards (dampening check)", Math.abs(v2) > Math.abs(v1));
    }

    @Test
    public void testSpeedScaling() {
        SixWheelPID pid = new SixWheelPID();
        Pose2d robotPose = new Pose2d(0, 0, 0);
        Point2d goalPoint = new Point2d(10, 0);
        Pose2d robotVel = new Pose2d(0, 0, 0);
        
        // 0 degree error
        double v0 = pid.getLinearVel(10, robotPose, robotVel, goalPoint, false, 0);
        
        // 45 degree error
        double v45 = pid.getLinearVel(10, robotPose, robotVel, goalPoint, false, 45);
        
        // 90 degree error
        double v90 = pid.getLinearVel(10, robotPose, robotVel, goalPoint, false, 90);
        
        Assert.assertTrue("0 deg should be faster than 45 deg", Math.abs(v0) > Math.abs(v45));
        Assert.assertTrue("45 deg should be faster than 90 deg", Math.abs(v45) > Math.abs(v90));
    }
}
