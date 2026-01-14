package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

/**
 * Simple test utility to verify turret field-centric angle calculations
 *
 * Usage:
 * 1. Modify the test cases in main()
 * 2. Run this class
 * 3. Check if the calculated angles make sense
 */
public class TurretAngleCalculator {

    public static void calculateFieldCentricTurretAngle(double robotX, double robotY, double robotHeadingDeg){
        Turret turret = new Turret(null, null, null);
        double turretAngle =  turret.getFieldCentricTargetGoalAngle(new Pose2d(robotX, robotY, robotHeadingDeg), new Pose2d(0,0,0))[0];
        System.out.println(String.format("For Robot Pose (%f, %f, %f), Turrent Target Angle is %f", robotX, robotY, robotHeadingDeg, turretAngle));
    }


    /**
     * Test with multiple robot poses
     */
    public static void main(String[] args) {
        System.out.println();
        System.out.println("╔════════════════════════════════════════╗");
        System.out.println("║   TURRET ANGLE CALCULATOR TESTS        ║");
        System.out.println("╚════════════════════════════════════════╝");

        // Test Red alliance goal
        Globals.setAlliance(Alliance.RED);
        System.out.println("Running RED alliance test scenarios...\n");

        // Test Case 1: Robot facing right (0°), directly to the left of target
        System.out.println("═══ TEST 1: Robot left of target, facing right ═══");
        calculateFieldCentricTurretAngle(50, 72, 0);

        // Test Case 2: Robot facing up (90°), directly below target
        System.out.println("═══ TEST 2: Robot below target, facing up ═══");
        calculateFieldCentricTurretAngle(72, 50, 90);

        // Test Case 3: Robot facing target at 45° angle
        System.out.println("═══ TEST 3: Robot at 45° angle to target ═══");
        calculateFieldCentricTurretAngle(50, 50, 45);

        // Test Case 4: Robot facing away from target
        System.out.println("═══ TEST 4: Robot facing away from target ═══");
        calculateFieldCentricTurretAngle(50, 72, 180);

        // Test Case 5: Robot at origin, target at (10, 10)
        System.out.println("═══ TEST 5: Robot at origin, facing right ═══");
        calculateFieldCentricTurretAngle(0, 0, 0);

        System.out.println();
        System.out.println("╔════════════════════════════════════════╗");
        System.out.println("║         TESTS COMPLETE                 ║");
        System.out.println("╚════════════════════════════════════════╝");
    }
}
