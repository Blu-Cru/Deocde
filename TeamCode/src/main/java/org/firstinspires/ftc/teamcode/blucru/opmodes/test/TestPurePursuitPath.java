package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;

/**
 * Test path for pure pursuit with heading control
 *
 * Tests multiple scenarios:
 * 1. Straight path without heading control (tests distance calculation)
 * 2. Straight path with heading control to 90 degrees
 * 3. Curved path with heading control to 180 degrees
 * 4. Path with sharp turn and heading control to 0 degrees
 */
public class TestPurePursuitPath extends SixWheelPIDPathBuilder {

    // Test 1: Straight path forward, no heading control (baseline test)
    Point2d[] straightPath = {
        new Point2d(0, 0),
        new Point2d(40, 0)
    };

    // Test 2: Straight path with 90 degree heading
    Point2d[] straightPath90 = {
        new Point2d(40, 0),
        new Point2d(40, 40)
    };

    // Test 3: Curved path with 180 degree heading
    Point2d[] curvedPath = {
        new Point2d(40, 40),
        new Point2d(30, 50),
        new Point2d(20, 55),
        new Point2d(10, 55),
        new Point2d(0, 50)
    };

    // Test 4: Sharp turn with return to 0 degrees
    Point2d[] returnPath = {
        new Point2d(0, 50),
        new Point2d(0, 20),
        new Point2d(0, 0)
    };

    public TestPurePursuitPath(){
        super();

        // Test 1: No heading control (should work as before)
        this.addMappedPurePursuitPath(straightPath, 5000);
        this.waitMilliseconds(500);

        // Test 2: End at 90 degrees (facing up)
        this.addMappedPurePursuitPath(straightPath90, 90.0, 5000);
        this.waitMilliseconds(500);

        // Test 3: End at 180 degrees (facing left)
        this.addMappedPurePursuitPath(curvedPath, 180.0, 5000);
        this.waitMilliseconds(500);

        // Test 4: End at 0 degrees (facing right)
        this.addMappedPurePursuitPath(returnPath, 0.0, 5000);
    }
}
