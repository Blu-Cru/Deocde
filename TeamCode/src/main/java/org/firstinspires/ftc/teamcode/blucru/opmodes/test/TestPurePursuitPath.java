package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;

public class TestPurePursuitPath extends SixWheelPIDPathBuilder {
    Point2d[] pointsOnPath = {new Point2d(30, 0)};
    public TestPurePursuitPath(){
        super();
        this.addMappedPurePursuitPath(pointsOnPath, 5000);
    }
}
