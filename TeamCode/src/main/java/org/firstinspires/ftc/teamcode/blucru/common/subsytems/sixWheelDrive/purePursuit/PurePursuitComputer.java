package org.firstinspires.ftc.teamcode.blucru.common.subsytems.sixWheelDrive.purePursuit;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class PurePursuitComputer {
    double[][] points;
    double lookAheadDist;

    public PurePursuitComputer(double[][] points, double lookAheadDist){
        this.points = points;
        this.lookAheadDist = lookAheadDist;
    }

}
