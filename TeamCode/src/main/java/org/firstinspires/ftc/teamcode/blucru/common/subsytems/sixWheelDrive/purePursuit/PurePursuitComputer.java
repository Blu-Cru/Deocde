package org.firstinspires.ftc.teamcode.blucru.common.subsytems.sixWheelDrive.purePursuit;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class PurePursuitComputer {
    double[][] points;
    double lookAheadDist;

    public PurePursuitComputer(double[][] points, double lookAheadDist){
        this.points = points;
        this.lookAheadDist = lookAheadDist;
    }

    public void getLineIntersections(double[] p1, double[] p2, Pose2d robotPose){

        double[] p1Shifted = {p1[0]-robotPose.getX(), p1[1]-robotPose.getY()};
        double[] p2Shifted = {p2[0] - robotPose.getX(), p2[1] - robotPose.getY()};
        //robot pose is now 0,0,h, and because heading doesnt matter for line intersections, the robot is equivalently at 0,0

        double dx = p2[0] - p1[0];
        double dy = p2[1] - p1[1];

        double dr = Math.sqrt(dx*dx + dy*dy);

    }

}
