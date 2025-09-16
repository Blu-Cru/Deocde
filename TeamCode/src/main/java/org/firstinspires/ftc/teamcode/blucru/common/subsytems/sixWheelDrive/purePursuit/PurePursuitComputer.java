package org.firstinspires.ftc.teamcode.blucru.common.subsytems.sixWheelDrive.purePursuit;

import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class PurePursuitComputer {
    double[][] points;
    double lookAheadDist;

    public PurePursuitComputer(double[][] points, double lookAheadDist){
        this.points = points;
        this.lookAheadDist = lookAheadDist;
    }

    public double sgn(double x){
        if (x<0){
            return -1;
        }
        return 1;
    }

    public Point2d[] getLineIntersections(Point2d p1, Point2d p2, Pose2d robotPose, double lookAheadDist){

        double[] p1Shifted = {p1.getX() -robotPose.getX(), p1.getY()-robotPose.getY()};
        double[] p2Shifted = {p2.getX() - robotPose.getX(), p2.getY() - robotPose.getY()};
        //robot pose is now 0,0,h, and because heading doesnt matter for line intersections, the robot is equivalently at 0,0

        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();

        double dr = Math.sqrt(dx*dx + dy*dy);

        double D = p1.getX()*p1.getX() - p2.getY()*p2.getX();

        double discriminant = lookAheadDist * lookAheadDist * dr * dr - D * D;

        if (discriminant < 0){
            return new Point2d[0];
        }

        Point2d sol1 = new Point2d((D * dy + sgn(dy) * dx * Math.sqrt(discriminant))/(dr * dr),
                (-D * dx + Math.abs(dy) * Math.sqrt(discriminant))/(dr * dr));

        Point2d sol2 = new Point2d((D * dy - sgn(dy) * dx * Math.sqrt(discriminant))/(dr * dr),
                (-D * dx - Math.abs(dy) * Math.sqrt(discriminant))/(dr * dr));

        boolean sol1XInRange = Math.min(p1.getX(), p2.getX()) <= sol1.getX() && sol1.getX() <= Math.max(p1.getX(), p2.getX());
        boolean sol1YInRange = Math.min(p1.getY(), p2.getY()) <= sol1.getY() && sol1.getY() <= Math.max(p1.getY(), p2.getY());
        boolean sol2XInRange = Math.min(p1.getX(), p2.getX()) <= sol2.getX() && sol2.getX() <= Math.max(p1.getX(), p2.getX());
        boolean sol2YInRange = Math.min(p1.getY(), p2.getY()) <= sol2.getY() && sol2.getY() <= Math.max(p1.getY(), p2.getY());

        boolean sol1InRange = sol1XInRange && sol1YInRange;
        boolean sol2InRange = sol2XInRange && sol2YInRange;

        if (sol1InRange && sol2InRange){
            return new Point2d[]{sol1, sol2};
        } else if (sol1InRange && !sol2InRange){
            return new Point2d[]{sol1};
        } else if (!sol1InRange && !sol2InRange){
            return new Point2d[0];
        } else {
            return new Point2d[]{sol2};
        }
    }

}
