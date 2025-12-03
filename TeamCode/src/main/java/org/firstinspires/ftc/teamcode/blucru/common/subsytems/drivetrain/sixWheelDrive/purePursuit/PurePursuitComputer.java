package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;



/**
 * Guide from https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit
 * */
public class PurePursuitComputer {
    private int lastFoundIndex;
    public PurePursuitComputer(){
        lastFoundIndex = 0;
    }

    public void resetLastFoundIndex(){
        lastFoundIndex = 0;
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

    private double findDistBetween2Points(Point2d p1, Point2d p2){
        double deltaX = p2.getX() - p1.getX();
        double deltaY = p2.getY() - p1.getY();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    public Point2d findOptimalGoToPoint(Pose2d robotPose, Point2d[] path, double lookAheadDist){
        boolean foundIntersection = false;
        Point2d goalPoint = null;
        Point2d[][] pointsSols = new Point2d[path.length-1][2];
        for (int i = 0; i < path.length - 1; i++) {
            pointsSols[i] = getLineIntersections(path[i], path[i+1],robotPose, lookAheadDist);
        }

        for (int i=0; i<pointsSols.length - 1; i++){
            Point2d[] sols = pointsSols[i];
            if (sols.length == 1){
                //only 1 val
                if (findDistBetween2Points(sols[0], path[i+1]) >
                        findDistBetween2Points(new Point2d(robotPose.getX(), robotPose.getY()), path[i+1])){
                    //going there would be bad, dont pick it
                    //there should be a better point
                    //setting lastFoundIndex to always be the point ahead in case the robot cant find a point in later sols
                    lastFoundIndex = i + 1;
                } else {
                    lastFoundIndex = i;
                    goalPoint = sols[0];
                    break;
                }
            } else {
                //2 vals
                //find closer point

                Point2d closerPoint = sols[1];

                if (findDistBetween2Points(sols[0], path[i+1]) < findDistBetween2Points(sols[1], path[i+1])){
                    closerPoint = sols[0];
                }

                if (findDistBetween2Points(closerPoint, path[i+1]) >
                        findDistBetween2Points(new Point2d(robotPose.getX(), robotPose.getY()), closerPoint)){
                    //going to point would be bad, dont pick it
                    //there should be a better point
                    //setting lastFoundIndex to always be the point ahead in case the robot cant find a point in later sols
                    lastFoundIndex = i + 1;
                } else {
                    lastFoundIndex = i;
                    goalPoint = closerPoint;
                    break;
                }

            }
        }

        if (goalPoint == null){
            //no goal point chosen, then go to last found index of intersection on path
            goalPoint = path[lastFoundIndex];
        }

        return goalPoint;
    }

    public double getReqAngleVelTowardsTargetPoint(Pose2d robotPose, Point2d goalPoint, double angleVel, SixWheelPID pid){
        return pid.getHeadingVel(robotPose, goalPoint, angleVel);
    }

    public double compute(Point2d[] path, Pose2d robotPose, double angleVel, double lookAheadDist, SixWheelPID pid){
        Point2d goalPoint = findOptimalGoToPoint(robotPose, path, lookAheadDist);

        return getReqAngleVelTowardsTargetPoint(robotPose, goalPoint, angleVel, pid);
    }

    public double[] computeRotAndXY(Point2d[] path, Pose2d robotPose, Pose2d robotVel, double lookAheadDist, SixWheelPID pid){
        Point2d goalPoint = findOptimalGoToPoint(robotPose, path, lookAheadDist);
        Globals.telemetry.addData("Target Point", goalPoint);
        double rot = getReqAngleVelTowardsTargetPoint(robotPose, goalPoint, robotVel.getH(), pid);

        double linear = pid.getLinearVel(robotPose, goalPoint, robotVel);

        return new double[]{linear, rot};
    }

}
