package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

/**
 * Guide from
 * https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit
 */
@Config
public class PurePursuitComputer {
    private int lastFoundIndex;
    private double dist;

    // Dynamic lookahead parameters - adjust via FTC Dashboard
    public static double MIN_LOOKAHEAD = 2.0;  // Minimum lookahead distance
    public static double LOOKAHEAD_SCALE_FACTOR = 0.6;  // Scale lookahead by this factor of remaining distance

    // Debug tracking
    private Point2d lastGoalPoint = new Point2d(0, 0);
    private double lastDistanceRemaining = 0;
    private double lastEffectiveLookahead = 0;

    public PurePursuitComputer() {
        lastFoundIndex = 0;
    }

    public void resetLastFoundIndex() {
        lastFoundIndex = 0;
    }

    public double sgn(double x) {
        if (x < 0) {
            return -1;
        }
        return 1;
    }

    public Point2d[] getLineIntersections(Point2d p1, Point2d p2, Pose2d robotPose, double lookAheadDist) {

        Point2d p1Shifted = new Point2d(p1.getX() - robotPose.getX(), p1.getY() - robotPose.getY());
        Point2d p2Shifted = new Point2d(p2.getX() - robotPose.getX(), p2.getY() - robotPose.getY());
        // robot pose is now 0,0,h, and because heading doesnt matter for line
        // intersections, the robot is equivalently at 0,0

        double dx = p2Shifted.getX() - p1Shifted.getX();
        double dy = p2Shifted.getY() - p1Shifted.getY();

        double dr = Math.sqrt(dx * dx + dy * dy);

        double D = p1Shifted.getX() * p2Shifted.getY() - p2Shifted.getX() * p1Shifted.getY();

        double discriminant = lookAheadDist * lookAheadDist * dr * dr - D * D;

        if (discriminant < 0) {
            Globals.telemetry.addData("Negative Discriminant, Discriminant Value", discriminant);
            return new Point2d[0];
        }

        Point2d sol1Unshifted = new Point2d((D * dy + sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr),
                (-D * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr));

        Point2d sol2Unshifted = new Point2d((D * dy - sgn(dy) * dx * Math.sqrt(discriminant)) / (dr * dr),
                (-D * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (dr * dr));

        Point2d sol1 = new Point2d(sol1Unshifted.getX() + robotPose.getX(), sol1Unshifted.getY() + robotPose.getY());
        Point2d sol2 = new Point2d(sol2Unshifted.getX() + robotPose.getX(), sol2Unshifted.getY() + robotPose.getY());

        boolean sol1XInRange = Math.min(p1.getX(), p2.getX()) - 0.5 <= sol1.getX()
                && sol1.getX() <= Math.max(p1.getX(), p2.getX()) + 0.5;
        boolean sol1YInRange = Math.min(p1.getY(), p2.getY()) - 0.5 <= sol1.getY()
                && sol1.getY() <= Math.max(p1.getY(), p2.getY()) + 0.5;
        boolean sol2XInRange = Math.min(p1.getX(), p2.getX()) - 0.5 <= sol2.getX()
                && sol2.getX() <= Math.max(p1.getX(), p2.getX()) + 0.5;
        boolean sol2YInRange = Math.min(p1.getY(), p2.getY()) - 0.5 <= sol2.getY()
                && sol2.getY() <= Math.max(p1.getY(), p2.getY()) + 0.5;

        boolean sol1InRange = sol1XInRange && sol1YInRange;
        boolean sol2InRange = sol2XInRange && sol2YInRange;

        if (sol1InRange && sol2InRange) {
            return new Point2d[] { sol1, sol2 };
        } else if (sol1InRange && !sol2InRange) {
            return new Point2d[] { sol1 };
        } else if (!sol1InRange && !sol2InRange) {
            // throw new RuntimeException("Both Sols for Pure Pursuit are Out of Range! Sol
            // 1 is " + sol1 + " and sol 2 is " + sol2 + ". The Robot Pose is " + robotPose
            // + "/n"
            // + "The value for sol1XInRange is " + sol1XInRange + " and the value for
            // sol1YInRange is " + sol1YInRange);
            return new Point2d[0];
        } else {

            return new Point2d[] { sol2 };
        }
    }

    private double findDistBetween2Points(Point2d p1, Point2d p2) {
        double deltaX = p2.getX() - p1.getX();
        double deltaY = p2.getY() - p1.getY();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public Point2d findOptimalGoToPoint(Pose2d robotPose, Point2d[] path, double lookAheadDist) {

        // Calculate distance to goal to enable dynamic lookahead scaling
        double distToGoal = findDistBetween2Points(new Point2d(robotPose.getX(), robotPose.getY()),
                path[path.length - 1]);

        // Scale down lookahead as we approach the end for smoother arrival
        // effectiveLookahead = max(MIN_LOOKAHEAD, min(requestedLookahead, distToGoal * SCALE_FACTOR))
        double effectiveLookahead = Math.max(MIN_LOOKAHEAD,
                                             Math.min(lookAheadDist, distToGoal * LOOKAHEAD_SCALE_FACTOR));
        lastEffectiveLookahead = effectiveLookahead;

        if (distToGoal < effectiveLookahead) {
            dist = distToGoal;
            // Store debug values for early return
            lastGoalPoint = path[path.length - 1];
            lastDistanceRemaining = dist;
            return path[path.length - 1];
        }

        boolean foundIntersection = false;
        Point2d goalPoint = null;
        Point2d[][] pointsSols = new Point2d[path.length - 1][2];
        for (int i = lastFoundIndex; i < path.length - 1; i++) {
            pointsSols[i] = getLineIntersections(path[i], path[i + 1], robotPose, effectiveLookahead);
        }

        for (int i = lastFoundIndex; i < pointsSols.length; i++) {
            Point2d[] sols = pointsSols[i];
            if (sols.length == 0) {
                continue;
            }
            if (sols.length == 1) {
                // only 1 val
                if (findDistBetween2Points(sols[0], path[i + 1]) > findDistBetween2Points(
                        new Point2d(robotPose.getX(), robotPose.getY()), path[i + 1])) {
                    // going there would be bad, dont pick it
                    // there should be a better point
                    // setting lastFoundIndex to always be the point ahead in case the robot cant
                    // find a point in later sols
                    lastFoundIndex = Math.min(i + 1, path.length - 2);
                    Globals.telemetry.addLine("1 sol, and it moves the robot farther away");
                } else {
                    lastFoundIndex = i;
                    goalPoint = sols[0];
                    break;
                }
            } else {
                // 2 vals
                // find closer point

                Point2d closerPoint = sols[1];

                if (findDistBetween2Points(sols[0], path[i + 1]) < findDistBetween2Points(sols[1], path[i + 1])) {
                    Globals.telemetry.addLine("closer sol");
                    closerPoint = sols[0];
                }

                if (findDistBetween2Points(closerPoint, path[i + 1]) > findDistBetween2Points(
                        new Point2d(robotPose.getX(), robotPose.getY()), path[i + 1])) {
                    // going to point would be bad, dont pick it
                    // there should be a better point
                    // setting lastFoundIndex to always be the point ahead in case the robot cant
                    // find a point in later sols
                    Globals.telemetry.addLine("2 sols, and closer one moves the robot farther away");
                    lastFoundIndex = Math.min(i + 1, path.length - 2);
                } else {
                    lastFoundIndex = i;
                    goalPoint = closerPoint;
                    break;
                }

            }
        }

        if (goalPoint == null) {
            // no goal point chosen, then go to the end of the current segment (point ahead)
            Globals.telemetry.addLine("No goal point set");
            // Clamp lastFoundIndex to prevent out of bounds
            if (lastFoundIndex >= path.length - 1) {
                lastFoundIndex = path.length - 2;
            }
            // Go to the end of the current segment, not the start
            goalPoint = path[lastFoundIndex + 1];

        }

        // Calculate distance remaining along the path
        // Start from robot position to goal point, then sum remaining segments
        dist = findDistBetween2Points(new Point2d(robotPose.getX(), robotPose.getY()), goalPoint);
        dist += findDistBetween2Points(goalPoint, path[lastFoundIndex+1]);

        // Add remaining path segments after the current segment
        // Start from the next segment (lastFoundIndex + 1) since goalPoint is already on the current segment
        // This avoids double-counting part of the current segment
        for (int i = lastFoundIndex + 1; i < path.length - 1; i++) {
            dist += findDistBetween2Points(path[i], path[i + 1]);
        }

        // Store debug values
        lastGoalPoint = goalPoint;
        lastDistanceRemaining = dist;

        return goalPoint;
    }

    public double getReqAngleVelTowardsTargetPoint(Pose2d robotPose, Point2d goalPoint, double angleVel,
            SixWheelPID pid, boolean isDrivingBackwards) {
        double dist = findDistBetween2Points(new Point2d(robotPose.getX(), robotPose.getY()), goalPoint);
        if (dist < 3){
            return 0;
        }
        return pid.getHeadingVel(robotPose, goalPoint, angleVel, isDrivingBackwards);
    }

    public double compute(Point2d[] path, Pose2d robotPose, double angleVel, double lookAheadDist, SixWheelPID pid) {
        Point2d goalPoint = findOptimalGoToPoint(robotPose, path, lookAheadDist);
        boolean isDrivingBackwards = pid.shouldDriveBackwards(robotPose, goalPoint);

        return getReqAngleVelTowardsTargetPoint(robotPose, goalPoint, angleVel, pid, isDrivingBackwards);
    }

    public double[] computeRotAndXY(Point2d[] path, Pose2d robotPose, Pose2d robotVel, double lookAheadDist,
            SixWheelPID pid) {
        Point2d goalPoint = findOptimalGoToPoint(robotPose, path, lookAheadDist);
        Globals.telemetry.addData("Target Point", goalPoint);

        // Determine backwards driving once, use for both linear and heading control
        boolean isDrivingBackwards = pid.shouldDriveBackwards(robotPose, goalPoint);

        // Calculate heading error for speed scaling
        double robotHeading = Math.toDegrees(robotPose.getH());
        double turnReq = Math.toDegrees(Math.atan2(goalPoint.getY() - robotPose.getY(), goalPoint.getX() - robotPose.getX()));
        double deltaAngle = turnReq - robotHeading;

        // Normalize to [-180, 180]
        while (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        while (deltaAngle <= -180) {
            deltaAngle += 360;
        }

        // Adjust for backwards driving
        if (isDrivingBackwards) {
            if (deltaAngle > 0) {
                deltaAngle -= 180;
            } else {
                deltaAngle += 180;
            }
        }

        // Both controllers use the same backwards driving decision and heading error
        double linear = pid.getLinearVel(dist, robotVel, isDrivingBackwards, deltaAngle);
        double rot = getReqAngleVelTowardsTargetPoint(robotPose, goalPoint, robotVel.getH(), pid, isDrivingBackwards);

        Globals.telemetry.addData("Rot", rot);
        Globals.telemetry.addData("Linear", linear);
        Globals.telemetry.addData("Dist to End", dist);
        Globals.telemetry.addData("Heading Error", deltaAngle);
        Globals.telemetry.addData("Effective Lookahead", lastEffectiveLookahead);
        Globals.telemetry.addData("Driving Backwards (PP)", isDrivingBackwards);

        return new double[] { linear, rot };
    }

    // Debug getters
    public Point2d getLastGoalPoint() { return lastGoalPoint; }
    public double getLastDistanceRemaining() { return lastDistanceRemaining; }
    public int getLastSegmentIndex() { return lastFoundIndex; }
    public double getLastEffectiveLookahead() { return lastEffectiveLookahead; }

}
