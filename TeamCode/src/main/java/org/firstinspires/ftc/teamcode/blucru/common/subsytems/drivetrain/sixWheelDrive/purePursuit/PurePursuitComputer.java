package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

/**
 * Guide from
 * https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit
 */
public class PurePursuitComputer {
    private int lastFoundIndex;
    private double dist;
    double totalPathLength = -1;
    Point2d[][] pointsSols;
    public double vMax = 60, aAccel = 40, aDecel = 40; // Default values, should be tuned

    public PurePursuitComputer() {
        lastFoundIndex = 0;
    }

    public PurePursuitComputer(double vMax, double aAccel, double aDecel) {
        this();
        this.vMax = vMax;
        this.aAccel = aAccel;
        this.aDecel = aDecel;
    }

    public void setMotionProfileConstraints(double vMax, double aAccel, double aDecel) {
        this.vMax = vMax;
        this.aAccel = aAccel;
        this.aDecel = aDecel;
    }

    public void resetLastFoundIndex() {
        lastFoundIndex = 0;
        totalPathLength = -1;
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

        if (findDistBetween2Points(new Point2d(robotPose.getX(), robotPose.getY()),
                path[path.length - 1]) < lookAheadDist) {
            return path[path.length - 1];
        }

        boolean foundIntersection = false;
        Point2d goalPoint = null;
        if (pointsSols == null || pointsSols.length != path.length - 1) {
            pointsSols = new Point2d[path.length - 1][2];
        }

        for (int i = 0; i < path.length - 1; i++) {
            // Optimization: only calculate if we are near or haven't passed
            // Actually for pure pursuit we usually check all or from last index.
            // Keeping original logic but reusing array.
            if (i >= lastFoundIndex)
                pointsSols[i] = getLineIntersections(path[i], path[i + 1], robotPose, lookAheadDist);
            else
                pointsSols[i] = new Point2d[0]; // fast clear
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
                    // lastFoundIndex = i + 1; // Don't skip ahead aggressively
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
                    closerPoint = sols[0];
                }

                if (findDistBetween2Points(closerPoint, path[i + 1]) > findDistBetween2Points(
                        new Point2d(robotPose.getX(), robotPose.getY()), path[i + 1])) {
                    // going to point would be bad, dont pick it
                    // lastFoundIndex = i + 1;
                } else {
                    lastFoundIndex = i;
                    goalPoint = closerPoint;
                    break;
                }

            }
        }

        if (goalPoint == null) {
            // no goal point chosen, then go to last found index of intersection on path
            Globals.telemetry.addLine("No goal point set");
            goalPoint = path[lastFoundIndex];

        }

        dist = findDistBetween2Points(new Point2d(robotPose.getX(), robotPose.getY()), path[lastFoundIndex]);
        return goalPoint;
    }

    public double getReqAngleVelTowardsTargetPoint(Pose2d robotPose, Point2d goalPoint, double angleVel,
            SixWheelPID pid) {
        return pid.getHeadingVel(robotPose, goalPoint, angleVel);
    }

    public double compute(Point2d[] path, Pose2d robotPose, double angleVel, double lookAheadDist, SixWheelPID pid) {
        Point2d goalPoint = findOptimalGoToPoint(robotPose, path, lookAheadDist);

        return getReqAngleVelTowardsTargetPoint(robotPose, goalPoint, angleVel, pid);
    }

    public double[] computeRotAndXY(Point2d[] path, Pose2d robotPose, Pose2d robotVel, double lookAheadDist,
            SixWheelPID pid) {

        // Calculate total path length if not already done
        if (totalPathLength == -1) {
            totalPathLength = 0;
            for (int i = 0; i < path.length - 1; i++) {
                totalPathLength += findDistBetween2Points(path[i], path[i + 1]);
            }
        }

        Point2d goalPoint = findOptimalGoToPoint(robotPose, path, lookAheadDist);
        Globals.telemetry.addData("Target Point", goalPoint);
        double rot = getReqAngleVelTowardsTargetPoint(robotPose, goalPoint, robotVel.getH(), pid);

        // --- Motion Profiling ---
        // 1. Calculate distance traveled along the path
        // We can estimate this by summing segments up to lastFoundIndex
        // and adding the distance from path[lastFoundIndex] to finding the robot's
        // projection.
        // For simplicity, we'll use: (sum of segments 0 to lastFoundIndex) +
        // dist(path[lastFoundIndex], robot)
        // Wait, 'lastFoundIndex' is the segment index where intersection was found.

        double distTraveled = 0;
        for (int i = 0; i < lastFoundIndex; i++) {
            distTraveled += findDistBetween2Points(path[i], path[i + 1]);
        }
        // Add projection on current segment? Or just distance to robot?
        // Let's use distance to the goalPoint from the start of the segment?
        // Actually, pure pursuit lookahead is ahead of the robot.
        // Robot position is approximately: distTraveled + dist(path[lastFoundIndex],
        // robot)
        // But robot might be off path.
        // Better approximation: Total Length - Distance to End.

        // Distance from goalPoint to End
        double distFromGoalToEnd = 0;
        // dist from goalPoint to path[lastFoundIndex+1]
        distFromGoalToEnd += findDistBetween2Points(goalPoint, path[lastFoundIndex + 1]);
        // sum remaining segments
        for (int i = lastFoundIndex + 1; i < path.length - 1; i++) {
            distFromGoalToEnd += findDistBetween2Points(path[i], path[i + 1]);
        }

        // Robot is roughly 'lookAheadDist' behind goalPoint (ideally),
        // but physically it is at 'robotPose'.
        // Let's calculate distance REMAINING for the robot.
        // distRemaining = dist(robot, goalPoint) + distFromGoalToEnd?
        // No, dist(robot, goalPoint) is roughly lookahead.
        // Pure Pursuit "cuts corners", so strict path following distance might be
        // wrong.
        // However, for velocity profile, we just need a good estimate of "distance
        // required to stop".

        double currentDistRemaining = distFromGoalToEnd
                + findDistBetween2Points(new Point2d(robotPose.getX(), robotPose.getY()), goalPoint);
        // This includes the lookahead chord, which is fine.

        double currentDistTraveled = totalPathLength - currentDistRemaining;
        if (currentDistTraveled < 0)
            currentDistTraveled = 0; // clamp

        // 2. Calculate Trapezoidal Profile Velocities
        // v_accel = sqrt(2 * a_accel * distTraveled)
        double vAccel = Math.sqrt(2 * aAccel * currentDistTraveled);

        // v_decel = sqrt(2 * a_decel * distRemaining)
        double vDecel = Math.sqrt(2 * aDecel * currentDistRemaining);

        // v_target = min(vMax, vAccel, vDecel)
        double targetVel = Math.min(vMax, Math.min(vAccel, vDecel));

        // Ensure we don't stall with 0 velocity if we are far?
        if (targetVel < 5 && currentDistRemaining > 2)
            targetVel = 5; // Minimum speed to overcome friction if not done
        if (currentDistRemaining < 1)
            targetVel = 0; // Stop at end

        // 3. Get Velocity Command from PID
        // Calculate current linear velocity magnitude
        double currentLinearVel = Math.sqrt(robotVel.getX() * robotVel.getX() + robotVel.getY() * robotVel.getY());

        double linear = pid.getProfiledLinearVel(targetVel, currentLinearVel);

        Globals.telemetry.addData("Rot", rot);
        Globals.telemetry.addData("Linear", linear);
        Globals.telemetry.addData("Target Vel", targetVel);
        Globals.telemetry.addData("Dist Rem", currentDistRemaining);

        return new double[] { linear, rot };
    }

}
