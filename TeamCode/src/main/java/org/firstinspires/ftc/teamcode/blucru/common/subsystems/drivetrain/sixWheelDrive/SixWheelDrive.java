package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.purePursuit.PurePursuitComputer;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.purePursuit.SixWheelPID;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

@Config
public class SixWheelDrive extends SixWheelDriveBase implements Subsystem {
    private double drivePower;
    private Point2d[] path;
    private Double targetHeading; // Target heading for turnTo command
    private PurePursuitComputer computer;

    // Look-ahead distance: larger = smoother but wider turns, smaller = tighter but
    // jerkier
    public static double LOOK_AHEAD_DIST = 15.0;
    public static double END_TOLERANCE = 0.5;
    public static double HEADING_TOLERANCE = 5.0; // Degrees
    private SixWheelPID pid;
    private double targetX;

    public SixWheelDrive() {
        super();
        drivePower = 1;
        path = null;
        targetHeading = null;
        computer = new PurePursuitComputer();
        pid = new SixWheelPID();
    }

    @Override
    public void init() {
        super.init();
    }

    public void read() {
        super.read();
    }

    public void write() {
        switch (dtState) {
            case IDLE:
                break;
            case PID:
                // Check if we're close enough to the end point
                if (path != null && path.length > 0) {
                    Point2d endPoint = path[path.length - 1];
                    double distToEnd = Math.sqrt(
                            Math.pow(localizer.getPose().getX() - endPoint.getX(), 2) +
                                    Math.pow(localizer.getPose().getY() - endPoint.getY(), 2));

                    if (distToEnd < END_TOLERANCE) {
                        // Close enough - stop
                        switchToIdle();
                        break;
                    }
                }

                double[] powers = computer.computeRotAndXY(path, localizer.getPose(), localizer.getVel(),
                        LOOK_AHEAD_DIST, pid);
                drive(powers[0], -powers[1]); // Negate rotation to match TURN convention
                break;
            case TURN:
                // Turn in place to target heading
                if (targetHeading == null) {
                    switchToIdle();
                    break;
                }

                double robotHeadingDeg = Math.toDegrees(localizer.getPose().getH());
                double headingError = targetHeading - robotHeadingDeg;

                // Normalize to [-180, 180]
                while (headingError > 180) {
                    headingError -= 360;
                }
                while (headingError <= -180) {
                    headingError += 360;
                }

                // Check if we're within tolerance
                if (Math.abs(headingError) < HEADING_TOLERANCE) {
                    switchToIdle();
                    break;
                }

                // Calculate rotation command
                double rotVel = pid.getHeadingVelToTargetTurnTo(localizer.getPose(), targetHeading,
                        localizer.getVel().getH());
                drive(0, -rotVel); // No linear movement, only rotation

                Globals.telemetry.addData("Turn Target", targetHeading);
                Globals.telemetry.addData("Current Heading", robotHeadingDeg);
                Globals.telemetry.addData("Heading Error", headingError);
                break;
            case TELE_DRIVE:
                break;
            case LINE_TO_X:
                drive(-pid.lineToX(targetX, localizer.getPose(), localizer.getVel()), 0);
        }

        super.write();
    }

    public void teleDrive(Gamepad g1, double tol) {
        double x = cubicScaling(g1.left_stick_y);
        double r = cubicScaling(g1.right_stick_x);

        if (Math.abs(x) <= tol) {
            x = 0;
        }
        if (Math.abs(r) <= tol) {
            r = 0;
        }

        if (x == 0 && r == 0) {
            if (dtState == State.PID) {
                // in pid
            } else {
                // either stopped driving or idle alr
                dtState = State.IDLE;
                drive(0, 0);
            }
        } else {
            dtState = State.TELE_DRIVE;
            drive(x, r);
        }

    }

    public void setDrivePower(double power) {
        this.drivePower = power;
    }

    public double getDrivePower() {
        return drivePower;
    }

    public void followPath(Point2d[] path) {
        this.path = path;
        this.targetHeading = null;
        computer.resetLastFoundIndex();
        pid.resetBackwardsDrivingState();
        dtState = State.PID;
    }

    /**
     * Turn in place to a target heading
     * 
     * @param headingDegrees Target heading in degrees (0 = right, 90 = up, 180 =
     *                       left, -90 = down)
     */
    public void turnTo(double headingDegrees) {
        this.targetHeading = headingDegrees;
        this.path = null;
        dtState = State.TURN;
    }

    /**
     * Check if the turn is complete
     * 
     * @return true if robot is within HEADING_TOLERANCE of target heading
     */
    public boolean isTurnComplete() {
        if (dtState != State.TURN || targetHeading == null) {
            return true; // Not turning
        }

        double robotHeadingDeg = Math.toDegrees(localizer.getPose().getH());
        double headingError = targetHeading - robotHeadingDeg;

        // Normalize to [-180, 180]
        while (headingError > 180) {
            headingError -= 360;
        }
        while (headingError <= -180) {
            headingError += 360;
        }

        return Math.abs(headingError) < HEADING_TOLERANCE;
    }

    public void switchToIdle() {
        drive(0, 0);
        dtState = State.IDLE;
    }

    public void setPosition(Pose2d pose) {
        localizer.setPosition(pose);
    }

    public void setXY(Vector2d xy) {
        localizer.setPosition(xy.getX(), xy.getY(), localizer.getHeading());
    }

    public void setHeading(double heading) {
        localizer.setHeading(heading);
    }

    public SixWheelPID getPID() {
        return pid;
    }

    public PurePursuitComputer getPurePursuitComputer() {
        return computer;
    }

    public double getLookAheadDist() {
        return LOOK_AHEAD_DIST;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry(telemetry);
    }

    public double cubicScaling(double value) {
        return 64 / 27.0 * value * value * value;
    }

    public void updatePID() {
        pid.updatePID();
    }

    public void lineToX(double x) {
        targetX = x;
        dtState = State.LINE_TO_X;
    }

    public State getState() {
        return dtState;
    }
}
