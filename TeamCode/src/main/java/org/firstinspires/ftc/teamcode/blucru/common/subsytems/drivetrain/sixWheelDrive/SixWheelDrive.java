package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit.PurePursuitComputer;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit.SixWheelPID;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

@Config
public class SixWheelDrive extends SixWheelDriveBase implements Subsystem {
    private double drivePower;
    private Point2d[] path;
    private int currentPathIndex;
    private PurePursuitComputer computer;
    private final double LOOK_AHEAD_DIST = 5;
    private SixWheelPID pid;

    // Naive Controller Fields
    private com.arcrobotics.ftclib.controller.PIDController distanceController;
    private com.arcrobotics.ftclib.controller.PIDController angleController;
    public static double kP_dist = 0.1, kD_dist = 0;
    public static double kP_angle = 1.0, kD_angle = 0;

    public SixWheelDrive() {
        super();
        drivePower = 1;
        path = null;
        computer = new PurePursuitComputer();
        pid = new SixWheelPID();

        distanceController = new com.arcrobotics.ftclib.controller.PIDController(kP_dist, 0, kD_dist);
        angleController = new com.arcrobotics.ftclib.controller.PIDController(kP_angle, 0, kD_angle);
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
                if (path != null && currentPathIndex < path.length) {
                    Point2d target = path[currentPathIndex];
                    Pose2d currentPose = localizer.getPose();

                    double xError = target.getX() - currentPose.getX();
                    double yError = target.getY() - currentPose.getY();
                    double theta = Math.atan2(yError, xError);

                    double distance = Math.hypot(xError, yError);

                    // Check if reached target
                    if (distance < 2.0) { // Tolerance of 2 inches
                        currentPathIndex++;
                        if (currentPathIndex >= path.length) {
                            switchToIdle();
                            break;
                        }
                        // Update target
                        target = path[currentPathIndex];
                        xError = target.getX() - currentPose.getX();
                        yError = target.getY() - currentPose.getY();
                        theta = Math.atan2(yError, xError);
                        distance = Math.hypot(xError, yError);
                    }

                    // Angle error wrapping
                    double angleError = theta - currentPose.getH();
                    while (angleError > Math.PI)
                        angleError -= 2 * Math.PI;
                    while (angleError <= -Math.PI)
                        angleError += 2 * Math.PI;

                    // Differential Drive Controller with Cosine Trick
                    // Drives forward while turning towards target, then stops forward once close
                    double f = 0;
                    double t = 0;
                    
                    if (distance < 2.0) {
                        // Within distance threshold: focus on final angle
                        f = 0;
                        t = angleController.calculate(currentPose.getH(), theta);
                    } else {
                        // Far from target: drive to position while turning
                        f = distanceController.calculate(0, distance);
                        t = angleController.calculate(currentPose.getH(), theta);
                        
                        // Cosine trick: reduce forward power when angle error is high
                        double clippedAngleError = Math.max(-Math.PI/2, Math.min(Math.PI/2, angleError));
                        f *= Math.cos(clippedAngleError);
                    }

                    drive(f, t);
                } else {
                    switchToIdle();
                }
                break;
            case TELE_DRIVE:
                break;
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
        computer.resetLastFoundIndex(); // Legacy computer reset, keeping just in case
        this.currentPathIndex = 0;
        dtState = State.PID;
    }

    public void followPathNaive(Point2d[] path) {
        this.path = path;
        this.currentPathIndex = 0;
        dtState = State.PID;
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

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry(telemetry);
        if (dtState == State.PID) {
            telemetry.addData("Path Index", currentPathIndex);
            telemetry.addData("Dist Error", distanceController.getPositionError());
            telemetry.addData("Angle Error", angleController.getPositionError());
        }
    }

    public void updatePIDConstants() {
        if (distanceController.getP() != kP_dist || distanceController.getD() != kD_dist) {
            distanceController.setPID(kP_dist, 0, kD_dist);
        }
        if (angleController.getP() != kP_angle || angleController.getD() != kD_angle) {
            angleController.setPID(kP_angle, 0, kD_angle);
        }
    }

    public double cubicScaling(double value) {
        return 64 / 27.0 * value * value * value;
    }
}
