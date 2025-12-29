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
    private Double targetHeading; // Target heading in degrees, null if no heading control
    private PurePursuitComputer computer;
    private final double LOOK_AHEAD_DIST = 5;
    public static double END_TOLERANCE = 2.0;
    private SixWheelPID pid;
    public SixWheelDrive(){
        super();
        drivePower = 1;
        path = null;
        targetHeading = null;
        computer = new PurePursuitComputer();
        pid = new SixWheelPID();
    }

    @Override
    public void init(){
        super.init();
    }

    public void read(){
        super.read();
    }

    public void write(){
        switch(dtState){
            case IDLE:
                break;
            case PID:
                // Check if we're close enough to the end point
                if (path != null && path.length > 0) {
                    Point2d endPoint = path[path.length - 1];
                    double distToEnd = Math.sqrt(
                        Math.pow(localizer.getPose().getX() - endPoint.getX(), 2) +
                        Math.pow(localizer.getPose().getY() - endPoint.getY(), 2)
                    );

                    if (distToEnd < END_TOLERANCE) {
                        // Close enough - stop
                        switchToIdle();
                        break;
                    }
                }

                double[] powers = computer.computeRotAndXY(path,localizer.getPose(), localizer.getVel(), LOOK_AHEAD_DIST, pid, targetHeading);
                drive(-powers[0], -powers[1]);
                break;
            case TELE_DRIVE:
                break;
        }

        super.write();
    }

    public void teleDrive(Gamepad g1, double tol){
        double x = cubicScaling(g1.left_stick_y);
        double r = cubicScaling(g1.right_stick_x);

        if (Math.abs(x) <= tol){
            x = 0;
        }
        if (Math.abs(r) <= tol){
            r = 0;
        }

        if (x == 0 && r == 0){
            if (dtState == State.PID){
                //in pid
            } else {
                //either stopped driving or idle alr
                dtState = State.IDLE;
                drive(0,0);
            }
        } else {
            dtState = State.TELE_DRIVE;
            drive(x,r);
        }

    }

    public void setDrivePower(double power){
        this.drivePower = power;
    }
    public double getDrivePower(){
        return drivePower;
    }
    public void followPath(Point2d[] path){
        this.path = path;
        this.targetHeading = null; // No heading control
        computer.resetLastFoundIndex();
        pid.resetBackwardsDrivingState();
        dtState = State.PID;
    }

    public void followPath(Point2d[] path, Double targetHeading){
        this.path = path;
        this.targetHeading = targetHeading;
        computer.resetLastFoundIndex();
        pid.resetBackwardsDrivingState();
        dtState = State.PID;
    }

    public void switchToIdle(){
        drive(0,0);
        dtState = State.IDLE;
    }

    public void setPosition(Pose2d pose){
        localizer.setPosition(pose);
    }
    public void setXY(Vector2d xy){
        localizer.setPosition(xy.getX(), xy.getY(), localizer.getHeading());
    }

    public void setHeading(double heading){
        localizer.setHeading(heading);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        super.telemetry(telemetry);
    }

    public double cubicScaling(double value){
        return 64 / 27.0 * value * value * value;
    }
}
