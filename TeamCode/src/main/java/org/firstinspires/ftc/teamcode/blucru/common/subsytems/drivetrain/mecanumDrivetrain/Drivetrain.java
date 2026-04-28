package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain;

import com.seattlesolvers.solverslib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.control.DriveKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.control.DrivePID;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.control.TurnToPointKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

public class Drivetrain extends DriveBase implements Subsystem {
    enum State{
        IDLE,
        PID
    }
    //default to field centric
    public boolean fieldCentric = true;
    //default power to 1
    double drivePower = 1;

    //TODO TUNE THIS IF NECESSARY
    final double MOVEMENT_TOLERANCE = 0.5;
    State currState;
    public DrivePID pid;
    TurnToPointKinematics turnPointKinematics;
    //for heading lock
    boolean lastTurning, lastTranslating;

    public Drivetrain(){
        super();
        currState = State.IDLE;
        pid = new DrivePID();

        lastTranslating = false;
        lastTurning = false;
    }

    @Override
    public void init(){
        super.init();

        drive(new Pose2d(0,0,0));

        pid.setTargetPose(currPose);
    }

    @Override
    public void read() {
        super.read();
    }

    @Override
    public void write() {
        switch(currState){
            case IDLE:
                break;
            case PID:
                driveFieldCentric(DriveKinematics.clip(pid.calculate(xState, yState, heading), drivePower));
                break;
        }

        super.write();
    }

    //driving

    public void teleOpDrive(Gamepad g1){
        currState = State.IDLE;

        double vertical = -g1.left_stick_y;
        double horizontal = g1.left_stick_x;
        //right increases angle, which turns to the left, so negate
        double rotation = -g1.right_stick_x;

        boolean translating = Math.abs(vertical) > MOVEMENT_TOLERANCE || Math.abs(horizontal) > MOVEMENT_TOLERANCE;
        boolean turning = Math.abs(rotation) > MOVEMENT_TOLERANCE;

        if (turning){
            //go normal
            if (fieldCentric){
                driveFieldCentric(new Vector2d(horizontal * drivePower, vertical * drivePower), rotation * drivePower, Globals.alliance);
            } else {
                drive(new Vector2d(horizontal * drivePower, vertical * drivePower), rotation);
            }
        } else if (lastTurning){
            //you were just turning, so turn to decel heading
            pid.headingController.reset();
            driveToHeading(horizontal, vertical, DriveKinematics.getHeadingDecel(heading, headingVel), Globals.alliance);
        } else if (translating && !lastTranslating){
            //you just started translating, turn on heading lock
            driveToHeading(horizontal, vertical, heading, Globals.alliance);
        } else if (!translating){
            //dont drive if ur not translating and not turning
            pid.headingController.reset();
            pid.setTargetHeading(heading);

            drive(new Pose2d(0,0,0));
        } else{
            //if you have been translating, drive to target heading
            driveToHeading(horizontal, vertical, Globals.alliance);
        }

        lastTranslating = translating;
        lastTurning = turning;
    }

    public void pidTo(Pose2d target){
        currState = State.PID;
        pid.setTargetPose(target);
    }

    public void driveToYHeading(double xInput, double ySetPoint, double headingSetPoint){
        pid.setTargetPose(new Pose2d(xInput, ySetPoint, headingSetPoint));
    }

    public void pidTurnToPos(Vector2d pidPoint, Vector2d turnToPoint){
        pid.setTargetPose(pidPoint);
        turnPointKinematics = new TurnToPointKinematics(turnToPoint);

        Vector2d xy = pid.getXY(xState, yState);

        Vector2d curr = new Vector2d(heading, headingVel);
        Vector2d translationHeadingState = turnPointKinematics.getHeadingStateTowardsPoint(currPose, vel);
        Vector2d sp = new Vector2d(translationHeadingState.getX(), -translationHeadingState.getY());

        double rotate = pid.getRotate(curr, sp);
        driveFieldCentric(DriveKinematics.clip(new Pose2d(xy, rotate), drivePower));
    }

    public void teleOpDriveTurnToPos(double x, double y, Vector2d pos, boolean useVel) {
        currState = State.IDLE;
        turnPointKinematics = new TurnToPointKinematics(pos);

        double rotate;

        if (useVel){
            Vector2d curr = new Vector2d(heading, headingVel);
            Vector2d headingState = turnPointKinematics.getHeadingStateTowardsPoint(currPose, vel);
            Vector2d sp = new Vector2d(headingState.getX(), -headingState.getY());

            rotate = pid.getRotate(curr, sp);
        } else {
            pid.setTargetHeading(turnPointKinematics.getHeadingTowardsPoint(currPose));
            rotate = pid.getRotate(headingState);
        }
        driveFieldCentric(new Vector2d(x * drivePower,y * drivePower), rotate);
    }

    public void driveToHeading(double x, double y){
        if (fieldCentric){
            driveFieldCentric(new Vector2d(x * drivePower, y * drivePower), pid.getRotate(headingState));
        } else {
            drive(new Vector2d(x * drivePower, y * drivePower), pid.getRotate(headingState));
        }
    }

    public void driveToHeading(double x, double y, Alliance alliance){
        if (fieldCentric){
            driveFieldCentric(new Vector2d(x * drivePower, y * drivePower), pid.getRotate(headingState), alliance);
        } else {
            drive(new Vector2d(x * drivePower, y * drivePower), pid.getRotate(headingState));
        }
    }

    public void driveToHeading(double x, double y, double targetHeading){
        pid.setTargetHeading(targetHeading);

        driveToHeading(x,y);
    }

    public void driveToHeading(double x, double y, double targetHeading, Alliance alliance){
        pid.setTargetHeading(targetHeading);

        driveToHeading(x,y, alliance);
    }

    public void pidYHeadingMapped(double x, double y, double heading){
        Pose2d mapped = Globals.mapPose(0,y, heading);
        y = mapped.getY();
        heading = mapped.getH();

        fieldCentric = true;
        double yOutput = pid.getXY(xState, new Vector2d(y, yState.getY())).getY();

        driveToHeading(x * Globals.reflect, yOutput, heading);
    }

    public void updatePID(){
        pid.updatePID();
    }

    public void switchToIdle(){
        currState = State.IDLE;
        drive(new Pose2d(0,0,0));
    }

    public Pose2d getStopPose(){return DriveKinematics.getStopPose(currPose, vel);}

    public boolean inRange(double xyTol, double headingTol){
        return pid.inRange(currPose, xyTol, headingTol);
    }

    public boolean inRangeTurnToPoint(Vector2d drivePoint, Vector2d turnToPoint, double xyTol, double headingTol){
        double targetHeading = Math.atan2(turnToPoint.getY() - currPose.getY(),
                turnToPoint.getX() - currPose.getX());

        boolean inRangeH = Math.abs(Globals.normalize(heading) - Globals.normalize(targetHeading)) < headingTol;
        boolean inRangeXY = drivePoint.getMag() < xyTol;

        return inRangeH && inRangeXY;

    }

    public void setHeadingTele(double heading){
        super.setHeading(heading);
        pid.setTargetHeading(heading);
    }
    public void setHeadingAuto(double heading){
        super.setHeading(heading);
        pid.setTargetHeading(heading);
    }

    public void setDrivePower(double power){
        this.drivePower = power;
    }

    @Override
    public void telemetry(Telemetry telemetry){
        telemetry.addData("dt state", currState);
        telemetry.addData("dt power", drivePower);
        telemetry.addData("dt heading", heading);
        Pose2d calc = DriveKinematics.clip(pid.calculate(xState, yState, heading), drivePower);
        telemetry.addData("dt pid xy", "x: " + calc.getX() + ", y: " + calc.getY());
        telemetry.addData("dt pid heading target val", pid.headingController.getSetPoint());
        telemetry.addData("dt pid values xy", "x: " + DrivePID.kPxy + "',y: " + DrivePID.kDxy);
        super.telemetry(telemetry);
    }

    public State getCurrState(){
        return currState;
    }
}
