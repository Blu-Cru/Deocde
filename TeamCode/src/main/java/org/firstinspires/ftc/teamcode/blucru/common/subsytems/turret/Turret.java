package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluPIDServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret implements BluSubsystem, Subsystem {
    private TurretServos servos;
    private BluEncoder encoder;
    private PDController nearController;
    private PDController farController;
    private double position;
    private final double TICKS_PER_REV = 8192 * 212.0/35;
    Pose2d goalPose;
    public static double farP = 0.06, farD = 0;
    public static double closeP = 0.07, closeD = 0.05;
    public static double farVsCloseCutoff = 10;
    public static double acceptableError = 0;
    public static double powerClip = 0.8;
    public static double MAX_ANGLE = 10;
    public static double MIN_ANGLE = -10;
    public static double distFromCenter = 72.35/25.4;

    private enum State{
        MANUAL,
        PID,
        LOCK_ON_GOAL
    }
    private State state;

    public Turret(BluCRServo servoLeft, BluCRServo servoRight, BluEncoder encoder){
        servos = new TurretServos(servoLeft, servoRight);
        this.encoder = encoder;
        nearController = new PDController(closeP, closeD);
        farController = new PDController(farP, farD);
        state = State.MANUAL;
    }

    @Override
    public void init() {
        servos.init();
        encoder.init();
    }

    @Override
    public void read() {
        servos.read();
        encoder.read();
    }

    @Override
    public void write() {

        switch(state){
            case MANUAL:
                break;
            case LOCK_ON_GOAL:
                double turretTargetDeg = getFieldCentricTargetGoalAngle(Robot.getInstance().sixWheelDrivetrain.getPos());
                Globals.telemetry.addData("Turret target", turretTargetDeg);
                turretTargetDeg *= -1;
                setFieldCentricPosition(turretTargetDeg, Math.toDegrees(Robot.getInstance().sixWheelDrivetrain.getPos().getH()), false);
                updateControlLoop();
                break;
            case PID:
                updateControlLoop();
                break;

        }

        servos.write();
        encoder.write();
    }

    public void setAngle(double angle){
        setAngle(angle, true);
    }
    public void setAngle(double angle, boolean switchState){
        this.position = angle;
        if (switchState){
            state = State.PID;
        }
    }

    public void setPower(double power){
        servos.setPower(power);
        state = State.MANUAL;
    }

    public void setFieldCentricPosition(double position, double robotHeading, boolean switchState){
        setAngle(180 - position - robotHeading, switchState);
    }

    public void lockOnGoal(){
        state = State.LOCK_ON_GOAL;
    }

    public void toggleManual(){
        state = State.MANUAL;
    }

    public boolean isManual(){
        return state == State.MANUAL;
    }

    public double getRotateError(double currAngle, double targetAngle){

        return targetAngle-currAngle;
    }

    public double getEncoderPos(){
        return encoder.getCurrentPos();
    }

    public double getPower(){return servos.getPower();}

    public double getAngle(){
        return encoder.getCurrentPos() * (360 / TICKS_PER_REV);
    }

    public void updatePD(){
        nearController.setPD(farP,farD);
        farController.setPD(closeP, closeD);
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public void updateControlLoop(){
        while (position > 180){
            position -= 360;
        }
        while (position <= -180){
            position += 360;
        }
        this.position = Range.clip(this.position, MIN_ANGLE, MAX_ANGLE);
        double currentAngle = getAngle();
        double rotateError = getRotateError(getAngle(), position);
        double power;
        if (Math.abs(rotateError) < farVsCloseCutoff){
            Globals.telemetry.addLine("CLOSE SETTINGS");
            power = nearController.calculate(rotateError, 0);
        } else {
            Globals.telemetry.addLine("FAR SETTINGS");
            power = farController.calculate(rotateError, 0);
        }
        power = Range.clip(power, -powerClip, powerClip);

        if (currentAngle > MAX_ANGLE+3 && power > 0) {
            power = 0;
        }
        else if (currentAngle < MIN_ANGLE-3 && power < 0) {
            power = 0;
        }

        Globals.telemetry.addData("Calculated Power", power);
        Globals.telemetry.addData("Position", position);
        if (Math.abs(rotateError) > acceptableError) {
            Globals.telemetry.addData("Rotate Error", rotateError);
            servos.setPower(power);
        } else {
            Globals.telemetry.addLine("At Set Point");
            servos.setPower(0);
        }
    }

    public double getFieldCentricTargetGoalAngle(Pose2d robotPose){
        Vector2d target = Globals.mapVector(
                Globals.shootingGoalLPose.getX(),
                Globals.shootingGoalLPose.getY()
        );
        Vector2d robotVec = robotPose.vec();

        // robot heading in field frame
        double robotHeadingDeg =
                Math.toDegrees(robotPose.getH());

        // turret center in field frame
        double turretCenterX = robotVec.getX() - distFromCenter * Math.cos(Math.toRadians(robotHeadingDeg));
        double turretCenterY = robotVec.getY() - distFromCenter * Math.sin(Math.toRadians(robotHeadingDeg));
        // vector from robot to target
        double dx = target.getX() - turretCenterX;
        Globals.telemetry.addData("dx", dx);
        double dy = target.getY() - turretCenterY;
        Globals.telemetry.addData("dy", dy);

        // absolute field angle to target
        return Math.toDegrees(Math.atan(Math.abs(dy/dx))) + 90;
    }


    @Override
    public void telemetry(Telemetry telemetry) {
        servos.telemetry();
        encoder.telemetry();
        telemetry.addData("Turret State", state);
    }

    @Override
    public void reset() {
//        encoder.reset();
    }
}
