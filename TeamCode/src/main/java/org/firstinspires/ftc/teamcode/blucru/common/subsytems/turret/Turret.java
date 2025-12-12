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
    private PIDController controller;
    private double position;
    private final double TICKS_PER_REV = 8192 * 212.0/35;
    Pose2d goalPose;
    public static double farP = 0.06, farI = 0, farD = 0;
    public static double closeP = 0.07, closeI = 0, closeD = 0.005;
    public static double acceptableError = 0;
    public static double powerClip = 0.8;
    public static double MAX_ANGLE = 100;
    public static double MIN_ANGLE = -100;

    private enum State{
        MANUAL,
        IDLE,
        PID,
        LOCK_ON_GOAL
    }
    private State state;

    public Turret(BluCRServo servoLeft, BluCRServo servoRight, BluEncoder encoder){
        servos = new TurretServos(servoLeft, servoRight);
        this.encoder = encoder;
        controller = new PIDController(farP, farI, farD);
        state = State.IDLE;
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
            case IDLE:
                break;
            case LOCK_ON_GOAL:
                Vector2d target = Globals.mapVector(Globals.shootingGoalLPose.getX(), Globals.shootingGoalLPose.getY());
                Vector2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos().vec();
                Vector2d delta = target.subtractNotInPlace(robotPose);

                double targetAngle = Math.toDegrees(delta.getHeading());
                Globals.telemetry.addData("target overall angle", targetAngle);
                double robotHeading = Math.toDegrees(Robot.getInstance().sixWheelDrivetrain.getPos().getH());
                this.position = targetAngle - robotHeading;
                Globals.telemetry.addData("target turret angle", this.position);
            case PID:
                this.position = Range.clip(this.position, MIN_ANGLE, MAX_ANGLE);
                double currentAngle = getAngle();
                double rotateError = getRotateError(getAngle(), position);
                if (Math.abs(rotateError) < 15){
                    controller.setPID(closeP, closeI, closeD);
                } else {
                    controller.setPID(farP, farI, farD);
                }
                double power = -controller.calculate(rotateError, 0);
                power = Range.clip(power, -powerClip, powerClip);

                if (currentAngle > MAX_ANGLE+3 && power > 0) {
                    power = 0;
                }
                else if (currentAngle < MIN_ANGLE-3 && power < 0) {
                    power = 0;
                }

                Globals.telemetry.addData("Power", power);
                Globals.telemetry.addData("Turret Power", servos.getPower());
                Globals.telemetry.addData("Position", position);
                if (Math.abs(rotateError) > acceptableError) {
                    Globals.telemetry.addData("Rotate Error", rotateError);
                    servos.setPower(power);
                } else {
                    Globals.telemetry.addLine("Here");
                    servos.setPower(0);
                }
                break;

        }

        servos.write();
        encoder.write();
    }

    public void setAngle(double angle){
        this.position = angle;
        state = State.PID;
    }

    public void setPower(double power){
        servos.setPower(power);
        if (state != State.MANUAL) {
            state = State.IDLE;
        }
    }

    public void setFieldCentricPosition(double position, double robotHeading){
        setAngle(position - robotHeading);
    }

    public void lockOnGoal(){
        if (state != State.MANUAL) {
            state = State.LOCK_ON_GOAL;
        }
    }

    public void toggleManual(){
        if(state == State.MANUAL){
            state = State.IDLE;
        }else{
            state = state.MANUAL;
        }
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

    public void updatePID(){
        controller.setPID(farP,farI,farD);
    }

    public void resetEncoder() {
        encoder.reset();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        servos.telemetry();
        encoder.telemetry();
    }

    @Override
    public void reset() {
//        encoder.reset();
    }
}
