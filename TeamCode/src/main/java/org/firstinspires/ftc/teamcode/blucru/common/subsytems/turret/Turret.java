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
    public static double farP = 0.8, farI = 0, farD = 0;
    public static double closeP = 0.2, closeI = 0, closeD = 0.5;
    public static double acceptableError = 0;
    public static double powerClip = 1;

    private enum State{
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
            case IDLE:
                break;
            case LOCK_ON_GOAL:
                Vector2d target = Globals.mapVector(Globals.shootingGoalLPose.getX(), Globals.shootingGoalLPose.getY());
                Vector2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos().vec();
                Vector2d delta = robotPose.subtractNotInPlace(target);

                double robotHeading = Math.toDegrees(Robot.getInstance().sixWheelDrivetrain.getPos().getH());
                this.position = delta.getHeading() - robotHeading;
            case PID:
                double rotateError = getRotateError(getAngle(), position);
                if (Math.abs(rotateError) < 15){
                    controller.setPID(closeP, closeI, closeD);
                } else {
                    controller.setPID(farP, farI, farD);
                }
                double power = -controller.calculate(rotateError, 0);
                power = Range.clip(power, -powerClip, powerClip);
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
        state = State.IDLE;
    }

    public void setFieldCentricPosition(double position, double robotHeading){
        setAngle(position - robotHeading);
    }

    public void lockOnGoal(){
        state = State.LOCK_ON_GOAL;
    }

    public double getRotateError(double currAngle, double targetAngle){

        double delta = targetAngle-currAngle;

        while (delta >= 180) {
            delta -= 360;
        }
        while (delta < -180){
            delta += 360;
        }

        return delta;
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
