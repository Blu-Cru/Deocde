package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluPIDServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class Turret implements BluSubsystem, Subsystem {
    private TurretServos servos;
    private BluEncoder encoder;
    private PDController controller;
    private double position;
    private final double TICKS_PER_REV = 8192;
    Pose2d goalPose;
    private enum State{
        IDLE,
        PID

    }
    private State state;

    public Turret(BluCRServo servoLeft, BluCRServo servoRight, BluEncoder encoder, PDController controller){
        servos = new TurretServos(servoLeft, servoRight);
        this.encoder = encoder;
        this.controller = controller;
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
            case PID:
                servos.setPower(controller.calculate(encoder.getCurrentPos(), position, servos.getPower(), 0));
                break;
        }

        servos.write();
        encoder.write();
    }

    public void setAngle(double angle){
        this.position = angle / 360 * TICKS_PER_REV;
        state = State.PID;
    }

    public void setPower(double power){
        servos.setPower(power);
        state = State.IDLE;
    }

    public void setFieldCentricPosition(double position, double robotHeading){
        setAngle(position - robotHeading);
    }

    public void turnToPointTowardsGoal(Pose2d robotPose){
        double targetAngle = goalPose.vec().subtractNotInPlace(robotPose.vec()).getHeading();

        setFieldCentricPosition(targetAngle, robotPose.getH());
    }


    @Override
    public void telemetry(Telemetry telemetry) {
        servos.telemetry();
        encoder.telemetry();
    }

    @Override
    public void reset() {
    }
}
