package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret;

import com.arcrobotics.ftclib.command.Subsystem;

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
        PID,
        LOCK_ON_GOAL
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
                servos.setPower(controller.calculate(getRotateError(encoder.getCurrentPos(), position), -servos.getPower()));
                break;
            case LOCK_ON_GOAL:
                setFieldCentricPosition((Globals.alliance == Alliance.BLUE) ? 144 : 36, Math.toDegrees(Robot.getInstance().sixWheelDrivetrain.getPos().getH()));
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

    public void lockOnGoal(){
        state = State.LOCK_ON_GOAL;
    }

    public double getRotateError(double currAngle, double targetAngle){

        double delta = Globals.normalize(targetAngle - currAngle);

        if (delta > 180) {
            delta -= 180;
        } else if (delta < -180){
            delta += 180;
        }

        return delta;
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
