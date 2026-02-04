package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

@Config
public class Turret implements BluSubsystem, Subsystem {

    private TurretServos servos;
    private BluEncoder encoder;
    private PIDController controller;
    Vector2d target;

    private double position;
    private Double lastSetpoint = null;

    private final double TICKS_PER_REV = 8192 * 212.0 / 35;


    public static double kP = 0.02;
    public static double kI = 0.06;
    public static double kD = 0.0014;

    public static double acceptableError = 0.5;
    public static double powerClip = 0.95;

    public static double MAX_ANGLE = 150;
    public static double MIN_ANGLE = -150;

    public static double distFromCenter = 72.35 / 25.4;


    private enum State {
        MANUAL,
        PID,
        LOCK_ON_GOAL
    }

    private State state;


    public Turret(BluCRServo servoLeft, BluCRServo servoRight, BluEncoder encoder) {
        servos = new TurretServos(servoLeft, servoRight);
        this.encoder = encoder;
        controller = new PIDController(kP, kI, kD);
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
        switch (state) {
            case MANUAL:
                break;

            case LOCK_ON_GOAL:
                double turretTargetDeg =
                        getFieldCentricTargetGoalAngle(
                                Robot.getInstance().sixWheelDrivetrain.getPos()
                        );

//                Globals.telemetry.addData("Turret Target (Field)", turretTargetDeg);

                setFieldCentricPositionAutoAim(
                        turretTargetDeg,
                        Math.toDegrees(
                                Robot.getInstance().sixWheelDrivetrain.getPos().getH()
                        ),
                        false
                );

                updateControlLoop();
                break;

            case PID:
                updateControlLoop();
                break;
        }

        servos.write();
        encoder.write();
    }


    public void setAngle(double angle) {
        angle = -angle;
        if (lastSetpoint == null || Math.abs(angle - lastSetpoint) > 1e-6) {
            controller.reset();
            lastSetpoint = angle;
        }
        position = angle;
        state = State.PID;
    }

    public void setAngle(double angle, boolean switchState) {
        position = angle;
        if (switchState) {
            state = State.PID;
        }
    }

    public void setPower(double power) {
        servos.setPower(power);
        state = State.MANUAL;
    }

    public void setFieldCentricPositionAutoAim(double targetHeading, double robotHeading, boolean switchState) {
        setAngle(180 - targetHeading - robotHeading, switchState);
    }

    public void setFieldCentricPosition(double targetHeading, double robotHeading, double desiredHeading, boolean switchState) {
        setAngle(270-targetHeading+robotHeading-desiredHeading, switchState);
    }



    public void lockOnGoal() {
        state = State.LOCK_ON_GOAL;
    }

    public void toggleManual() {
        if (state != State.MANUAL) {
            state = State.MANUAL;
        } else {
            position = 0;
            state = State.PID;
        }
    }

    public boolean isManual() {
        return state == State.MANUAL;
    }

    public void updatePID() {
        controller.setPID(kP, kI, kD);
    }

    public void updateControlLoop() {
        updatePID();

        while (position > 180) position -= 360;
        while (position <= -180) position += 360;

        position = Range.clip(position, MIN_ANGLE, MAX_ANGLE);

        double currentAngle = getAngle();
        double error = position - currentAngle;

        double power = controller.calculate(currentAngle, position);
        power = Range.clip(power, -powerClip, powerClip);

        // software safety limits
        if (currentAngle > MAX_ANGLE + 3 && power > 0) power = 0;
        if (currentAngle < MIN_ANGLE - 3 && power < 0) power = 0;

//        Globals.telemetry.addData("Turret Angle", currentAngle);
//        Globals.telemetry.addData("Turret Target", position);
//        Globals.telemetry.addData("PID Error", error);
//        Globals.telemetry.addData("PID Output", power);

        if (Math.abs(error) > acceptableError) {
            servos.setPower(power);
        } else {
            servos.setPower(0);
//            Globals.telemetry.addLine("Turret At Setpoint");
        }
    }


    public double getAngle() {
        return encoder.getCurrentPos() * (360.0 / TICKS_PER_REV);
    }

    public double getEncoderPos() {
        return encoder.getCurrentPos();
    }

    public double getPower() {
        return servos.getPower();
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public double getFieldCentricTargetGoalAngle(Pose2d robotPose) {
        if(Globals.alliance == Alliance.RED){
//            target = Globals.turretTargetRedPose;
            target = new Vector2d(Globals.turretTargetRedX, Globals.turretTargetRedY);
        }else{
//            target = Globals.turretTargetBluePose;
            target = new Vector2d(Globals.turretTargetBlueX, Globals.turretTargetBlueY);
        }

        Vector2d robotVec = robotPose.vec();
        double robotHeadingDeg = Math.toDegrees(robotPose.getH());

        double turretCenterX =
                robotVec.getX() - distFromCenter * Math.cos(Math.toRadians(robotHeadingDeg));
        double turretCenterY =
                robotVec.getY() - distFromCenter * Math.sin(Math.toRadians(robotHeadingDeg));

        double dx = target.getX() - turretCenterX;
        double dy = target.getY() - turretCenterY;

        return Math.toDegrees(Math.atan2(dx, dy)) - 90;
    }


    @Override
    public void telemetry(Telemetry telemetry) {
//        servos.telemetry();
//        encoder.telemetry();
//        telemetry.addData("Turret State", state);
    }

    @Override
    public void reset() {
        // encoder.reset();
    }
    public double getTargetPosition(){
        return position;
    }
}
