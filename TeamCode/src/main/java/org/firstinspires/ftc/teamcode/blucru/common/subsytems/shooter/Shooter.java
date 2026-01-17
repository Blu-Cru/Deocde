package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

@Config
public class Shooter implements BluSubsystem, Subsystem {

    public static double leftP = 0.01,leftI = 0, leftD = 0, leftF = 0.00058;
    public static double middleP = 0.01,middleI = 0, middleD = 0, middleF = 0.00058;
    public static double rightP = 0.01,rightI = 0, rightD = 0, rightF = 0.00058;
    public static double limit = 20;
    public static boolean redAlliance = true; //false  for blueAlliance

    private ShooterPod leftShooter, middleShooter, rightShooter;
    public HoodLeft hoodLeft;
    public HoodMiddle hoodMiddle;
    public HoodRight hoodRight;

    enum State{
        IDLE,
        VELOCITY,
        AUTO_AIM
    }
    private State state;
    double targetVel = 0;
    ShooterVelocityPID pid;
    public Shooter(){
        ShooterVelocityPID leftPID = new ShooterVelocityPID(leftP, leftI, leftD, leftF);
        BluMotorWithEncoder leftShooterMotor = new BluMotorWithEncoder("leftShooter");
        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter = new ShooterPod(leftShooterMotor, new HoodLeft(), leftPID);

        ShooterVelocityPID middlePID = new ShooterVelocityPID(middleP, middleI, middleD, middleF);
        BluMotorWithEncoder middleShooterMotor = new BluMotorWithEncoder("middleShooter");
        middleShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        middleShooter = new ShooterPod(middleShooterMotor, new HoodMiddle(), middlePID);

        ShooterVelocityPID rightPID = new ShooterVelocityPID(rightP, rightI, rightD, rightF);
        BluMotorWithEncoder rightShooterMotor = new BluMotorWithEncoder("rightShooter");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter = new ShooterPod(rightShooterMotor, new HoodRight(), rightPID);

        state = State.IDLE;
    }

    @Override
    public void init() {
        leftShooter.init();
        middleShooter.init();
        rightShooter.init();
    }

    @Override
    public void read() {
        leftShooter.read();
        middleShooter.read();
        rightShooter.read();
    }

    @Override
    public void write() {
        switch (state){
            case IDLE:
                targetVel = 0;
                break;
            case VELOCITY:
                leftShooter.setPower(leftShooter.getPowerToGoToVel());
                middleShooter.setPower(middleShooter.getPowerToGoToVel());
                rightShooter.setPower(rightShooter.getPowerToGoToVel());
                break;
            case AUTO_AIM:
                Vector2d robotToGoal = Globals.shootingGoalRPose.subtractNotInPlace(Robot.getInstance().sixWheelDrivetrain.getPos().vec());
                Vector2d turretToRobot = new Vector2d(-72.35/25.4, 0).rotate(Robot.getInstance().sixWheelDrivetrain.getPos().getH());
                Vector2d leftShooterToRobot = new Vector2d(140,0).rotate(180 + Math.toRadians(-Robot.getInstance().turret.getAngle()));
                Vector2d middleShooterToRobot = new Vector2d(0,0).rotate(180 + Math.toRadians(-Robot.getInstance().turret.getAngle()));
                Vector2d rightShooterToRobot = new Vector2d(-140,0).rotate(180 + Math.toRadians(-Robot.getInstance().turret.getAngle()));
                double leftDist = Math.sqrt(robotToGoal.addNotInPlace(turretToRobot).addNotInPlace(leftShooterToRobot).getDist());
                double middleDist = Math.sqrt(robotToGoal.addNotInPlace(turretToRobot).addNotInPlace(middleShooterToRobot).getDist());
                double rightDist = Math.sqrt(robotToGoal.addNotInPlace(turretToRobot).addNotInPlace(rightShooterToRobot).getDist());
                Globals.telemetry.addData("left distance", leftDist);
                Globals.telemetry.addData("middle distance", middleDist);
                Globals.telemetry.addData("right distance", rightDist);
                double[] leftLerps = ShooterAutoAimInterpolation.interpolateLeft(leftDist);
                double[] middleLerps = ShooterAutoAimInterpolation.interpolateMiddle(middleDist);
                double[] rightLerps = ShooterAutoAimInterpolation.interpolateMiddle(rightDist);

                leftShooter.setVel(leftLerps[0]);
                leftShooter.setHoodAngle(leftLerps[1]);

                middleShooter.setVel(middleLerps[0]);
                middleShooter.setHoodAngle(middleLerps[1]);

                rightShooter.setVel(rightLerps[0]);
                rightShooter.setHoodAngle(rightLerps[1]);
                break;
        }

        leftShooter.write();
        middleShooter.write();
        rightShooter.write();
    }

    public void shoot(double power){
        leftShooter.setPower(power);
        middleShooter.setPower(power);
        rightShooter.setPower(power);
        state = State.IDLE;
    }
    public void idle(){
        leftShooter.idle();
        middleShooter.idle();
        rightShooter.idle();
        state = State.IDLE;
    }
    public void shootWithVelocity(double vel){
        targetVel = vel;
        state = State.VELOCITY;
    }

    public void shootReverseWithVelocity(double vel){
        targetVel = -Math.abs(vel);
        state = State.VELOCITY;
    }

    public void spinReverseSlowPower(){
        leftShooter.reverseIdle();
        middleShooter.reverseIdle();
        rightShooter.reverseIdle();
        state = State.IDLE; // direct power mode
    }

    public double getLeftVel(){
        return leftShooter.getVel();
    }
    public double getMiddleVel(){
        return middleShooter.getVel();
    }
    public double getRightVel(){
        return rightShooter.getVel();
    }
    public double getLeftPower(){
        return leftShooter.getMotorPower();
    }
    public double getMiddlePower(){
        return middleShooter.getMotorPower();
    }
    public double getRightPower(){
        return rightShooter.getMotorPower();
    }

    public void autoAim(){
        this.state = State.AUTO_AIM;
    }
    public void rampDownShooter(){
        this.state = State.IDLE;
        leftShooter.setPower(0);
        middleShooter.setPower(0);
        rightShooter.setPower(0);
    }
    public void setHoodAngle(double angle){
        hoodLeft.setShooterAngle(angle);
        hoodMiddle.setShooterAngle(angle);
        hoodRight.setShooterAngle(angle);
    }
    public void setHoodAngleIndependent(double langle, double mangle, double rangle){
        setLeftHoodAngle(langle);
        setMiddleHoodAngle(mangle);
        setRightHoodAngle(rangle);
    }
    public void setLeftHoodAngle(double angle){
        leftShooter.setHoodAngle(angle);
    }

    public void setMiddleHoodAngle(double angle){
        middleShooter.setHoodAngle(angle);
    }

    public void setRightHoodAngle(double angle){
        rightShooter.setHoodAngle(angle);
    }

    public double getHoodAngle(){
        return leftShooter.getAngle();
    }


    @Override
    public void telemetry(Telemetry telemetry) {
    }

    @Override
    public void reset() {
        leftShooter.reset();
        middleShooter.reset();
        rightShooter.reset();
    }
}