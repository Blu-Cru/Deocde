package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import android.service.vr.VrListenerService;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

@Config
public class Shooter implements BluSubsystem, Subsystem {

    public static double leftP = 0.0015,leftI = 0, leftD = 0, leftF =
            0.000485
            ;
    public static double middleP = 0.0015,middleI = 0, middleD = 0, middleF = 0.00044;
    public static double rightP = 0.0015,rightI = 0, rightD = 0, rightF = 0.00045;
    public static double limit = 20;
    public static boolean redAlliance = true; //false  for blueAlliance

    public ShooterPod leftShooter, middleShooter, rightShooter;
    public HoodLeft hoodLeft;
    public HoodMiddle hoodMiddle;
    public HoodRight hoodRight;
    public Vector2d robotToGoal;
    private double shooterDist = 145/25.4;

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
        leftShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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

                //getting the shooter poses
                if (Globals.alliance == Alliance.RED) {
                    robotToGoal = Globals.shootingGoalRPose.subtractNotInPlace(Robot.getInstance().sixWheelDrivetrain.getPos().vec());
                }else {
                    robotToGoal = Globals.shootingGoalLPose.subtractNotInPlace(Robot.getInstance().sixWheelDrivetrain.getPos().vec());
                }
                Vector2d turretToRobot = new Vector2d(-72.35/25.4, 0).rotate(Robot.getInstance().sixWheelDrivetrain.getPos().getH());
                Vector2d midToGoal = turretToRobot.addNotInPlace(robotToGoal);
                double k = shooterDist / midToGoal.getMag();
                Vector2d midToLeft = midToGoal.rotate(Math.PI/2).scalarMultiplication(k);
                Vector2d midToRight = midToGoal.rotate(-Math.PI/2).scalarMultiplication(k);
                Vector2d leftToGoal = midToGoal.addNotInPlace(midToLeft);
                Vector2d rightToGoal = midToGoal.addNotInPlace(midToRight);
//                Globals.telemetry.addData("left vector to goal", leftToGoal);
//                Globals.telemetry.addData("middle vector to goal", midToGoal);
//                Globals.telemetry.addData("right vector to goal", rightToGoal);

                //find the extra dists
                double cosine = Vector2d.getCosineOfAngleBetween2Vectors(midToGoal, Globals.mapVector(Globals.lineVector.getX(), Globals.lineVector.getY()));
//                Globals.telemetry.addData("Cosine", cosine);
                double sine = Math.sqrt(1-cosine * cosine);
                double tan;
                try {
                    tan = cosine/sine;
                } catch (Exception e){
//                    Globals.telemetry.addLine("ETHAN WHAT THE HELLIANTE ARE U DOING");
                    tan = 0;
                }
                double shooterOffset = shooterDist * tan;
//                Globals.telemetry.addData("Tan", tan);
                double leftDist = leftToGoal.getMag() + shooterOffset;
                double middleDist = midToGoal.getMag();
                double rightDist = rightToGoal.getMag() - shooterOffset;

//                Globals.telemetry.addData("left dist", leftDist);
//                Globals.telemetry.addData("middle dist", middleDist);
//                Globals.telemetry.addData("right dist", rightDist);


                double[] leftLerps = ShooterAutoAimInterpolation.interpolateLeft(leftDist);
                double[] middleLerps = ShooterAutoAimInterpolation.interpolateMiddle(middleDist);
                double[] rightLerps = ShooterAutoAimInterpolation.interpolateRight(rightDist);

                leftShooter.setVel(leftLerps[0]);
                leftShooter.setHoodAngle(leftLerps[1]);

                middleShooter.setVel(middleLerps[0]);
                middleShooter.setHoodAngle(middleLerps[1]);

                rightShooter.setVel(rightLerps[0]);
                rightShooter.setHoodAngle(rightLerps[1]);

                leftShooter.setPower(leftShooter.getPowerToGoToVel());
                middleShooter.setPower(middleShooter.getPowerToGoToVel());
                rightShooter.setPower(rightShooter.getPowerToGoToVel());
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
        leftShooter.setVel(vel);
        middleShooter.setVel(vel);
        rightShooter.setVel(vel);
        state = State.VELOCITY;
    }

    public void shootWithVelocityIndependent(double leftVel, double middleVel, double rightVel){
        leftShooter.setVel(leftVel);
        middleShooter.setVel(middleVel);
        rightShooter.setVel(rightVel);
        state = State.VELOCITY;
    }

    public void shootReverseWithVelocity(double vel){
        targetVel = -Math.abs(vel);
        leftShooter.setVel(targetVel);
        middleShooter.setVel(targetVel);
        rightShooter.setVel(targetVel);
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