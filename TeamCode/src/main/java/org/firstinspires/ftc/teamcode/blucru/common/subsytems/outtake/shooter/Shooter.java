package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter;

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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class Shooter implements BluSubsystem, Subsystem {

    public static double leftP = 0.0015,leftI = 0, leftD = 0, leftF =
            0.000485
            ;
    public static double middleP = 0.0015,middleI = 0, middleD = 0, middleF = 0.00044;
    public static double rightP = 0.0015,rightI = 0, rightD = 0, rightF = 0.00045;
    public static double limit = 20;
    public static double triggerError = 10;
    public static boolean redAlliance = true; //false  for blueAlliance

    public ShooterPod leftShooter, middleShooter, rightShooter;
    public HoodLeft hoodLeft;
    public HoodMiddle hoodMiddle;
    public HoodRight hoodRight;
    public Vector2d robotToGoal;
    private double shooterDist = 145/25.4;

    public double getBallExitVel(double vel) {
        return ShooterAutoAimInterpolation.getBallExitVel(vel);
    }

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
                double[] dists = new double[3];
                if (Robot.getInstance().turretCam.detectedThisLoop()){
                    dists = tagBasedShooterAutoAim(Robot.getInstance().turretCam.getDetection());
                } else {
                    dists = localizationBasedAutoAim();
                }
                double leftDist = dists[0];
                double middleDist = dists[1];
                double rightDist = dists[2];

                Globals.telemetry.addData("left dist", leftDist);
                Globals.telemetry.addData("middle dist", middleDist);
                Globals.telemetry.addData("right dist", rightDist);


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
    public boolean targetHit(){
        if (Math.abs(leftShooter.getError()) < triggerError && Math.abs(rightShooter.getError()) < triggerError && Math.abs(middleShooter.getError()) < triggerError){
            return true;
        }
        return false;
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
    public double[] localizationBasedAutoAim(){

        //getting the shooter poses
        if (Globals.alliance == Alliance.RED) {
            robotToGoal = Globals.shootingGoalRPose.subtractNotInPlace(Robot.getInstance().sixWheelDrivetrain.getVelPose().vec());
        }else {
            robotToGoal = Globals.shootingGoalLPose.subtractNotInPlace(Robot.getInstance().sixWheelDrivetrain.getVelPose().vec());
        }
        Vector2d turretToRobot = new Vector2d(-72.35/25.4, 0).rotate(Robot.getInstance().sixWheelDrivetrain.getVelPose().getH());
        Vector2d midToGoal = turretToRobot.addNotInPlace(robotToGoal);
        double k = shooterDist / midToGoal.getMag();
        Vector2d midToLeft = midToGoal.rotate(Math.PI/2).scalarMultiplication(k);
        Vector2d midToRight = midToGoal.rotate(-Math.PI/2).scalarMultiplication(k);
        Vector2d leftToGoal = midToGoal.addNotInPlace(midToLeft);
        Vector2d rightToGoal = midToGoal.addNotInPlace(midToRight);
//                Globals.telemetry.addData("left vector to goal", leftToGoal);
//                Globals.telemetry.addData("middle vector to goal", midToGoal);
//                Globals.telemetry.addData("right vector to goal", rightToGoal);

        // find the extra dists (SIGNED)
        Vector2d line = Globals.mapVector(Globals.lineVector.getX(), Globals.lineVector.getY());

        Vector2d a = midToGoal; // vector from mid shooter to goal
        Vector2d b = line;      // reference direction

        double amag = a.getMag();
        double bmag = b.getMag();

// avoid divide-by-zero
        double cosine = 0;
        double sine = 0;
        if (amag > 1e-6 && bmag > 1e-6) {
            double dot = a.getX() * b.getX() + a.getY() * b.getY();
            double cross = a.getX() * b.getY() - a.getY() * b.getX(); // SIGNED

            cosine = dot / (amag * bmag);
            sine   = cross / (amag * bmag); // SIGNED
        }

        double tan = 0;
        if (Math.abs(sine) > 1e-6) {
            tan = cosine / sine; // your same formula, but now with correct sign
        }

// optional safety clamp (prevents blow-ups near sine ~ 0)
        tan = Math.max(-3.0, Math.min(3.0, tan));

        double shooterOffset = shooterDist * tan;
//                Globals.telemetry.addData("Tan", tan);
        double leftDist = leftToGoal.getMag() + shooterOffset;
        double middleDist = midToGoal.getMag();
        double rightDist = rightToGoal.getMag() - shooterOffset;
        return new double[]{leftDist, middleDist, rightDist};
    }

    public double[] tagBasedShooterAutoAim(AprilTagDetection detection){
        double camDistToTag = detection.ftcPose.z;
        double middleShooterDistToTag = camDistToTag + Robot.getInstance().turretCam.getTagDistToMiddleShooter();
        Vector2d middleShooterToTag = Vector2d.polarToCartesian(middleShooterDistToTag, detection.ftcPose.yaw);
        Vector2d a = middleShooterToTag; // vector from mid shooter to goal
        Vector2d b = Globals.mapVector(Globals.lineVector.getX(), Globals.lineVector.getY());;      // reference direction

        double amag = a.getMag();
        double bmag = b.getMag();

// avoid divide-by-zero
        double cosine = 0;
        double sine = 0;
        if (amag > 1e-6 && bmag > 1e-6) {
            double dot = a.getX() * b.getX() + a.getY() * b.getY();
            double cross = a.getX() * b.getY() - a.getY() * b.getX(); // SIGNED

            cosine = dot / (amag * bmag);
            sine   = cross / (amag * bmag); // SIGNED
        }

        double tan = 0;
        if (Math.abs(sine) > 1e-6) {
            tan = cosine / sine; // your same formula, but now with correct sign
        }

// optional safety clamp (prevents blow-ups near sine ~ 0)
        tan = Math.max(-3.0, Math.min(3.0, tan));

        double shooterOffset = shooterDist * tan;
//                Globals.telemetry.addData("Tan", tan);
        double leftDist = middleShooterDistToTag + shooterOffset;
        double middleDist = middleShooterDistToTag;
        double rightDist = middleShooterDistToTag - shooterOffset;
        return new double[]{leftDist, middleDist, rightDist};
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