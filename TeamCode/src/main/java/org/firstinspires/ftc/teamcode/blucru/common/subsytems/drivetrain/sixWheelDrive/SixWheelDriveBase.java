package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.RobotLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit.PurePursuitComputer;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit.SixWheelPID;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

import java.util.Arrays;

public class SixWheelDriveBase implements BluSubsystem{

    private BluMotor[] dtMotors;

    RobotLocalizer localizer;
    public enum State{
        IDLE,
        PID,
        TELE_DRIVE,
        TURN,
        LINE_TO_X
    }

    State dtState;

    public SixWheelDriveBase(){
        this(new BluMotor(Globals.flMotorName, DcMotorSimple.Direction.FORWARD),
                new BluMotor(Globals.frMotorName, DcMotorSimple.Direction.REVERSE),
                new BluMotor(Globals.blMotorName, DcMotorSimple.Direction.FORWARD),
                new BluMotor(Globals.brMotorName, DcMotorSimple.Direction.REVERSE));
    }
    private SixWheelDriveBase(BluMotor fl, BluMotor fr, BluMotor bl, BluMotor br){
        dtMotors = new BluMotor[]{fl, fr, bl, br};
        localizer = new Pinpoint("pinpoint");
        dtState = State.IDLE;
        dtMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtMotors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtMotors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init() {
        for (BluMotor motors: dtMotors){
            motors.init();
        }
    }

    @Override
    public void read() {
        for (BluMotor motors: dtMotors){
            motors.read();
        }
        localizer.read();
    }

    @Override
    public void write() {
        for (BluMotor motors: dtMotors){
            motors.write();
        }
    }

    public void drive(double x, double r){
        double[] powers = SixWheelKinematics.getPowers(x,r);
        dtMotors[0].setPower(powers[0]);
        dtMotors[2].setPower(powers[0]);
        dtMotors[1].setPower(powers[1]);
        dtMotors[3].setPower(powers[1]);
        Globals.telemetry.addData("Powers", Arrays.toString(powers));
    }

    public void makeMotorsBeInBrake(){
        for (BluMotor motors: dtMotors){
            motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void makeMotorsBeInFloat(){
        /**for (BluMotor motors: dtMotors){
            motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }*/
    }

    public Pose2d getPos(){
        return localizer.getPose();
    }

    public Pose2d getVel(){
        return localizer.getVel();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        /**for (BluMotor motor:dtMotors){
            motor.telemetry();
        }*/
        localizer.telemetry(telemetry);
        telemetry.addData("pos", localizer.getPose());
        telemetry.addData("Brake Mode", dtMotors[0].getZeroPowerBehavior());
    }

    @Override
    public void reset() {
        localizer.reset();
    }
}
