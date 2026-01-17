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

    Pinpoint localizer;
    public enum State{
        IDLE,
        PID,
        TELE_DRIVE,
        TURN,
        LINE_TO_X
    }

    State dtState;

    public SixWheelDriveBase(){
        this(new BluMotor(Globals.flMotorName, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE),
                new BluMotor(Globals.frMotorName, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE),
                new BluMotor(Globals.blMotorName, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE),
                new BluMotor(Globals.brMotorName, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE));
    }
    private SixWheelDriveBase(BluMotor fl, BluMotor fr, BluMotor bl, BluMotor br){
        dtMotors = new BluMotor[]{fl, fr, bl, br};
        localizer = new Pinpoint("pinpoint");
        dtState = State.IDLE;
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

        //this is mainly for defense current saving it should be fine
        //detection based on accel and vel

        if (beingPushed()){
            dtMotors[2].setPower(powers[0]);
            dtMotors[3].setPower(powers[1]);
            dtMotors[0].setPower(0);
            dtMotors[1].setPower(0);
            Globals.telemetry.addLine("rear wheel drive");
        } else {
            dtMotors[0].setPower(powers[0]);
            dtMotors[2].setPower(powers[0]);
            dtMotors[1].setPower(powers[1]);
            dtMotors[3].setPower(powers[1]);
        }

        Globals.telemetry.addData("Powers", Arrays.toString(powers));
        Globals.telemetry.addData("Being Pushed?", beingPushed());
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

    public boolean beingPushed(){
        //low vel and low accel
        Pose2d vel = localizer.getVel();
        double xyVel = Math.hypot(vel.getX(), vel.getY());
        return (localizer.getXyAccel() < 1 && localizer.gethAccel() < 0.3) && (xyVel < 3 && vel.getH() < 0.1);
    }
}
