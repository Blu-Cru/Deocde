package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive;

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

public class SixWheelDriveBase implements BluSubsystem{

    BluMotor[] dtMotors;

    RobotLocalizer localizer;
    PurePursuitComputer computer;
    double lookAheadDist = 10;
    SixWheelPID pid;
    public enum State{
        IDLE,
        PID,
        TELE_DRIVE
    }

    State dtState;

    public SixWheelDriveBase(){
        this(new BluMotor(Globals.flMotorName),
                new BluMotor(Globals.flMotorName),
                new BluMotor(Globals.blMotorName),
                new BluMotor(Globals.brMotorName));
    }
    private SixWheelDriveBase(BluMotor fl, BluMotor fr, BluMotor bl, BluMotor br){
        dtMotors = new BluMotor[]{fl, fr, bl, br};
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        localizer = new Pinpoint("pinpoint");
        dtState = State.IDLE;
        computer = new PurePursuitComputer();
        pid = new SixWheelPID();
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
        for(int i = 0; i<4; i+=2){
            dtMotors[i].setPower(powers[0]);
            dtMotors[i+1].setPower(powers[1]);
        }
    }

    public Pose2d getPos(){
        return localizer.getPose();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        for (BluMotor motor:dtMotors){
            motor.telemetry();
        }
        localizer.telemetry(telemetry);
        telemetry.addData("pos", localizer.getPose());
    }

    @Override
    public void reset() {
        localizer.setPosition(new Pose2d(0,0,0));
    }
}
