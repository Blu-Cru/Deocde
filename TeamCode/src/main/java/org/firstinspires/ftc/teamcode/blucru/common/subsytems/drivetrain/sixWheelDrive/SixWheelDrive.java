package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.RobotLocalizer;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class SixWheelDrive implements BluSubsystem {

    BluMotor[] dtMotors;

    RobotLocalizer localizer;

    public enum State{
        IDLE,
        PID,
        TELE_DRIVE
    }

    State dtState;

    public SixWheelDrive(){
        this(new BluMotor(Globals.flMotorName),
                new BluMotor(Globals.flMotorName),
                new BluMotor(Globals.blMotorName),
                new BluMotor(Globals.brMotorName));
    }
    private SixWheelDrive(BluMotor fl, BluMotor fr, BluMotor bl, BluMotor br){
        dtMotors = new BluMotor[]{fl, fr, bl, br};
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
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
        for(int i = 0; i<4; i+=2){
            dtMotors[i].setPower(powers[0]);
            dtMotors[i+1].setPower(powers[1]);
        }
    }

    public void teleDrive(Gamepad g1, double tol){
        double x = -g1.left_stick_y;
        double r = g1.left_stick_x;

        if (Math.abs(x) <= tol){
            x = 0;
        }
        if (Math.abs(r) <= tol){
            r = 0;
        }

        if (x == 0 && r == 0){
            if (dtState == State.PID){
                //in pid
            } else {
                //either stopped driving or idle alr
                dtState = State.IDLE;
            }
        } else {
            dtState = State.TELE_DRIVE;
            drive(x,r);
        }

    }

    @Override
    public void telemetry(Telemetry telemetry) {
        for (BluMotor motor:dtMotors){
            motor.telemetry();
        }
    }

    @Override
    public void reset() {
        localizer.setPosition(new Pose2d(0,0,0));
    }
}
