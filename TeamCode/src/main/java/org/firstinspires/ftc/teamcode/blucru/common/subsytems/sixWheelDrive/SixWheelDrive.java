package org.firstinspires.ftc.teamcode.blucru.common.subsytems.sixWheelDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class SixWheelDrive implements BluSubsystem {

    BluMotor[] dtMotors;

    public SixWheelDrive(BluMotor fl, BluMotor fr, BluMotor bl, BluMotor br){
        dtMotors = new BluMotor[]{fl, fr, bl, br};
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
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

    @Override
    public void telemetry(Telemetry telemetry) {
        for (BluMotor motor:dtMotors){
            motor.telemetry();
        }
    }

    @Override
    public void reset() {

    }
}
