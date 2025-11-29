package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

@Config
public class Intake implements BluSubsystem, Subsystem {
    private BluMotorWithEncoder leftMotor;
    private BluMotorWithEncoder rightMotor;
    public boolean jammed;
    public static double JAM_CURRENT_THRESHOLD = 9800; // milliamps, adjust as needed
    public static double NOMINAL_VOLTAGE = 12.0;
    public enum State{
        IN,
        OUT,
        IDlE
    }
    private State state;

    public void setIn() {
        state = State.IN;
    }

    public void setOut() {
        state = State.OUT;
    }

    public void stop(){
        state = State.IDlE;
    }

    public void setIdle() {
        state = State.IDlE;
    }

    public State getState() {
        return state;
    }

    public Intake(String leftMotorName, String rightMotorName) {
        leftMotor = new BluMotorWithEncoder(leftMotorName, DcMotorSimple.Direction.FORWARD);
        rightMotor = new BluMotorWithEncoder(rightMotorName, DcMotorSimple.Direction.REVERSE);
        state = State.IDlE;
        jammed = false;
    }

    @Override
    public void init() {
        leftMotor.init();
        rightMotor.init();
    }

    @Override
    public void read() {
        leftMotor.read();
        rightMotor.read();
        double currentVoltage = Robot.getInstance().getVoltage();

        jammed = (state == State.IN && leftMotor.getVelocity() > 100); // Jam detected, spit out the ball
    }

    @Override
    public void write() {
        if (jammed){
            leftMotor.setPower(-1);
            rightMotor.setPower(-1);
        } else {
            switch(state){
                case IN:
                    leftMotor.setPower(1);
                    rightMotor.setPower(1);
                    break;
                case OUT:
                    leftMotor.setPower(-1);
                    rightMotor.setPower(-1);
                    break;
                case IDlE:
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
            }
        }

        leftMotor.write();
        rightMotor.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        leftMotor.telemetry();
        rightMotor.telemetry();
        telemetry.addData("Current", leftMotor.getCurrent());
    }

    @Override
    public void reset() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.write();
        rightMotor.write();
    }
}
