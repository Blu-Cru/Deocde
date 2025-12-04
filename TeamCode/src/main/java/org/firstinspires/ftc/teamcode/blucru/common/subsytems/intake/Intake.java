package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluDigitalChannel;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@Config
public class Intake implements BluSubsystem, Subsystem {
    private BluMotorWithEncoder leftMotor;
    private BluMotorWithEncoder rightMotor;
    private BluDigitalChannel parallelSensor;
    public boolean jammed;
    public static double JAM_CURRENT_THRESHOLD = 9800; // milliamps, adjust as needed
    public static double NOMINAL_VOLTAGE = 12.0;
    public enum State{
        IN,
        OUT,
        IDlE,
        CUSTOM_POWER
    }
    private State state;
    private double power;
    private boolean parallelingArms;

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

    public void setPower(double power){
        this.power = power;
        state = State.CUSTOM_POWER;
    }
    public void setParallelingArms(boolean bool){
        parallelingArms = bool;
    }
    public boolean getParallelingArms(){
        return  parallelingArms;
    }

    public State getState() {
        return state;
    }

    public Intake(String leftMotorName, String rightMotorName, String sensorName) {
        leftMotor = new BluMotorWithEncoder(leftMotorName, DcMotorSimple.Direction.FORWARD);
        rightMotor = new BluMotorWithEncoder(rightMotorName, DcMotorSimple.Direction.REVERSE);
        parallelSensor = new BluDigitalChannel(sensorName);
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
        parallelSensor.read();
        double currentVoltage = Robot.getInstance().getVoltage();

        jammed = (state == State.IN && leftMotor.getVelocity() > 100); // Jam detected, spit out the ball
    }

    public boolean armsParallel(){
        return parallelSensor.getState();
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
                case CUSTOM_POWER:
                    leftMotor.setPower(power);
                    rightMotor.setPower(power);
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
