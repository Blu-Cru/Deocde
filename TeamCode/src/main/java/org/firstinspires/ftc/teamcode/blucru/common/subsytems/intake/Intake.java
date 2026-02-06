package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluDigitalChannel;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

@Config
public class Intake implements BluSubsystem, Subsystem {
    private BluMotor motor;
    private double encoderIteration = 0;
    private BluEncoder encoder;
    public BluDigitalChannel parallelSensor;
    public boolean jammed;
    public static double JAM_CURRENT_THRESHOLD = 9800; // milliamps, adjust as needed
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double ENCODER_PPR_INTAKE = 145.090909091;
    public static double curr = 0;
    public static double offset = 0;
    boolean armsParallel;
    boolean withinRange;
    private PDController pid;
    public enum State{
        IN,
        OUT,
        IDlE,
        CUSTOM_POWER,
        PID
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

    public Intake(String motorName, String sensorName) {
        motor = new BluMotor(motorName, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        parallelSensor = new BluDigitalChannel(sensorName);
        encoder = new BluEncoder(motorName);
        pid = new PDController(0.01, 0.002);
        state = State.IDlE;
        jammed = false;
    }

    public void setPID(){
        state = State.PID;
    }

    @Override
    public void init() {
        motor.init();
        encoder.init();
    }

    @Override
    public void read() {
        motor.read();
        //double currentVoltage = Robot.getInstance().getVoltage();

//        jammed = (state == State.IN && encoder.getVelocity() > 100); // Jam detected, spit out the ball
    }

    public boolean armsParallel(){
        return withinRange;
    }

    @Override
    public void write() {
        if (jammed){
            motor.setPower(-1);
        } else {
            switch(state){
                case IN:
                    armsParallel = false;
                    motor.setPower(1);
                    break;
                case OUT:
                    armsParallel = false;
                    motor.setPower(-1);
                    break;
                case IDlE:
                    armsParallel = false;
                    if (motor.getPower() > 0.05){
                        motor.setPower(-0.01);
                    } else if (motor.getPower() < -0.05){
                        motor.setPower(0.01);
                    } {
                        motor.setPower(0);
                    }
                    break;
                case CUSTOM_POWER:
                    armsParallel = false;
                    motor.setPower(power);
                case PID:
//                    parallelSensor.read();
                    encoder.read();
//                    Globals.telemetry.addData("parallel sensor state", parallelSensor.getState());
                    if (!armsParallel) {
                        double half = ENCODER_PPR_INTAKE / 2.0;
                        double quarter = ENCODER_PPR_INTAKE / 4.0;

                        double curr = encoder.getCurrentPos() % half;
                        if (curr >  quarter) curr -= half;
                        if (curr < -quarter) curr += half;

                        double error = curr - offset;
                        error %= half;
                        if (error >  quarter) error -= half;
                        if (error < -quarter) error += half;

                        armsParallel = Math.abs(error) < 2;
                        withinRange = Math.abs(error) < 5;

                        double power = pid.calculate(error, -motor.getPower());
                        motor.setPower(power);
                    } else {
                        //resetEncoder();
                        armsParallel = true;
                    }
            }
        }

        motor.write();
    }


    @Override
    public void telemetry(Telemetry telemetry) {
        motor.telemetry();
        telemetry.addData("Pos", encoder.getCurrentPos());
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("State", state);
    }

    @Override
    public void reset() {
        motor.setPower(0);
        motor.write();
    }

    public void resetEncoder(){
        encoder.reset();
    }
}
