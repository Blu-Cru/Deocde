package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
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
    private BluEncoder encoder;
    public BluDigitalChannel parallelSensor;
    public boolean jammed;
    public static double JAM_CURRENT_THRESHOLD = 9800; // milliamps, adjust as needed
    public static double NOMINAL_VOLTAGE = 12.0;
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
        motor = new BluMotor(motorName, DcMotorSimple.Direction.REVERSE);
        parallelSensor = new BluDigitalChannel(sensorName);
        encoder = new BluEncoder(motorName);
        pid = new PDController(0.011, 0.001);
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

        jammed = (state == State.IN && encoder.getVelocity() > 100); // Jam detected, spit out the ball
    }

    public boolean armsParallel(){
        return parallelSensor.getState();
    }

    @Override
    public void write() {
        if (jammed){
            motor.setPower(-1);
        } else {
            switch(state){
                case IN:
                    motor.setPower(1);
                    break;
                case OUT:
                    motor.setPower(-1);
                    break;
                case IDlE:
                    motor.setPower(0);
                    break;
                case CUSTOM_POWER:
                    motor.setPower(power);
                case PID:
                    parallelSensor.read();
                    encoder.read();
                    if (!parallelSensor.getState()) {
                        double curr = encoder.getCurrentPos() % (145.1 / 2);
                        if (curr > 145.1 / 4) {
                            curr -= 145.1 / 2;
                        }

                        if (curr < -145.1 / 4) {
                            curr += 145.1 / 2;
                        }
                        double power = pid.calculate(curr, -motor.getPower());
                        Globals.telemetry.addData("Curr", curr);
                        motor.setPower(power);
                    }
            }
        }

        motor.write();
    }


    @Override
    public void telemetry(Telemetry telemetry) {
        motor.telemetry();
        telemetry.addData("Pos", encoder.getCurrentPos());
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
