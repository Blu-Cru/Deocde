package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

@Config
public class Intake implements BluSubsystem, Subsystem {
    private BluMotor motor;
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

    public Intake(String motorName) {
        motor = new BluMotor(motorName);
        state = State.IDlE;
        jammed = false;
    }

    @Override
    public void init() {
        motor.init();
    }

    @Override
    public void read() {
        motor.read();
        double currentVoltage = Robot.getInstance().getVoltage();
        double adjustedThreshold = JAM_CURRENT_THRESHOLD * (currentVoltage / NOMINAL_VOLTAGE);

        jammed = (state == State.IN && motor.getCurrent() > adjustedThreshold); // Jam detected, spit out the ball
    }

    @Override
    public void write() {
        if (jammed){
            motor.setPower(1);
        } else {
            switch(state){
                case IN:
                    motor.setPower(-1);
                    break;
                case OUT:
                    motor.setPower(1);
                    break;
                case IDlE:
                    motor.setPower(0);
                    break;
            }
        }

        motor.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        motor.telemetry();
        telemetry.addData("Current", motor.getCurrent());
    }

    @Override
    public void reset() {
        motor.setPower(0);
        motor.write();
    }
}
