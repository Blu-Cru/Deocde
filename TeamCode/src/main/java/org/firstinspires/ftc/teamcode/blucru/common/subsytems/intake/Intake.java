package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;

public class Intake implements BluSubsystem, Subsystem {
    private BluMotor motor;
    public boolean jammed;
    private static final double JAM_CURRENT_THRESHOLD = 2500; // milliamps, adjust as needed
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
        if (state == State.IN && motor.getCurrent() > JAM_CURRENT_THRESHOLD) {
            jammed = true; // Jam detected, spit out the ball
        }
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
            }
        }

        motor.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        motor.telemetry();
    }

    @Override
    public void reset() {
        motor.setPower(0);
        motor.write();
    }
}
