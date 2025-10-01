package org.firstinspires.ftc.teamcode.blucru.common.subsytems.Intake;

import static org.firstinspires.ftc.teamcode.blucru.common.subsytems.Intake.Intake.State.IN;
import static org.firstinspires.ftc.teamcode.blucru.common.subsytems.Intake.Intake.State.OUT;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;

public class Intake implements BluSubsystem {
    private BluMotor motor;
    private static final double JAM_CURRENT_THRESHOLD = 2500; // milliamps, adjust as needed
    public enum State{
        IN,
        OUT,
        IDlE
    }
    public State state;

    public Intake(String motorName) {
        motor = new BluMotor(motorName);
        state = State.IDlE;
    }

    @Override
    public void init() {
        motor.init();
    }

    @Override
    public void read() {
        motor.read();
        if (state == State.IN && motor.getCurrent() > JAM_CURRENT_THRESHOLD) {
            state = State.OUT; // Jam detected, spit out the ball
        }
    }

    @Override
    public void write() {
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
