package org.firstinspires.ftc.teamcode.blucru.common.subsytems.Intake;

import static org.firstinspires.ftc.teamcode.blucru.common.subsytems.Intake.Intake.State.IN;
import static org.firstinspires.ftc.teamcode.blucru.common.subsytems.Intake.Intake.State.OUT;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;

public class Intake implements BluSubsystem {
    private BluMotor motor;
    public enum State{
        IN,
        OUT,
        IDlE
    }
    public State state;
    @Override
    public void init() {

    }

    @Override
    public void read() {

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
    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }

    @Override
    public void reset() {

    }
}
