package org.firstinspires.ftc.teamcode.blucru.common.subsytems.tilt;

import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

public class Tilt implements Subsystem, BluSubsystem {
    private BluCRServo tilt;
    private BluEncoder encoder;
    private PDController pid;
    private double targetPos;
    private final double DOWN = 9000;
    private final double UP = 0;

    public enum State{
        PID,
        MANUAL
    }

    private State state;
    /**
     *
     * class assumes that positions need to be inverted(ie if one servo needs to go to
     *
     * */
    public Tilt(String tilt, String encoder){
        this.tilt = new BluCRServo(tilt);
        this.encoder = new BluEncoder(encoder);
        pid = new PDController(0.1, 0.001);
        state = State.MANUAL;
    }

    @Override
    public void init() {
        tilt.init();
    }

    @Override
    public void read() {
        tilt.read();
    }

    @Override
    public void write() {
        switch(state){
            case PID:
                tilt.setPower(pid.calculate(encoder.getCurrentPos() - targetPos, encoder.getVel()));
                break;
            case MANUAL:
                break;
        }
        tilt.write();
    }

    public void setPower(double power){
        tilt.setPower(power);
        state = State.MANUAL;
    }

    public double getPower(){
        return tilt.getPower();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        tilt.telemetry();
    }

    @Override
    public void reset() {
        tilt.setPower(0);
        state = State.MANUAL;
    }

    public void setDown(){
        targetPos = DOWN;
        state = State.PID;
    }

    public void setUp(){
        targetPos = UP;
        state = State.PID;
    }


}
