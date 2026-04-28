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
    private PDController pid;
    private double targetPos;
    private final double DOWN = 9000;
    private final double UP = 0;
    /**
     *
     * class assumes that positions need to be inverted(ie if one servo needs to go to
     *
     * */
    public Tilt(String tilt){
        this.tilt = new BluCRServo(tilt);
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
        tilt.write();
    }

    public void setPower(double power){
        tilt.setPower(power);
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
    }

    public void goUp(){
        tilt.setPower(-1);
    }
    public void goDown(){
        tilt.setPower(1);
    }

}
