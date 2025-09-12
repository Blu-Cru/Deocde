package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;

public class Turret implements BluSubsystem {
    BluMotorWithEncoder turret;
    public Turret(BluMotorWithEncoder turret){
        this.turret = turret;
    }

    private enum State{
        POWER_MODE,
        PID_MODE,
        GOAL_LOCK_MODE;

    }

    @Override
    public void init() {
        turret.init();
    }

    @Override
    public void read() {
        turret.read();
    }

    @Override
    public void write() {
        turret.write();
    }

    public void moveWithPower(double power){
        turret.setPower(power);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        turret.telemetry();
    }

    @Override
    public void reset() {
        turret.reset();
    }
}
