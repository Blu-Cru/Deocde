package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;

public class Shooter implements BluSubsystem {

    BluMotorWithEncoder shooter;

    public Shooter(){
        shooter = new BluMotorWithEncoder("shooter");
    }

    @Override
    public void init() {
        shooter.init();
    }

    @Override
    public void read() {
        shooter.read();
    }

    @Override
    public void write() {
        shooter.write();
    }

    public void shoot(double power){
        shooter.setPower(power);
    }

    public void rampDownShooter(){
        shooter.setPower(0.5);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Shooter power", shooter.getPower());
        telemetry.addData("Shooter Pos", shooter.getCurrentPos());
    }

    @Override
    public void reset() {
        shooter.reset();
        shooter.read();
    }
}
