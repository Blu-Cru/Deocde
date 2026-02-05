package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

public class ShooterPod implements BluSubsystem {
    private BluMotorWithEncoder shooter;
    private Hood hood;
    private ShooterVelocityPID pid;
    private double power;
    private double targetVel;
    private final double idlePower = 0.3;
    public ShooterPod(BluMotorWithEncoder shooter, Hood hood, ShooterVelocityPID pid){
        this.shooter = shooter;
        this.hood = hood;
        this.pid = pid;
    }


    @Override
    public void init() {
        shooter.init();
        hood.init();
    }

    public void read(){
        shooter.read();
        hood.read();
    }

    public void write(){
        shooter.write();
        hood.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }

    @Override
    public void reset() {
        power = 0;
        shooter.setPower(0);
    }

    public void setPower(double power){
        shooter.setPower(power);
    }

    public void idle(){
        setPower(idlePower);
    }
    public void reverseIdle(){
        setPower(-idlePower);
    }

    public void setVel(double targetVel){
        this.targetVel = targetVel;
    }

    public double getTargetVel(){
        return targetVel;
    }

    public double getVel(){
        return shooter.getVel();
    }

    public double getError(){
        return getVel()-getTargetVel();
    }

    public double getMotorPower(){
        return shooter.getPower();
    }

    public double getTargetPower(){
        return power;
    }

    public void setHoodAngle(double angle){
        hood.setShooterAngle(angle);
    }

    public double getAngle(){
        return hood.getHoodAngle();
    }

    public double getPowerToGoToVel(){
        return pid.calculateDeltaPower(shooter.getVel(), targetVel);
    }


}
