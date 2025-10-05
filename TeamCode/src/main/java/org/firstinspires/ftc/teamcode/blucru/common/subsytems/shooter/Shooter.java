package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class Shooter implements BluSubsystem, Subsystem {

    private BluMotorWithEncoder shooter;
    private BluServo hood;

    public Shooter(){
        shooter = new BluMotorWithEncoder("shooter");
        hood = new BluServo("hood");
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
        hood.write();
    }

    public void shoot(double power){
        shooter.setPower(power);
    }

    public void rampDownShooter(){
        shooter.setPower(0);
    }
    public void setHoodAngle(double angle){
        hood.setPos(Globals.convertAngleToServoPos(255, angle));
    }
    public void setHoodServoPos(double pos){
        hood.setPos(pos);
    }

    public double getHoodAngle(){
        return Globals.convertServoPosToAngle(255,hood.getPos());
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