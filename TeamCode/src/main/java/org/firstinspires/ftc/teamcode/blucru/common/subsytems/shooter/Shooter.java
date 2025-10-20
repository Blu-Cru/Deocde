package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
@Config
public class Shooter implements BluSubsystem, Subsystem {

    public static double p = 0.2, d = 0.1, f = 0.01;
    public static double limit = 20;

    private BluMotorWithEncoder shooter;
    private BluServo hood;

    enum State{
        IDLE,
        VELOCITY
    }
    private State state;
    double targetVel = 0;
    ShooterVelocityPID pid;
    public Shooter(){
        shooter = new BluMotorWithEncoder("shooter");
        hood = new BluServo("hood");
        state = State.IDLE;
        pid = new ShooterVelocityPID(p,d,f);
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
        switch (state){
            case IDLE:
                targetVel = 0;
                break;
            case VELOCITY:
                if (Math.abs(shooter.getVel()- targetVel) >= limit){
                    shooter.setPower(shooter.getPower() + pid.calculateDeltaPower(shooter.getVel(), targetVel));
                    Globals.telemetry.addData("delta", pid.calculateDeltaPower(shooter.getVel(), targetVel));
                }
        }

        shooter.write();
        hood.write();
    }

    public void shoot(double power){
        shooter.setPower(power);
    }

    public void shootWithVelocity(double vel){
        targetVel = vel;
        state = State.VELOCITY;
    }

    public double getVel(){
        return shooter.getVel();
    }
    public double getPower(){
        return shooter.getPower();
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

    public void updatePID(){
        pid.setPD(p,d);
    }
}