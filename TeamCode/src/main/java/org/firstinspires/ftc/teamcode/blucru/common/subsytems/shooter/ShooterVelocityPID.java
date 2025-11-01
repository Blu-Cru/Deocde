package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

public class ShooterVelocityPID{
    double p;
    double i;
    double d;
    double f;
    PIDController controller;

    public ShooterVelocityPID(double p, double i, double d, double f){
        controller = new PIDController(p,i,d);
        this.f=f;
    }

    public double calculateDeltaPower(double currVel, double targetVel){

        return controller.calculate(currVel, targetVel) + f*targetVel;

    }

    public void setP(double p){
        controller.setP(p);
    }

    public void setD(double d){
        controller.setD(d);
    }

    public void setPD(double p, double d){
        controller.setP(p);
        controller.setD(d);
    }

    public void setPDF(double p, double d, double f){
        setPD(p,d);
        this.f = f;
    }

    public void setPIDF(double p, double i, double d, double f){
        setPDF(p,d,f);
        controller.setI(i);
    }

}
