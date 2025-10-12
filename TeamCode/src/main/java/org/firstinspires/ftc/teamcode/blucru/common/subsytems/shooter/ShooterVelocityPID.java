package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;

public class ShooterVelocityPID{
    PDController controller;
    double currAccel;
    public ShooterVelocityPID(PDController controller){
        this.controller = controller;
        currAccel = 0;
    }

    public ShooterVelocityPID(double p, double d){
        this(new PDController(p,d));
    }

    public double calculateDeltaPower(double currVel, double targetVel){

        //this should keep currAccel always updated
        currAccel = controller.calculate(currVel, targetVel, currAccel ,0);

        return currAccel;
    }

    public void setP(double p){
        controller.setP(p);
    }

    public void setD(double d){
        controller.setD(d);
    }

    public void setPD(double p, double d){
        controller.setPD(p,d);
    }

}
