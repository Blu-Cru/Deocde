package org.firstinspires.ftc.teamcode.blucru.common.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotionProfile {
    double vMax, aMax, xTarget, xI;
    double vI, flip;
    boolean decel;
    double xDecel, xAccel;
    double tAccel;
    double t0, t1, t2, t3;
    double d0, d1, d2, d3;
    double v0, v1, v2, v3;
    double a0, a1, a2, a3;
    double dist;
    double startTime;

    Vector2d instantState;

    public MotionProfile(double xTarget, double xI, double vMax, double aMax){
        this.xTarget = xTarget;
        this.xI = xI;
        this.vMax = vMax;
        this.vI = 0;
        this.aMax = aMax;
        if (xTarget < xI){
            //going backwards
            flip = -1;
        } else{
            flip = 1;
        }
        decel = false;
        calculate();
        instantState = new Vector2d(0,0);
    }

    public MotionProfile(double xTarget, double xI, double vI, double vMax, double aMax){
        this.xTarget = xTarget;
        this.xI = xI;
        this.vI = vI;
        this.vMax = vMax;
        this.aMax = aMax;

        xDecel = vI < 0 ? -(vI * vI) / (2.0 * aMax) : (vI * vI) / (2.0 * aMax);

        if (xTarget < xI + xDecel){
            //going backwards
            flip = -1;
        } else{
            flip = 1;
        }

        if (vI < 0 && xTarget > xI + xDecel){
            decel = true;
        } else{
            decel = vI > 0 && xTarget < xI + xDecel;
        }
    }

    public void calculate(){
        if (decel){
            //needs to reverse direction

            //stop time
            t0 = Math.abs(vI/aMax);

            //stopping dist (should be flipped)
            d0 = -0.5 * aMax * t0 * t0;

            //time to max
            t1 = vMax/aMax;

            //distance to max vel
            d1 = 0.5 * aMax * t1 * t1;
            dist = Math.abs(xTarget - xI - xDecel);

            double halfDist = dist/2.0;

            if (d1 > halfDist){
                //cant reach max vel
                t1 = Math.sqrt(2.0 * halfDist / aMax);
            }
            //need to recacl bc t1 might have changed
            d1 = 0.5 * aMax * t1 * t1;

            //new vMax based on if there is enough dist
            vMax = aMax * t1;

            d2 = dist - 2 * d1;
            t2 = d2/vMax;

            t3 = t1;
            d3 = d1;

        } else {
            //direction is fine

            t0 = 0.0;
            d0 = 0.0;

            //get time to max
            t1 = Math.abs((vMax * flip - vI)/ aMax);

            //get dist to max
            d1 = Math.abs((0.5 * aMax * t1 * t1) * flip + vI * t1);

            //get distance from 0 to init vel
            xAccel = (vI * vI) / (2 * aMax);

            //get time to accel from 0 to init vel
            tAccel = Math.sqrt(2.0 * xAccel/aMax);

            //get dist
            dist = Math.abs(xTarget - xI) + xAccel;
            double halfDist = dist/2;

            if (d1 > halfDist - xAccel){
                //cant reach max, update d1 and t1 off that
                d1 = halfDist - xAccel;
                t1 = Math.sqrt(2.0 * halfDist/aMax) - tAccel;
            }

            //update vmax if t1 changed
            vMax = aMax * t1 + Math.abs(vI);

            //get dist and tme where vel doesnt change
            d2 = dist - 2 * (xAccel + d1);

            t2 = d2 / vMax;

            //get decel time and dist
            t3 = vMax/aMax;

            d3 = Math.abs(vMax * t3 - 0.5 * aMax * t3 * t3);
        }
    }

    public Vector2d getInstantState(){
        double instantPos = getInstantTargetPos();
        double instantVel = getInstantTargetVel();

        instantState = new Vector2d(instantPos, instantVel);
        return instantState;
    }

    public double getInstantTargetPos(){
        //get time in seconds
        double time = (System.currentTimeMillis() - startTime)/1000;

        //variable for change in time
        double dt;

        if (time < t0){
            //getting to the right direction
            dt = time;
            return (vI * dt) + 0.5 * aMax * t0 * t0 * flip + xI;
        } else if (time < t0 + t1){
            //accelerating in the right direction

            //time from vel of 0
            dt = time - t0;
            return (d0 + 0.5 * aMax * dt * dt * flip) + xI;
        } else if (time < t0 + t1+ t2){
            //at cruise
            dt = time - t0 - t1;
            return (d0 + d1 +vMax * dt) * flip + xI;
        } else if (time < t0 + t1 + t2 + t3) {
            //slowing down
            dt = time - t0 - t1 - t2;
            return (d0 + d1 + d2 + vMax * dt - 0.5 * aMax * dt * dt) * flip + xI;
        }
        return xTarget;
    }
    public double getInstantTargetVel(){
        double instantTargetVel = 0;
        //get time in seconds
        double time = (System.currentTimeMillis() - startTime)/1000;

        //variable for change in time
        double dt;

        if (time < t0){
            dt = time;
            instantTargetVel = vI + aMax * dt * flip;
        } else if (time < t0 + t1){
            dt = time - t0;
            instantTargetVel = aMax * dt * flip;
        } else if (time < t0 + t1 + t2){
            dt = time - t0 - t1;
            instantTargetVel = vMax * flip;
        } else if (time < t0 + t1 + t2 + t3){
            dt = time - t0 - t1 - t2;
            instantTargetVel = (vMax - aMax * dt) * flip;
        }
        return instantTargetVel;
    }

    public MotionProfile start(){
        startTime = System.currentTimeMillis();
        return this;
    }

    public boolean done(){
        double time = (System.currentTimeMillis() - startTime)/1000;
        return time > t0 + t1 + t2 + t3;
    }

    public void setVandAMax(double vMax, double aMax){
        this.vMax = vMax;
        this.aMax = aMax;
    }

    public void telemetry(Telemetry telemetry, boolean debug){
        if (debug){
            telemetry.addData("t0", t0);
            telemetry.addData("t1", t1);
            telemetry.addData("t2", t2);
            telemetry.addData("t3", t3);
            telemetry.addData("d0", d0);
            telemetry.addData("d1", d1);
            telemetry.addData("d2", d2);
            telemetry.addData("d3", d3);
        }

        telemetry.addData("Instant Pos", instantState.getX());
        telemetry.addData("Instant State", instantState.getY());
        telemetry.addData("Dist", dist);
        telemetry.addData("Final Pos", xTarget);
        telemetry.addData("Initial Pos", xI);
        telemetry.addData("vMax", vMax);
        telemetry.addData("aMax", aMax);
    }

    public void telemetry(Telemetry telemetry){
        telemetry(telemetry, false);
    }


}
