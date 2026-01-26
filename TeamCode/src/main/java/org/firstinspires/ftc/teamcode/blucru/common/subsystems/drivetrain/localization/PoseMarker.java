package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.os.SystemClock;
import android.util.Log;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

public class PoseMarker {

    long nanoTime;
    private Pose2d pose, vel;

    public PoseMarker(Pose2d pose, Pose2d vel){
        nanoTime = (long) (System.currentTimeMillis() * Math.pow(10, 6));
        this.pose = pose;
        this.vel = vel;
    }

    public PoseMarker(long nanoTime, Pose2d pose){
        this.nanoTime = nanoTime;
        this.pose = new Pose2d(new Vector2d(pose.getX(), pose.getY()), pose.getH());
    }

    public Pose2d getPose(){
        return pose;
    }

    public Pose2d getVel(){
        return vel;
    }
    public void setPose(Pose2d pose){
        this.pose = pose;
    }

    public void setVel(Pose2d vel){
        this.vel = vel;
    }

    public void log(String tag){
        Log.v(tag, "PoseMarker at pose: " + pose + ", Vel:" + vel + ", Time: " + nanoTime/Math.pow(10.0,6));
    }
    public String toString(){
        return pose.toString();
    }

}
