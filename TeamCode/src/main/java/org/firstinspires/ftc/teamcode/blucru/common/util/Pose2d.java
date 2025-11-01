package org.firstinspires.ftc.teamcode.blucru.common.util;

/**
 * This class is for 2d poses. It is heavily based off Roadrunner's Pose2d class
 * */
public class Pose2d {
    private double x, y, h;
    public Pose2d(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Pose2d(Vector2d vec){
        this.x = vec.getX();
        this.y = vec.getY();
        this.h = vec.getHeading();
    }
    public Pose2d(Vector2d vec, double h){
        this.x = vec.getX();
        this.y = vec.getY();
        this.h = h;
    }

    public Vector2d vec(){
        return new Vector2d(x,y);
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getH(){
        return h;
    }
    public void set(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public void set(double x, double y){
        this.x = x;
        this.y = y;
    }
    public void set(double x){
        this.x = x;
    }

    public double getDistTo(Pose2d pose2d){
        return Math.sqrt(Math.pow(pose2d.getX(),2) + Math.pow(pose2d.getY(),2));
    }


}
