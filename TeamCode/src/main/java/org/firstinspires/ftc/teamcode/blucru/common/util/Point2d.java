package org.firstinspires.ftc.teamcode.blucru.common.util;

public class Point2d {
    double x;
    double y;
    public Point2d(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public Point2d mirror(double offset) {
        // Since X is length (audience to back) and Y is width (blue to red),
        // we reflect across the X-axis (negate Y) to mirror Blue to Red!
        return new Point2d(x, -y);
    }

    public String toString(){
        return x + ", " + y;
    }
}
