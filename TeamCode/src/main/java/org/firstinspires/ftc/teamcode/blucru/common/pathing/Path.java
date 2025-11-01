package org.firstinspires.ftc.teamcode.blucru.common.pathing;

public interface Path {
    Path start();
    public void run();
    public boolean isDone();
    public void endMecanum();
    public void endSixWheel();
}
