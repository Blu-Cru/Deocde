package org.firstinspires.ftc.teamcode.blucru.common.subsytems.sixWheelDrive.localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public interface RobotLocalizer {

    public void read();
    public Pose2d getPose();
    public double getX();
    public double getY();
    public double getHeading();
    public void setOffset(double x, double y, double h);
    public void setPosition(double x, double y, double h);
    public void setPosition(Pose2d pose);
    public Pose2d getVel();
    public void setHeading(double heading);
    public void telemetry(Telemetry telemetry);
}
