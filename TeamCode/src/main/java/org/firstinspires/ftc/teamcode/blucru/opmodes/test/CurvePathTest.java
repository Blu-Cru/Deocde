package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import java.util.ArrayList;

@Autonomous(name = "Curve Path Test", group = "Test")
public class CurvePathTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        addSixWheel();
        enableDash();
        telemetry.addLine("Curve Path Test Initialized");
        telemetry.addLine("Press Start to follow a sine wave path.");
    }

    @Override
    public void onStart() {
        // Generate a sine wave path
        // f(x) = 10 * sin(x / 10)
        // x from 0 to 60 inches
        ArrayList<Point2d> waypoints = new ArrayList<>();

        for (double x = 0; x <= 60; x += 2.0) { // Waypoint every 2 inches
            double y = 10 * Math.sin(x / 10.0);
            waypoints.add(new Point2d(x, y));
        }

        Point2d[] path = waypoints.toArray(new Point2d[0]);

        // Start following
        sixWheel.followPathNaive(path);
    }
}
