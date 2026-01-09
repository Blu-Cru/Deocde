package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous(name = "Test LineToX Only", group = "TEST")
public class TestLineToXOnlyAuto extends BluLinearOpMode {

    private Path currentPath;

    /** Builder that ONLY builds a single line-to-X step. */
    public class LineToXOnlyPath extends SixWheelPIDPathBuilder {
        public LineToXOnlyPath() {
            super();

            // Just one command: drive in a straight line until X == targetX (using whatever your impl does).
            this.addLineToX(-27, 5)
                    .build();
        }
    }

    @Override
    public void initialize() {
        robot.clear();
        addSixWheel();

        // If your line-to functions assume localization already exists, make sure it's reset.
        sixWheel.reset();
        sixWheel.write();
    }

    @Override
    public void onStart() {
        // Start pose should match where you actually place the robot on the field for this test.
        // Use the same starting pose you used in PurePursuitAuto, or change as needed.
        sixWheel.setPosition(new Pose2d(-45, 52, Math.toRadians(127 + 180)));

        // Build and start the path
        currentPath = new LineToXOnlyPath().build().start();
    }

    @Override
    public void periodic() {
        if (currentPath != null) {
            currentPath.run();
        }
    }
}
