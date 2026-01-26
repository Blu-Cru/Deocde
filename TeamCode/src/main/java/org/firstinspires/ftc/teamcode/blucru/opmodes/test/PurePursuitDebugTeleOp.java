package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.SixWheelDriveBase;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.purePursuit.PurePursuitComputer;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.purePursuit.PurePursuitDebugData;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.purePursuit.SixWheelPID;
import org.firstinspires.ftc.teamcode.blucru.common.util.PurePursuitLogger;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

/**
 * Debug TeleOp for diagnosing PurePursuit issues
 * Logs detailed debug data while robot follows test paths
 *
 * Controls:
 * A - Start following selected test path and begin logging
 * B - Stop path following and save log
 * X - Cycle through test paths
 * Y - Reset robot position to (0, 0, 0°)
 * DPad Up - Start new log file
 * Left/Right Sticks - Manual drive when not following path
 */
@Config
@TeleOp(name = "PurePursuit Debug", group = "test")
public class PurePursuitDebugTeleOp extends BluLinearOpMode {

    // Dashboard-configurable parameters
    public static double LOG_INTERVAL_MS = 50.0; // 20Hz logging
    public static int SELECTED_TEST_PATH = 0;
    //test

    // Test paths
    private static class TestPath {
        String name;
        String description;
        Point2d[] waypoints;

        TestPath(String name, String description, Point2d[] waypoints) {
            this.name = name;
            this.description = description;
            this.waypoints = waypoints;
        }
    }

    private TestPath[] testPaths;
    private int currentPathIndex = 0;

    // Debug components
    private PurePursuitLogger logger;

    // State
    private boolean isFollowingPath = false;
    private long pathStartTime = 0;
    private long lastLogTime = 0;
    private int logSampleCount = 0;

    @Override
    public void initialize() {
        robot.clear();
        addSixWheel();
        enableDash();

        // Initialize logging
        logger = new PurePursuitLogger();

        // Define test paths
        initializeTestPaths();
    }

    private void initializeTestPaths() {
        testPaths = new TestPath[] {
            new TestPath(
                "Straight Line Short",
                "40 inches forward - tests basic straight-line tracking",
                new Point2d[] {
                    new Point2d(0, 0),
                    new Point2d(40, 0)
                }
            ),
            new TestPath(
                "Straight Line Long",
                "100 inches forward - tests distance calculation and drift",
                new Point2d[] {
                    new Point2d(0, 0),
                    new Point2d(100, 0)
                }
            ),
            new TestPath(
                "Straight Line Multi-Segment",
                "90 inches in 3 segments - tests segment transitions",
                new Point2d[] {
                    new Point2d(0, 0),
                    new Point2d(30, 0),
                    new Point2d(60, 0),
                    new Point2d(90, 0)
                }
            ),
            new TestPath(
                "Simple Curve",
                "Gentle curve - tests heading control",
                new Point2d[] {
                    new Point2d(0, 0),
                    new Point2d(30, 0),
                    new Point2d(50, 10)
                }
            )
        };

        // Set initial path
        currentPathIndex = SELECTED_TEST_PATH % testPaths.length;
    }

    @Override
    public void periodic() {
        handleControls();

        if (isFollowingPath) {
            // Check if path is complete
            if (sixWheel.getState() == SixWheelDriveBase.State.IDLE) {
                stopFollowing();
            } else {
                // Collect and log data at specified interval
                long currentTime = System.currentTimeMillis();
                if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
                    collectAndLogData(currentTime);
                    lastLogTime = currentTime;
                }
            }
        } else {
            // Manual drive mode
            sixWheel.teleDrive(gamepad1, 0.001);
        }

        // Always update telemetry
        updateTelemetry();
    }

    private void handleControls() {
        // A - Start path following
        if (driver1.pressedA()) {
            startFollowing();
        }

        // B - Stop path following
        if (driver1.pressedB()) {
            stopFollowing();
        }

        // X - Cycle through test paths
        if (driver1.pressedX()) {
            cycleTestPath();
        }

        // Y - Reset robot position
        if (driver1.pressedY()) {
            resetRobotPosition();
        }

        // DPad Up - Start new log file
        if (driver1.pressedDpadUp()) {
            startNewLog();
        }
    }

    private void startFollowing() {
        if (isFollowingPath) {
            return; // Already following
        }

        TestPath path = testPaths[currentPathIndex];

        // Start logging
        if (logger.startLogging(path.name)) {
            // Start path following
            sixWheel.followPath(path.waypoints);

            isFollowingPath = true;
            pathStartTime = System.currentTimeMillis();
            lastLogTime = pathStartTime;
            logSampleCount = 0;

            telemetry.addLine("Started following: " + path.name);
        } else {
            telemetry.addLine("ERROR: Failed to start logging!");
        }
    }

    private void stopFollowing() {
        if (!isFollowingPath) {
            return; // Not following
        }

        // Stop path following
        sixWheel.switchToIdle();

        // Stop logging
        logger.stopLogging();

        isFollowingPath = false;

        telemetry.addLine("Stopped following");
        telemetry.addLine("Log saved: " + logger.getLogFilename());
        telemetry.addLine("Samples logged: " + logSampleCount);
    }

    private void cycleTestPath() {
        if (isFollowingPath) {
            telemetry.addLine("Stop current path first!");
            return;
        }

        currentPathIndex = (currentPathIndex + 1) % testPaths.length;
        TestPath path = testPaths[currentPathIndex];

        telemetry.addLine("Selected: " + path.name);
        telemetry.addLine(path.description);
    }

    private void resetRobotPosition() {
        sixWheel.setPosition(new Pose2d(0, 0, 0));
        telemetry.addLine("Robot position reset to (0, 0, 0°)");
    }

    private void startNewLog() {
        if (isFollowingPath) {
            // Save current log and start new one
            logger.stopLogging();
            TestPath path = testPaths[currentPathIndex];
            if (logger.startLogging(path.name)) {
                pathStartTime = System.currentTimeMillis();
                lastLogTime = pathStartTime;
                logSampleCount = 0;
                telemetry.addLine("Started new log file");
            }
        } else {
            telemetry.addLine("Start following a path first!");
        }
    }

    private void collectAndLogData(long currentTime) {
        // Create debug data point
        PurePursuitDebugData data = new PurePursuitDebugData();

        // Timestamp
        data.timestamp = currentTime - pathStartTime;

        // Robot state
        Pose2d pose = sixWheel.getPos();
        data.robotX = pose.getX();
        data.robotY = pose.getY();
        data.robotHeadingDeg = Math.toDegrees(pose.getH());

        Pose2d vel = sixWheel.getVel();
        data.velX = vel.getX();
        data.velY = vel.getY();
        data.angularVel = Math.toDegrees(vel.getH());

        // PurePursuit state
        PurePursuitComputer computer = sixWheel.getPurePursuitComputer();
        Point2d target = computer.getLastGoalPoint();
        data.targetX = target.getX();
        data.targetY = target.getY();
        data.distanceRemaining = computer.getLastDistanceRemaining();
        data.currentSegmentIndex = computer.getLastSegmentIndex();
        data.lookAheadDistance = computer.getLastEffectiveLookahead();

        // PID values
        data.linearError = SixWheelPID.getLastLinearError();
        data.linearPTerm = SixWheelPID.getLastLinearPTerm();
        data.linearDTerm = SixWheelPID.getLastLinearDTerm();
        data.linearOutput = SixWheelPID.getLastLinearOutput();

        data.headingCurrentDeg = data.robotHeadingDeg;
        data.headingTargetDeg = SixWheelPID.getLastHeadingTarget();
        data.headingErrorDeg = SixWheelPID.getLastHeadingError();
        data.headingPTerm = SixWheelPID.getLastHeadingPTerm();
        data.headingDTerm = SixWheelPID.getLastHeadingDTerm();
        data.headingOutput = SixWheelPID.getLastHeadingOutput();

        data.isDrivingBackwards = SixWheelPID.getLastBackwardsState();

        // Log to file
        if (logger.logDataPoint(data)) {
            logSampleCount++;
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("===== PUREPURSUIT DEBUG =====");

        // Status
        String status = isFollowingPath ? "FOLLOWING PATH" : "IDLE";
        telemetry.addData("Status", status);

        TestPath currentPath = testPaths[currentPathIndex];
        telemetry.addData("Test Path", "[" + currentPathIndex + "] " + currentPath.name);

        if (logger.isLogging()) {
            telemetry.addData("Log File", logger.getLogFilename());
            long elapsed = System.currentTimeMillis() - pathStartTime;
            telemetry.addData("Time / Samples", String.format("%.1fs / %d", elapsed / 1000.0, logSampleCount));
        }

        telemetry.addLine();

        // Robot State
        telemetry.addLine("--- Robot State ---");
        Pose2d pose = sixWheel.getPos();
        telemetry.addData("Position", String.format("X=%.2f, Y=%.2f, H=%.1f°",
            pose.getX(), pose.getY(), Math.toDegrees(pose.getH())));

        Pose2d vel = sixWheel.getVel();
        telemetry.addData("Velocity", String.format("vX=%.2f, vY=%.2f, vAng=%.1f°/s",
            vel.getX(), vel.getY(), Math.toDegrees(vel.getH())));

        if (isFollowingPath) {
            telemetry.addLine();

            // Path Following
            telemetry.addLine("--- Path Following ---");
            PurePursuitComputer computer = sixWheel.getPurePursuitComputer();
            Point2d target = computer.getLastGoalPoint();
            telemetry.addData("Target Point", String.format("(%.2f, %.2f)", target.getX(), target.getY()));
            telemetry.addData("Distance Remaining", String.format("%.2f in", computer.getLastDistanceRemaining()));
            telemetry.addData("Current Segment", computer.getLastSegmentIndex());
            telemetry.addData("Look-Ahead", String.format("%.2f in", sixWheel.getLookAheadDist()));
            telemetry.addData("Backwards", SixWheelPID.getLastBackwardsState());

            telemetry.addLine();

            // Linear Control
            telemetry.addLine("--- Linear Control (PD) ---");
            telemetry.addData("Error", String.format("%.2f in", SixWheelPID.getLastLinearError()));
            telemetry.addData("P Term", String.format("%.3f (gain: %.3f)",
                SixWheelPID.getLastLinearPTerm(), SixWheelPID.pXY));
            telemetry.addData("D Term", String.format("%.3f (gain: %.3f)",
                SixWheelPID.getLastLinearDTerm(), SixWheelPID.dXY));
            telemetry.addData("Output", String.format("%.3f", SixWheelPID.getLastLinearOutput()));

            telemetry.addLine();

            // Heading Control
            telemetry.addLine("--- Heading Control (PD) ---");
            telemetry.addData("Current", String.format("%.2f°", Math.toDegrees(pose.getH())));
            telemetry.addData("Target", String.format("%.2f°", SixWheelPID.getLastHeadingTarget()));
            telemetry.addData("Error", String.format("%.2f°", SixWheelPID.getLastHeadingError()));
            telemetry.addData("P Term", String.format("%.3f (gain: %.3f)",
                SixWheelPID.getLastHeadingPTerm(), SixWheelPID.pR));
            telemetry.addData("D Term", String.format("%.3f (gain: %.3f)",
                SixWheelPID.getLastHeadingDTerm(), SixWheelPID.dR));
            telemetry.addData("Output", String.format("%.3f", SixWheelPID.getLastHeadingOutput()));
        }

        telemetry.addLine();

        // Controls
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("A=Start | B=Stop | X=Next Path");
        telemetry.addLine("Y=Reset Pos | DPad↑=New Log");
    }

    @Override
    public void end() {
        // Make sure to stop logging when OpMode ends
        if (logger.isLogging()) {
            logger.stopLogging();
        }
    }
}
