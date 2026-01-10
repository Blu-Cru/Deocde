package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

/**
 * Pure Pursuit PID Tuning OpMode
 *
 * This opmode guides you through tuning your pure pursuit controller step by step.
 *
 * CONTROLS:
 * - A: Run test path
 * - B: Next tuning stage
 * - X: Previous tuning stage
 * - Y: Reset robot position to start
 *
 * Use FTC Dashboard to adjust gains in real-time while testing!
 */
@TeleOp(name = "Pure Pursuit Tuner", group = "Tuning")
@Config
public class PurePursuitTuner extends BluLinearOpMode {

    // Test path configurations
    @Config
    public static class TestPath {
        // Simple test path for initial tuning
        public static double straightDistance = 80.0;
        public static double curveRadius = 20.0;

        // Path timeout
        public static double maxTime = 10000; // 10 seconds
    }

    // Tuning stages
    private enum TuningStage {
        STAGE_0_INTRO(
            "STAGE 0: Introduction",
            "Welcome to Pure Pursuit Tuner!\n" +
            "This will guide you through tuning:\n" +
            "1. P gains (response speed)\n" +
            "2. D gains (damping/smoothing)\n" +
            "3. Look-ahead distance\n" +
            "4. Final validation\n\n" +
            "Press B to start tuning →"
        ),

        STAGE_1_P_LINEAR(
            "STAGE 1: Tune Linear P Gain",
            "Tune: SixWheelPID.pXY\n" +
            "Set: SixWheelPID.dXY = 0.0 (disable D)\n\n" +
            "Goal: Robot follows path at good speed\n\n" +
            "TOO SLOW: Increase pXY (try 0.15-0.2)\n" +
            "TOO AGGRESSIVE/OVERSHOOTS: Decrease pXY (try 0.05-0.07)\n\n" +
            "Watch: 'Linear Vel' should be smooth\n" +
            "A = Run Path | B = Next Stage →"
        ),

        STAGE_2_P_ROTATION(
            "STAGE 2: Tune Rotation P Gain",
            "Tune: SixWheelPID.pR\n" +
            "Set: SixWheelPID.dR = 0.0 (disable D)\n\n" +
            "Goal: Robot turns smoothly without spinning\n\n" +
            "TOO SLOW TO TURN: Increase pR (try 0.04-0.05)\n" +
            "SPINS/OVERSHOOTS: Decrease pR (try 0.015-0.02)\n\n" +
            "Watch: 'Rot Vel' should be smooth\n" +
            "A = Run Path | B = Next Stage →"
        ),

        STAGE_3_D_LINEAR(
            "STAGE 3: Tune Linear D Gain",
            "Tune: SixWheelPID.dXY\n" +
            "Start: dXY = pXY * 0.5 (as starting point)\n\n" +
            "Goal: Reduce oscillations without jitter\n\n" +
            "OSCILLATES: Increase dXY (+0.02 at a time)\n" +
            "JERKY/JITTERY: Decrease dXY (-0.01 at a time)\n" +
            "SLUGGISH: Decrease dXY\n\n" +
            "Watch: Motion should be smooth, no vibration\n" +
            "A = Run Path | B = Next Stage →"
        ),

        STAGE_4_D_ROTATION(
            "STAGE 4: Tune Rotation D Gain",
            "Tune: SixWheelPID.dR\n" +
            "Start: dR = pR * 0.5 (as starting point)\n\n" +
            "Goal: Smooth rotation without oscillation\n\n" +
            "WOBBLES SIDE-TO-SIDE: Increase dR\n" +
            "JERKY ROTATION: Decrease dR\n\n" +
            "Watch: Rotation should be smooth\n" +
            "A = Run Path | B = Next Stage →"
        ),

        STAGE_5_LOOKAHEAD(
            "STAGE 5: Tune Look-Ahead Distance",
            "Tune: SixWheelDrive.LOOK_AHEAD_DIST\n" +
            "Current: 5.0 inches (likely too small)\n\n" +
            "Goal: Smooth path following, especially at waypoints\n\n" +
            "JERKY AT WAYPOINTS: Increase (try 10-15)\n" +
            "CUTS CORNERS TOO MUCH: Decrease (try 6-8)\n\n" +
            "Typical range: 8-15 inches\n" +
            "A = Run Path | B = Next Stage →"
        ),

        STAGE_6_FINAL_TEST(
            "STAGE 6: Final Validation",
            "All gains should now be tuned!\n\n" +
            "Run multiple test paths and verify:\n" +
            "✓ Smooth motion (no jerking)\n" +
            "✓ No oscillations\n" +
            "✓ Reaches target accurately\n" +
            "✓ Good speed throughout\n\n" +
            "If issues remain, go back:\n" +
            "X = Previous Stage | A = Run Path\n\n" +
            "When satisfied, RECORD YOUR GAINS!"
        );

        private final String title;
        private final String instructions;

        TuningStage(String title, String instructions) {
            this.title = title;
            this.instructions = instructions;
        }

        public String getTitle() { return title; }
        public String getInstructions() { return instructions; }
    }

    private TuningStage currentStage;
    private Path currentPath;
    private boolean pathRunning;

    // Test paths
    private Point2d[][] testPaths;
    private int currentTestPathIndex = 0;

    @Override
    public void initialize() {
        robot.clear();
        addSixWheel();

        currentStage = TuningStage.STAGE_0_INTRO;
        pathRunning = false;

        // Create multiple test paths for comprehensive tuning
        testPaths = new Point2d[][] {
            // Path 1: Simple straight line
            {
                new Point2d(0, 0),
                new Point2d(TestPath.straightDistance, 0)
            },
            // Path 2: Gentle curve
            {
                new Point2d(0, 0),
                new Point2d(20, 10),
                new Point2d(20, 30)
            },
            // Path 3: Sharp turn
            {
                new Point2d(0, 0),
                new Point2d(30, 0),
                new Point2d(30, 30)
            },
            // Path 4: S-curve
            {
                new Point2d(0, 0),
                new Point2d(15, 15),
                new Point2d(0, 30),
                new Point2d(-15, 45)
            }
        };
    }

    @Override
    public void onStart() {
        sixWheel.setPosition(new Pose2d(0, 0, 0));
    }

    @Override
    public void periodic() {
        // Stage navigation
        if (driver1.pressedB()) {
            nextStage();
        }
        if (driver1.pressedX()) {
            previousStage();
        }

        // Reset position
        if (driver1.pressedY()) {
            sixWheel.setPosition(new Pose2d(0, 0, 0));
            telemetry.addLine("✓ Position reset to (0, 0, 0)");
        }

        // Run test path
        if (driver1.pressedA()) {
            runTestPath();
        }

        // Path execution
        if (currentPath != null) {
            currentPath.run();

            if (currentPath.isDone()) {
                telemetry.addLine("✓ Path completed!");
                currentPath = null;
                pathRunning = false;
            }
        }

        // Display current stage info
        displayStageInfo();

        // Display current gains
        displayCurrentGains();

        // Display relevant telemetry based on stage
        displayStageTelemetry();
    }

    private void nextStage() {
        TuningStage[] stages = TuningStage.values();
        int currentIndex = currentStage.ordinal();
        if (currentIndex < stages.length - 1) {
            currentStage = stages[currentIndex + 1];
            telemetry.speak("Stage " + (currentIndex + 1));
        }
    }

    private void previousStage() {
        TuningStage[] stages = TuningStage.values();
        int currentIndex = currentStage.ordinal();
        if (currentIndex > 0) {
            currentStage = stages[currentIndex - 1];
            telemetry.speak("Stage " + (currentIndex - 1));
        }
    }

    private void runTestPath() {
        if (pathRunning) {
            telemetry.addLine("⚠ Path already running!");
            return;
        }

        // Select appropriate path based on tuning stage
        Point2d[] selectedPath;
        String pathDescription;

        switch (currentStage) {
            case STAGE_1_P_LINEAR:
            case STAGE_3_D_LINEAR:
                // Use simple straight line for linear tuning
                selectedPath = testPaths[0];
                pathDescription = "Straight Line";
                break;

            case STAGE_2_P_ROTATION:
            case STAGE_4_D_ROTATION:
                // Use curved path for rotation tuning
                selectedPath = testPaths[1];
                pathDescription = "Gentle Curve";
                break;

            case STAGE_5_LOOKAHEAD:
                // Use sharp turn for look-ahead tuning (shows waypoint behavior)
                selectedPath = testPaths[2];
                pathDescription = "Sharp Turn";
                break;

            case STAGE_6_FINAL_TEST:
                // Final stage: cycle through all paths for comprehensive testing
                selectedPath = testPaths[3];
                pathDescription = getPathName(3);
                currentTestPathIndex = (currentTestPathIndex + 1) % testPaths.length;
                break;

            default:
                // Introduction stage - use simple straight line
                selectedPath = testPaths[0];
                pathDescription = "Straight Line";
                break;
        }

        // Build and start path
        currentPath = new org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder()
            .addMappedPurePursuitPath(selectedPath, TestPath.maxTime)
            .build()
            .start();

        pathRunning = true;
        telemetry.addLine("✓ Running: " + pathDescription);
    }

    private String getPathName(int index) {
        switch (index) {
            case 0: return "Straight Line";
            case 1: return "Gentle Curve";
            case 2: return "Sharp Turn";
            case 3: return "S-Curve";
            default: return "Test Path " + index;
        }
    }

    private String getPathForStage(TuningStage stage) {
        switch (stage) {
            case STAGE_1_P_LINEAR:
            case STAGE_3_D_LINEAR:
                return "Straight Line";
            case STAGE_2_P_ROTATION:
            case STAGE_4_D_ROTATION:
                return "Gentle Curve";
            case STAGE_5_LOOKAHEAD:
                return "Sharp Turn";
            case STAGE_6_FINAL_TEST:
                return "All Paths (cycles)";
            default:
                return "Straight Line";
        }
    }

    private void displayStageInfo() {
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine(currentStage.getTitle());
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine();

        // Split instructions by newline for better formatting
        String[] lines = currentStage.getInstructions().split("\n");
        for (String line : lines) {
            telemetry.addLine(line);
        }
        telemetry.addLine();
    }

    private void displayCurrentGains() {
        telemetry.addLine("─── Current Gains (FTC Dashboard) ───");

        // Get current PID instance
        org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit.SixWheelPID pid = sixWheel.getPID();

        telemetry.addData("pXY (Linear P)", "%.3f", pid.pXY);
        telemetry.addData("dXY (Linear D)", "%.3f", pid.dXY);
        telemetry.addData("pR (Rotation P)", "%.3f", pid.pR);
        telemetry.addData("dR (Rotation D)", "%.3f", pid.dR);
        telemetry.addData("Look-Ahead Dist", "%.1f in", sixWheel.getLookAheadDist());
        telemetry.addData("Stop Distance", "%.1f in", pid.STOP_DISTANCE);
        telemetry.addLine();
    }

    private void displayStageTelemetry() {
        telemetry.addLine("─── Live Telemetry ───");

        Pose2d robotPose = sixWheel.getPos();
        telemetry.addData("Robot Pos", "X=%.1f, Y=%.1f, H=%.1f°",
            robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getH()));

        if (pathRunning && currentPath != null) {
            telemetry.addData("Path Status", "RUNNING");

            // Stage-specific telemetry
            switch (currentStage) {
                case STAGE_1_P_LINEAR:
                    telemetry.addData("→ Linear Vel", "WATCH THIS");
                    telemetry.addData("→ Robot Speed", "Should be smooth");
                    break;

                case STAGE_2_P_ROTATION:
                    telemetry.addData("→ Rot Vel", "WATCH THIS");
                    telemetry.addData("→ Turning", "Should be smooth");
                    break;

                case STAGE_3_D_LINEAR:
                    telemetry.addData("→ Oscillation", "Should decrease");
                    telemetry.addData("→ Jitter", "Should not vibrate");
                    break;

                case STAGE_4_D_ROTATION:
                    telemetry.addData("→ Rotation Wobble", "Should be smooth");
                    break;

                case STAGE_5_LOOKAHEAD:
                    telemetry.addData("→ Waypoint Smoothness", "WATCH THIS");
                    telemetry.addData("→ Path Following", "Should be fluid");
                    break;

                case STAGE_6_FINAL_TEST:
                    telemetry.addData("→ Overall Motion", "Should be perfect!");
                    break;
            }
        } else {
            // Show which path will run when A is pressed
            String nextPath = getPathForStage(currentStage);
            telemetry.addData("Path Status", "IDLE");
            telemetry.addData("Next Path", nextPath + " (Press A)");
        }

        // Always show if D gains are disabled (important reminder)
        if (currentStage == TuningStage.STAGE_1_P_LINEAR ||
            currentStage == TuningStage.STAGE_2_P_ROTATION) {
            telemetry.addLine();
            telemetry.addLine("⚠ REMINDER: Set D gains to 0.0 in Dashboard!");
        }

        telemetry.addLine();
        telemetry.addLine("─── Controls ───");
        telemetry.addLine("A = Run Path | B = Next | X = Prev | Y = Reset Pos");
    }

}
