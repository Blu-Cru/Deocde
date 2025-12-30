package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

/**
 * Test OpMode for the turnTo() function
 *
 * Tests turning to specific headings and validates the functionality
 *
 * CONTROLS:
 * - A: Turn to 0° (right)
 * - B: Turn to 90° (up)
 * - X: Turn to 180° (left)
 * - Y: Turn to -90° (down)
 * - DPAD_UP: Turn to 45°
 * - DPAD_RIGHT: Turn to -45°
 * - DPAD_LEFT: Turn to 135°
 * - DPAD_DOWN: Turn to -135°
 * - START: Reset position to (0, 0, 0°)
 */
@TeleOp(name = "TurnTo Test", group = "Test")
public class TurnToTest extends BluLinearOpMode {

    private enum TestState {
        IDLE,
        TURNING,
        COMPLETE
    }

    private TestState state;
    private double targetHeading;
    private long turnStartTime;

    @Override
    public void initialize() {
        robot.clear();
        addSixWheel();
        state = TestState.IDLE;
    }

    @Override
    public void onStart() {
        sixWheel.setPosition(new Pose2d(0, 0, 0));
        telemetry.addLine("✓ TurnTo Test Ready!");
        telemetry.addLine("Press buttons to test turning:");
        telemetry.addLine("A=0°, B=90°, X=180°, Y=-90°");
    }

    @Override
    public void periodic() {
        // Handle turn commands
        if (state == TestState.IDLE) {
            // Cardinal directions
            if (driver1.pressedA()) {
                startTurn(0.0, "RIGHT (0°)");
            } else if (driver1.pressedB()) {
                startTurn(90.0, "UP (90°)");
            } else if (driver1.pressedX()) {
                startTurn(180.0, "LEFT (180°)");
            } else if (driver1.pressedY()) {
                startTurn(-90.0, "DOWN (-90°)");
            }
            // Diagonal directions
            else if (driver1.pressedDpadUp()) {
                startTurn(45.0, "UP-RIGHT (45°)");
            } else if (driver1.pressedDpadRight()) {
                startTurn(-45.0, "DOWN-RIGHT (-45°)");
            } else if (driver1.pressedDpadLeft()) {
                startTurn(135.0, "UP-LEFT (135°)");
            } else if (driver1.pressedDpadDown()) {
                startTurn(-135.0, "DOWN-LEFT (-135°)");
            }
        }

        // Reset position
        if (driver1.pressedOptions()) {
            sixWheel.setPosition(new Pose2d(0, 0, 0));
            state = TestState.IDLE;
            telemetry.addLine("✓ Position reset!");
        }

        // Monitor turn progress
        if (state == TestState.TURNING) {
            if (sixWheel.isTurnComplete()) {
                state = TestState.COMPLETE;
                long duration = System.currentTimeMillis() - turnStartTime;
                telemetry.addLine("✓ Turn complete in " + duration + "ms");
                telemetry.speak("Turn complete");
            }
        }

        // Auto-return to idle after completion
        if (state == TestState.COMPLETE) {
            if (isAnyButtonPressed()) {
                state = TestState.IDLE;
            }
        }

        // Display telemetry
        displayStatus();
    }

    private void startTurn(double heading, String description) {
        targetHeading = heading;
        sixWheel.turnTo(heading);
        state = TestState.TURNING;
        turnStartTime = System.currentTimeMillis();
        telemetry.addLine("→ Turning to " + description);
        telemetry.speak("Turning");
    }

    private void displayStatus() {
        telemetry.addLine("═════ TurnTo Test ═════");
        telemetry.addLine();

        // Current pose
        Pose2d pose = sixWheel.getPos();
        double currentHeadingDeg = Math.toDegrees(pose.getH());
        telemetry.addData("Current Heading", "%.1f°", currentHeadingDeg);
        telemetry.addData("Position", "X=%.1f, Y=%.1f", pose.getX(), pose.getY());
        telemetry.addLine();

        // State-specific info
        switch (state) {
            case IDLE:
                telemetry.addLine("─── Ready to Turn ───");
                telemetry.addLine("Press any button to test turning");
                telemetry.addLine();
                telemetry.addLine("Cardinal Directions:");
                telemetry.addLine("  A = 0° (Right)");
                telemetry.addLine("  B = 90° (Up)");
                telemetry.addLine("  X = 180° (Left)");
                telemetry.addLine("  Y = -90° (Down)");
                telemetry.addLine();
                telemetry.addLine("Diagonal Directions:");
                telemetry.addLine("  DPAD_UP = 45°");
                telemetry.addLine("  DPAD_RIGHT = -45°");
                telemetry.addLine("  DPAD_LEFT = 135°");
                telemetry.addLine("  DPAD_DOWN = -135°");
                break;

            case TURNING:
                telemetry.addLine("─── Turning... ───");
                telemetry.addData("Target Heading", "%.1f°", targetHeading);

                // Calculate heading error
                double headingError = targetHeading - currentHeadingDeg;
                while (headingError > 180) headingError -= 360;
                while (headingError <= -180) headingError += 360;

                telemetry.addData("Error", "%.1f°", headingError);
                telemetry.addData("Abs Error", "%.1f°", Math.abs(headingError));
                telemetry.addData("Tolerance", "%.1f°", sixWheel.HEADING_TOLERANCE);

                // Progress indicator
                double progress = Math.max(0, 100 - Math.abs(headingError) * 2);
                telemetry.addData("Progress", "%.0f%%", progress);

                long elapsed = System.currentTimeMillis() - turnStartTime;
                telemetry.addData("Elapsed Time", "%d ms", elapsed);
                break;

            case COMPLETE:
                telemetry.addLine("─── ✓ Turn Complete! ───");
                telemetry.addData("Final Heading", "%.1f°", currentHeadingDeg);
                telemetry.addData("Target Was", "%.1f°", targetHeading);

                // Calculate final error
                double finalError = targetHeading - currentHeadingDeg;
                while (finalError > 180) finalError -= 360;
                while (finalError <= -180) finalError += 360;

                telemetry.addData("Final Error", "%.2f°", Math.abs(finalError));

                long totalTime = System.currentTimeMillis() - turnStartTime;
                telemetry.addData("Total Time", "%d ms", totalTime);
                telemetry.addLine();
                telemetry.addLine("Press any button to test again");
                break;
        }

        telemetry.addLine();
        telemetry.addLine("─── Tuning (FTC Dashboard) ───");
        telemetry.addData("pR", "%.3f", sixWheel.getPID().pR);
        telemetry.addData("dR", "%.3f", sixWheel.getPID().dR);
        telemetry.addData("Heading Tolerance", "%.1f°", sixWheel.HEADING_TOLERANCE);
        telemetry.addLine();
        telemetry.addLine("OPTIONS = Reset Position");
    }

    private boolean isAnyButtonPressed() {
        return driver1.pressedA() || driver1.pressedB() || driver1.pressedX() || driver1.pressedY() ||
               driver1.pressedDpadUp() || driver1.pressedDpadDown() ||
               driver1.pressedDpadLeft() || driver1.pressedDpadRight();
    }
}
