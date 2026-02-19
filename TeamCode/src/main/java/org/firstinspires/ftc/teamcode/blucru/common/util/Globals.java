package org.firstinspires.ftc.teamcode.blucru.common.util;

import static org.firstinspires.ftc.teamcode.blucru.common.util.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Globals {
    public static HardwareMap hwMap;
    public static Telemetry telemetry;
    public static MultipleTelemetry multiTelemetry;
    public static double voltage = 13.0;
    public static Alliance alliance = RED;
    public static double leftGoalX = -58;
    public static double leftGoalY= -58;
    // "Pose2d" here refers to YOUR CUSTOM class (org.firstinspires...Pose2d)
    public static Pose2d startPose = new Pose2d(0, 0, Math.PI / 2);

    // FIX IS HERE: Changed these back to Custom "Vector2d" so Shooter.java works
    public static Vector2d shootingGoalLPose = new Vector2d(-56.75, -59.25);
    public static Vector2d shootingGoalRPose = new Vector2d(-56.75, 59.25);
    public static final Vector2d OGshootingGoalLPose = new Vector2d(-56.75, -59.25);
    public static final Vector2d OGshootingGoalRPose = new Vector2d(-56.75, 59.25);

    public static Vector2d lineVector = new Vector2d(3,4);

    public static Vector2d goalTagPoseBlue = new Vector2d(-58.3928021693, -55.628136098);
    public static Vector2d goalTagPoseRed = new Vector2d(-58.3928021693, 55.628136098);

    public static Vector2d turretTargetRedPose = new Vector2d(-66, 53);
    public static double turretTargetRedX = -65;
    public static double turretTargetRedY = 65;

    public static double turretTargetBlueX = -65;
    public static double turretTargetBlueY = -66;

    public static double OGturretTargetRedX = -65;
    public static double OGturretTargetRedY = 65;

    public static double OGturretTargetBlueX = -65;
    public static double OGturretTargetBlueY = -65;

    public static Vector2d turretTargetBluePose = new Vector2d(-66, -66);

    public static ElapsedTime matchTime;
    public static double defaultXYTol;
    public static double reflect = 1;

    public static double[] motors = {6000, 1620, 1150, 435, 312, 223, 117, 84, 60, 43, 30};
    public static double[] encoderResPerRPM = {28, 103.8, 145.1, 384.5, 537.7, 751.8, 1425.1, 1993.6, 2786.2, 3895.9, 5281.1};

    public static String pinpointName = "pinpoint";
    public static String flMotorName = "FL";
    public static String frMotorName = "FR";
    public static String blMotorName = "BL";
    public static String brMotorName = "BR";

    // =========================================================================
    // MAPPING METHODS
    // =========================================================================

    // 1. Returns CUSTOM Pose2d
    public static Pose2d mapPose(double x, double y, double heading) {
        y = y * reflect;
        heading = heading * reflect;
        return new Pose2d(x, y, heading % (2 * Math.PI));
    }

    // 2. Returns CUSTOM Pose2d from existing Custom Pose
    public static Pose2d mapPose(Pose2d pose) {
        return mapPose(pose.getX(), pose.getY(), pose.getH());
    }

    // 3. Returns ROADRUNNER Pose2d (Explicitly Typed)
    public static com.acmerobotics.roadrunner.Pose2d mapRRPose2d(com.acmerobotics.roadrunner.Pose2d pose2d) {
        return new com.acmerobotics.roadrunner.Pose2d(
                pose2d.position.x,
                pose2d.position.y * reflect,
                pose2d.heading.toDouble() * reflect
        );
    }

    // 4. Returns ROADRUNNER Vector2d (Explicitly Typed)
    public static com.acmerobotics.roadrunner.Vector2d mapRRVector(com.acmerobotics.roadrunner.Vector2d rrVector) {
        return new com.acmerobotics.roadrunner.Vector2d(rrVector.x, rrVector.y * reflect);
    }

    // 5. Returns CUSTOM Vector2d
    public static Vector2d mapVector(double x, double y) {
        y = y * reflect;
        return new Vector2d(x, y);
    }

    // 6. Map Angle
    public static double mapAngle(double angle) {
        return angle * reflect;
    }
    public static Vector2d mapPoint(Vector2d point) {
        return mapVector(point.getX(), point.getY());
    }
    // Assuming Point2d has .x and .y fields or getX()/getY() methods
    public static Point2d mapPoint(Point2d point) {
        // 1. Convert Point2d to the math values
        double newX = point.x;
        double newY = point.y * reflect; // Apply the alliance reflection

        // 2. Return a NEW Point2d
        return new Point2d(newX, newY);
    }
    // =========================================================================
    // DRAWING METHODS (Handling Both Types)
    // =========================================================================

    // Method A: Accepts ROADRUNNER Pose2d
    public static void drawPose(com.acmerobotics.roadrunner.Pose2d pose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();
        field.setStroke("#cf1d1d");
        field.strokeCircle(pose.position.x, pose.position.y, 9);
        com.acmerobotics.roadrunner.Vector2d v = pose.heading.vec().times(9);
        field.strokeLine(pose.position.x, pose.position.y, pose.position.x + v.x, pose.position.y + v.y);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // Method B: Accepts CUSTOM Pose2d (and converts it to RR for drawing)
    public static void drawPose(Pose2d pose) {
        // Convert Custom -> RR
        com.acmerobotics.roadrunner.Pose2d rrPose = new com.acmerobotics.roadrunner.Pose2d(pose.getX(), pose.getY(), pose.getH());
        drawPose(rrPose);
    }

    // =========================================================================
    // UTILITY METHODS
    // =========================================================================

    public static double getCorrectPower(double power) {
        return power * 12.0 / voltage;
    }

    public static void setAlliance(Alliance alliance) {
        Globals.alliance = alliance;
        reflect = alliance == RED ? 1 : -1;
    }

    public static void flipAlliance() {
        Globals.setAlliance(Globals.alliance.flip());
    }

    public static void updateVoltage(double voltage) {
        Globals.voltage = voltage;
    }

    public static double normalize(double angle) {
        return angle % (2.0 * Math.PI);
    }

    public static double convertServoPosToAngle(double range, double servoPos) {
        return (servoPos - 0.5) * range;
    }

    public static double convertAngleToServoPos(double range, double angle) {
        return (angle) / range + 0.5;
    }

    public static double convertMotorPositionToRotations(double rpm, double pos) {
        int i = 0;
        for (i = 0; i < motors.length; i++) {
            if (motors[i] == rpm) {
                break;
            }
        }
        double encoderRes = encoderResPerRPM[i];
        return pos / encoderRes;
    }
}
