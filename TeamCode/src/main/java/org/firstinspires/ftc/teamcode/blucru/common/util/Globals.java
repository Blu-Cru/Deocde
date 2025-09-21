package org.firstinspires.ftc.teamcode.blucru.common.util;

import static org.firstinspires.ftc.teamcode.blucru.common.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.blucru.common.util.Alliance.BLUE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Globals {
    public static HardwareMap hwMap;
    public static Telemetry telemetry;
    public static MultipleTelemetry multiTelemetry;
    public static double voltage = 13.0;
    public static Alliance alliance = RED;
    public static String pinpointName = "pinpoint";
    public static String flMotorName = "fl";
    public static String frMotorName = "fr";
    public static String blMotorName = "bl";
    public static String brMotorName = "br";
    public static Pose2d startPose = new Pose2d(0,0,Math.PI/2);

    public static ElapsedTime matchTime;

    public static double defaultXYTol;

    //1 is dont reflect
    public static double reflect = 1;

    public static double[] motors = {6000,1620,1150,435,312,223,117,84,60,43,30};
    public static double[] encoderResPerRPM = {28,103.8,145.1,384.5,537.7,751.8,1425.1,1993.6,2786.2,3895.9,5281.1};

    public static Pose2d mapPose(double x, double y, double heading){
        x = x*reflect;
        y = y*reflect;
        if (reflect < 0){
            heading += Math.PI;
        }

        return new Pose2d(x, y, heading % (2 * Math.PI));
    }

    public static void drawPose(Pose2d pose){
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay().setStroke("#cf1d1d");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public static Pose2d mapPose(Pose2d pose){
        return mapPose(pose.getX(), pose.getY(), pose.getH());
    }
    public static Vector2d mapPoint(Vector2d vec){
        return new Vector2d(vec.getX() * reflect, vec.getY()*reflect);
    }
    public static double getCorrectPower(double power){
        return power * 12.0/voltage;
    }
    public static void setAlliance(Alliance alliance){
        Globals.alliance = alliance;
    }
    public static void flipAlliance(){
        Globals.setAlliance(Globals.alliance.flip());
    }

    public static void updateVoltage(double voltage){
        Globals.voltage = voltage;
    }

    public static double normalize(double angle) {
        return angle % (2.0 * Math.PI);
    }
    public static double convertServoPosToAngle(double range, double servoPos){
        return (servoPos - 0.5) * range;
    }
    public static double convertAngleToServoPos(double range, double angle){
        return (angle)/range + 0.5;
    }

    public static double convertMotorPositionToRotations(double rpm, double pos){

        //find motor index
        int i = 0;
        for (i = 0; i<motors.length; i++){
            if (motors[i] == rpm){
                break;
            }
        }

        double encoderRes = encoderResPerRPM[i];

        return pos/encoderRes;
    }

}
