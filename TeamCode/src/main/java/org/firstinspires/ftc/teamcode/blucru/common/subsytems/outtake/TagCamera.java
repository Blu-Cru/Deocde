package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake;

import android.util.Size;

import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterAutoAimInterpolation;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotifPattern;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

import java.util.ArrayList;

public class TagCamera implements BluSubsystem, Subsystem {
    AprilTagProcessor tags;
    VisionPortal portal;
    AprilTagDetection detection;
    boolean currentlySeeingGoodTags;
    boolean streaming;
    final double tagDistToMiddleShooter = 8;
    MotifPattern motifPattern;


    public TagCamera(){

        tags = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawAxes(false)
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                // Intrinsics from arducam-ov9281.cameramodel (mrcal, 320x240), scaled 2x for 640x480
                .setLensIntrinsics(563.115, 562.734, 312.667, 239.793)
                .setSuppressCalibrationWarnings(true)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(Globals.hwMap.get(WebcamName.class, "autoaim cam"))
                .enableLiveView(false)
                .addProcessor(tags)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStartStreamOnBuild(true)
                .build();
        motifPattern = MotifPattern.UNKNOWN;
        streaming = true;
    }

    @Override
    public void init() {
        read();
    }

    @Override
    public void read() {
        currentlySeeingGoodTags = false;
        //using streaming first because it is a lot easier to get
        if (streaming && portal.getProcessorEnabled(tags)) {

            Globals.telemetry.addLine("Looking for tags");
            ArrayList<AprilTagDetection> detections = tags.getDetections();
            for (AprilTagDetection detect : detections) {
                if ((detect.id == 20 && Globals.alliance == Alliance.BLUE)
                        || (detect.id == 24 && Globals.alliance == Alliance.RED)) {
                    currentlySeeingGoodTags = true;
                    detection = detect;
                    break;
                }
                if (detect.id == 21 && motifPattern == MotifPattern.UNKNOWN){
                    motifPattern = MotifPattern.GPP;
                    ShooterMotifCoordinator.setMotif(motifPattern);
                    break;
                }
                if (detect.id == 22 && motifPattern == MotifPattern.UNKNOWN){
                    motifPattern = MotifPattern.PGP;
                    ShooterMotifCoordinator.setMotif(motifPattern);
                    break;
                }
                if (detect.id == 23 && motifPattern == MotifPattern.UNKNOWN){
                    motifPattern = MotifPattern.PPG;
                    ShooterMotifCoordinator.setMotif(motifPattern);
                    break;
                }
                double tagFieldX = detection.metadata.fieldPosition.get(0);
                double tagFieldY = detection.metadata.fieldPosition.get(1);
                double cameraFieldHeading = Robot.getInstance().sixWheelDrivetrain.getPos().getH() + Robot.getInstance().turret.getAngle();
                double angle = cameraFieldHeading;
                double dx = detection.ftcPose.x; // left/right relative to camera
                double dy = detection.ftcPose.y; // forward/back relative to camera
                double camToTagFieldX = dx * Math.cos(angle) - dy * Math.sin(angle);
                double camToTagFieldY = dx * Math.sin(angle) + dy * Math.cos(angle);
                double cameraFieldX = tagFieldX - camToTagFieldX;
                double cameraFieldY = tagFieldY - camToTagFieldY;
                double turretOffsetX = 0.0;
                double turretOffsetY = 0.0;
                double rotatedOffsetX = turretOffsetX * Math.cos(Robot.getInstance().sixWheelDrivetrain.getPos().getH()) - turretOffsetY * Math.sin(Robot.getInstance().sixWheelDrivetrain.getPos().getH());
                double rotatedOffsetY = turretOffsetX * Math.sin(Robot.getInstance().sixWheelDrivetrain.getPos().getH()) + turretOffsetY * Math.cos(Robot.getInstance().sixWheelDrivetrain.getPos().getH());
                double robotFieldX = cameraFieldX - rotatedOffsetX;
                double robotFieldY = cameraFieldY - rotatedOffsetY;
                Robot.getInstance().sixWheelDrivetrain.setPosition(new Pose2d(robotFieldX, robotFieldY, Robot.getInstance().sixWheelDrivetrain.getPos().getH())
                );
            }

        }
    }

    public void disable(){
        if (streaming){
            portal.stopStreaming();
            streaming = false;
        }
    }
    public void enable(){
        if (!streaming){
            portal.resumeStreaming();
            streaming = true;
        }
    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }
    public AprilTagDetection getDetection(){
        return detection;
    }
    public boolean detectedThisLoop(){
        return currentlySeeingGoodTags;
    }
    public double getTagDistToMiddleShooter(){
        return tagDistToMiddleShooter;
    }
    public MotifPattern getMotif(){
        return motifPattern;
    }

    @Override
    public void reset() {
        disable();
    }
}
