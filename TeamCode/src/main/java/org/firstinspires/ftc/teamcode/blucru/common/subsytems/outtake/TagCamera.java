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
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterAutoAimInterpolation;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.KalmanFilter;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotifPattern;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;
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
    final double tagDistToMiddleShooter = 7.5;
    final double turretCenterToLocPoint = -72.35/25.4;
    long captureTime;
    MotifPattern motifPattern;
    Pose2d botpose;
    Pose2d botposeOnTheMove;
    Pose2d kalmanFilteredBotpose;
    KalmanFilter xFilter, yFilter;
    final Pose2d TAG_20 = new Pose2d(-58, -58, Math.toDegrees(0));
    final Pose2d TAG_24 = new Pose2d(-58, 58, Math.toDegrees(0));


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
        captureTime = 0;
        botpose = null;
        botposeOnTheMove = null;
        kalmanFilteredBotpose = null;
        detection = null;
        xFilter = new KalmanFilter(Robot.getInstance().drivetrain.getCurrPose().getX(), 0.7,0.5,0.01,1);
        yFilter = new KalmanFilter(Robot.getInstance().drivetrain.getCurrPose().getX(), 0.7,0.5,0.01,1);

    }

    @Override
    public void init() {
        read();
        xFilter.setVal(Robot.getInstance().sixWheelDrivetrain.getPos().getX());
        yFilter.setVal(Robot.getInstance().sixWheelDrivetrain.getPos().getY());
    }

    @Override
    public void read() {
        currentlySeeingGoodTags = false;
        //using streaming first because it is a lot easier to get
        if (streaming && portal.getProcessorEnabled(tags)) {

            Globals.telemetry.addLine("Looking for tags");
            ArrayList<AprilTagDetection> detections = tags.getDetections();
            if (!detections.isEmpty()){
                Globals.telemetry.addLine("Detected Tag");
            }
            for (AprilTagDetection detect : detections) {
                captureTime = detect.frameAcquisitionNanoTime;
                if ((detect.id == 20 && Globals.alliance == Alliance.BLUE)
                        || (detect.id == 24 && Globals.alliance == Alliance.RED)) {
                    currentlySeeingGoodTags = true;
                    detection = detect;
                    Globals.telemetry.addData("Detect ID", detect.id);
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
                Pose2d tagPos = new Pose2d(0,0,0);
                if (detect.id == 20){
                    tagPos = TAG_20;
                } else {
                    tagPos = TAG_24;
                }
                Vector2d originToTag = tagPos.vec();
                double cameraFieldHeading = Robot.getInstance().sixWheelDrivetrain.getPos().getH() + Math.toRadians(Robot.getInstance().turret.getAngle()) + Math.PI;
                double angle = cameraFieldHeading;
                double dx = detect.ftcPose.x; // left/right relative to camera
                double dy = detect.ftcPose.y; // forward/back relative to camera
                Vector2d tagToCamCamCentric = new Vector2d(dx, dy);

                Globals.telemetry.addData("Tag To Cam Cam Centric", tagToCamCamCentric);
                Vector2d tagToCam = tagToCamCamCentric.rotate(-angle - Math.PI/2);
                tagToCam = new Vector2d(tagToCam.getX() * -1, tagToCam.getY());

                Globals.telemetry.addData("Tag To Cam", tagToCam);
                Vector2d camToTurret = new Vector2d(-tagDistToMiddleShooter,0).rotate(cameraFieldHeading);
                Globals.telemetry.addData("Cam To Turret", camToTurret);


                Vector2d turretToRobot = new Vector2d(turretCenterToLocPoint, 0).rotate(Robot.getInstance().sixWheelDrivetrain.getPos().getH());

                Vector2d tagToRobot = tagToCam.addNotInPlace(camToTurret).addNotInPlace(turretToRobot);
                Globals.telemetry.addData("Tag To Robot", tagToRobot);

                Vector2d originToRobot = originToTag.addNotInPlace(tagToRobot);

                botpose = new Pose2d(originToRobot, Robot.getInstance().sixWheelDrivetrain.getPos().getH());
                if (Robot.getInstance().positionHistory.getPoseAtTime(captureTime) == null) return;
                Vector2d oldVec = Robot.getInstance().positionHistory.getPoseAtTime(captureTime).getPose().vec();
                Vector2d offset = botpose.vec().subtractNotInPlace(oldVec);
                // now that we know offsets we can assume we havent changed off that much
                botposeOnTheMove = new Pose2d(Robot.getInstance().sixWheelDrivetrain.getPos().vec().addNotInPlace(offset),
                        Robot.getInstance().sixWheelDrivetrain.getPos().getH());
                xFilter.update(Robot.getInstance().sixWheelDrivetrain.getPos().getX(), botposeOnTheMove.getX());
                yFilter.update(Robot.getInstance().sixWheelDrivetrain.getPos().getY(), botposeOnTheMove.getY());
                //non-vector code
                /*double camToTagFieldX = dx * Math.cos(angle) - dy * Math.sin(angle);
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
                );*/
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
    public Pose2d getBotPosePoseHistory() {
        Vector2d oldVec = Robot.getInstance().positionHistory.getPoseAtTime(captureTime).getPose().vec();
        Vector2d offset = botpose.vec().subtractNotInPlace(oldVec);
        // now that we know offsets we can assume we havent changed off that much
        return new Pose2d(Robot.getInstance().sixWheelDrivetrain.getPos().vec().addNotInPlace(offset),
                Robot.getInstance().sixWheelDrivetrain.getPos().getH());
    }

    public Pose2d getBotpose(){
        return botpose;
    }
    public double getTagDistToMiddleShooter(){
        return tagDistToMiddleShooter;
    }
    public MotifPattern getMotif(){
        return motifPattern;
    }

    public double getDistance(){
        if (detection == null){
            return 0;
        }
        return detection.ftcPose.range * Math.cos(Math.toRadians(24)) + tagDistToMiddleShooter;
    }

    @Override
    public void reset() {
        disable();
    }

    public Pose2d getKalmanFilteredBotpose() {
        return new Pose2d(xFilter.get(),yFilter.get(), Robot.getInstance().sixWheelDrivetrain.getPos().getH());
    }
}
