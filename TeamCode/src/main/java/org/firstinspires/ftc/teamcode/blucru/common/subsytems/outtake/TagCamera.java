package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake;

import android.util.Size;

import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class TagCamera implements BluSubsystem, Subsystem {
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;

    AprilTagProcessor tags;
    VisionPortal portal;
    AprilTagDetection detection;
    boolean currentlySeeingGoodTags;
    boolean computedBotposeThisLoop;
    boolean streaming;
    boolean exposureSet = false;  // one-shot flag for exposure init
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
                .setCameraPose(new Position(),new YawPitchRollAngles(AngleUnit.DEGREES,0,20, 0,0))
                .build();
        // Decimation speeds up detection by downsampling internally
        // 2 = process at half res, good balance of speed vs range
        tags.setDecimation(2);

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

        xFilter = new KalmanFilter(0, 0.7,0.5,0.01,1);
        yFilter = new KalmanFilter(0, 0.7,0.5,0.01,1);
    }

    /**
     * Call after the camera has started streaming to lock exposure.
     * Low exposure (6ms) eliminates motion blur when the turret is rotating.
     */
    public void setManualExposure(int exposureMs, int gain) {
        // Wait for camera to be streaming
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);

        GainControl gainControl = portal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    @Override
    public void init() {
        read();
        xFilter.setVal(Robot.getInstance().sixWheelDrivetrain.getPos().getX());
        yFilter.setVal(Robot.getInstance().sixWheelDrivetrain.getPos().getY());
    }

    @Override
    public void read() {
        // One-shot: lock exposure the first time camera is streaming
        if (!exposureSet && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            setManualExposure(7, 1);

            exposureSet = true;
        }

        //using streaming first because it is a lot easier to get
        if (streaming && portal.getProcessorEnabled(tags)) {
            currentlySeeingGoodTags = false;
            computedBotposeThisLoop = false;
            detection = null;

            Globals.telemetry.addLine("Looking for tags");
            ArrayList<AprilTagDetection> detections = tags.getDetections();
            if (!detections.isEmpty()) {
                Globals.telemetry.addLine("Detected Tag");
            }

            AprilTagDetection goalDetection = null;
            for (AprilTagDetection detect : detections) {
                if (detect.ftcPose == null) continue;
                if (goalDetection == null && isAllianceGoalTag(detect)) {
                    goalDetection = detect;
                }
                if (detect.id == 21 && motifPattern == MotifPattern.UNKNOWN){
                    motifPattern = MotifPattern.GPP;
                    ShooterMotifCoordinator.setMotif(motifPattern);
                    continue;
                }
                if (detect.id == 22 && motifPattern == MotifPattern.UNKNOWN){
                    motifPattern = MotifPattern.PGP;
                    ShooterMotifCoordinator.setMotif(motifPattern);
                    continue;
                }
                if (detect.id == 23 && motifPattern == MotifPattern.UNKNOWN){
                    motifPattern = MotifPattern.PPG;
                    ShooterMotifCoordinator.setMotif(motifPattern);
                    continue;
                }
            }

            if (goalDetection != null) {
                captureTime = goalDetection.frameAcquisitionNanoTime;
                currentlySeeingGoodTags = true;
                detection = goalDetection;
                updateBotposeFromGoalTag(goalDetection);
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
    public boolean computedBotposeThisLoop(){
        return computedBotposeThisLoop;
    }
    public Pose2d getBotPosePoseHistory() {
        if (botpose == null) return null;
        if (Robot.getInstance().positionHistory.getPoseAtTime(captureTime) == null) return null;
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
        if (detection == null || detection.ftcPose == null){
            return 0;
        }
        return detection.ftcPose.range + tagDistToMiddleShooter;
    }

    @Override
    public void reset() {
        disable();
    }

    public Pose2d getKalmanFilteredBotpose() {
        return new Pose2d(xFilter.get(),yFilter.get(), Robot.getInstance().sixWheelDrivetrain.getPos().getH());
    }

    private void updateBotposeFromGoalTag(AprilTagDetection detect) {
        Pose2d tagPos = detect.id == BLUE_GOAL_TAG_ID ? TAG_20 : TAG_24;
        Vector2d originToTag = tagPos.vec();
        double cameraFieldHeading = Robot.getInstance().sixWheelDrivetrain.getPos().getH()
                + Math.toRadians(Robot.getInstance().turret.getAngle()) + Math.PI;
        double dx = detect.ftcPose.x; // left/right relative to camera
        double dy = detect.ftcPose.y; // forward/back relative to camera
        Vector2d tagToCamCamCentric = new Vector2d(dx, dy);

        Vector2d tagToCam = tagToCamCamCentric.rotate(-cameraFieldHeading - Math.PI / 2);
        tagToCam = new Vector2d(tagToCam.getX() * -1, tagToCam.getY());

        Vector2d camToTurret = new Vector2d(-tagDistToMiddleShooter, 0).rotate(cameraFieldHeading);
        Vector2d turretToRobot = new Vector2d(turretCenterToLocPoint, 0)
                .rotate(Robot.getInstance().sixWheelDrivetrain.getPos().getH());

        Vector2d tagToRobot = tagToCam.addNotInPlace(camToTurret).addNotInPlace(turretToRobot);
        Vector2d originToRobot = originToTag.addNotInPlace(tagToRobot);

        botpose = new Pose2d(originToRobot, Robot.getInstance().sixWheelDrivetrain.getPos().getH());

        if (Robot.getInstance().positionHistory.getPoseAtTime(captureTime) == null) {
            return;
        }

        computedBotposeThisLoop = true;
        Vector2d oldVec = Robot.getInstance().positionHistory.getPoseAtTime(captureTime).getPose().vec();
        Vector2d offset = botpose.vec().subtractNotInPlace(oldVec);
        botposeOnTheMove = new Pose2d(Robot.getInstance().sixWheelDrivetrain.getPos().vec().addNotInPlace(offset),
                Robot.getInstance().sixWheelDrivetrain.getPos().getH());
        xFilter.update(Robot.getInstance().sixWheelDrivetrain.getPos().getX(), botposeOnTheMove.getX());
        yFilter.update(Robot.getInstance().sixWheelDrivetrain.getPos().getY(), botposeOnTheMove.getY());
    }

    private boolean isAllianceGoalTag(AprilTagDetection detect) {
        if (detect == null) {
            return false;
        }

        return detect.id == getAllianceGoalTagId();
    }

    private int getAllianceGoalTagId() {
        return Globals.alliance == Alliance.BLUE ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;
    }
}
