package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake;

import android.util.Size;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class TagCamera implements BluSubsystem, Subsystem {

    AprilTagProcessor tags;
    VisionPortal portal;
    AprilTagDetection detection;
    boolean currentlySeeingGoodTags;
    boolean streaming;
    final double tagDistToMiddleShooter = 8;
    public TagCamera(){
        int[] viewId = VisionPortal.makeMultiPortalView(1, VisionPortal.MultiPortalLayout.VERTICAL);
        tags = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawAxes(false)
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                .setLensIntrinsics(0,0,0,0)
                .build();
        portal = new VisionPortal.Builder()
                .setCamera(Globals.hwMap.get(WebcamName.class, "autoaim cam"))
                .enableLiveView(false)
                .addProcessor(tags)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
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
            ArrayList<AprilTagDetection> detections = tags.getDetections();
            for (AprilTagDetection detect : detections) {
                if ((detect.id == 20 && Globals.alliance == Alliance.BLUE)
                        || (detect.id == 24 && Globals.alliance == Alliance.RED)) {
                    currentlySeeingGoodTags = true;
                    detection = detect;
                    break;
                }
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
        return detectedThisLoop();
    }
    public double getTagDistToMiddleShooter(){
        return tagDistToMiddleShooter;
    }

    @Override
    public void reset() {
        disable();
    }
}
