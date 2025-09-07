package org.firstinspires.ftc.teamcode.blucru.common.util;

import android.graphics.Canvas;
import android.util.Log;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;

public class ObeliskTagDetector implements BluSubsystem {

    private static ObeliskTagDetector instance;
    private final String cameraName = "atagCam";

    private final int cameraResWidth = 1000;
    private final int cameraResHeight = 1000;
    VisionPortal obeliskTagPortal;

    AprilTagProcessor obeliskDetection;

    private String[] pattern = {"p","p","p"};
    int greenIndex;


    public static ObeliskTagDetector getInstance(){
        if (instance == null){
            instance = new ObeliskTagDetector();
        }

        return instance;
    }


    private ObeliskTagDetector(){
        int[] id = VisionPortal.makeMultiPortalView(1, VisionPortal.MultiPortalLayout.VERTICAL);

        obeliskDetection = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();


        obeliskTagPortal = new VisionPortal.Builder()
                .setCamera(Globals.hwMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(cameraResWidth, cameraResHeight))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(obeliskDetection)
                .setLiveViewContainerId(id[0])
                .build();

        obeliskTagPortal.setProcessorEnabled(obeliskDetection, true);

        //stop tag streaming bc uneccessary
        try{
            obeliskTagPortal.stopStreaming();
        } catch (Exception e){
            Log.e("AprilTags", "Could not stop obelisk tag portal streaming at init");
        }
    }

    public void detectTags(){
        obeliskTagPortal.resumeStreaming();

        //enable processor
        obeliskTagPortal.setProcessorEnabled(obeliskDetection, true);
    }

    @Override
    public void init() {
        read();
    }

    @Override
    public void read() {
        if (obeliskTagPortal.getProcessorEnabled(obeliskDetection)){
            ArrayList<AprilTagDetection> ids = obeliskDetection.getDetections();
            for (AprilTagDetection tag: ids){
                int id = tag.id;
                if (id > 21 && id < 23){
                    //tag is obelisk, set correct array pos
                    Globals.telemetry.addData("id", id);
                    greenIndex = id-21;
                    pattern[greenIndex] = "g";
                }
            }
        }
    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("pattern", Arrays.toString(pattern));
        telemetry.addData("streaming?", obeliskTagPortal.getProcessorEnabled(obeliskDetection));
    }

    @Override
    public void reset() {
        obeliskTagPortal.stopStreaming();
    }

    public String[] getPattern(){
        return pattern;
    }
    public int getGreenIndex(){
        return greenIndex;
    }
}
