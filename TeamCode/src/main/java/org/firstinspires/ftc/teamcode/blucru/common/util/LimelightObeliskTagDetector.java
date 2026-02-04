package org.firstinspires.ftc.teamcode.blucru.common.util;

import android.util.Log;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.distribution.AbstractRealDistribution;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.ShooterMotifCoordinator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class LimelightObeliskTagDetector implements BluSubsystem, Subsystem {
    Limelight3A limelight;
    int greenIndex;
    Pose2d botpose;
    long captureTime;
    private final int POSITION_PIPELINE = 0;
    private final int PATTERN_PIPELINE = 1;
    private boolean validReadsThisLoop = false;
    private static boolean detectedMotif = false;
    private ElapsedTime timer;
    public LimelightObeliskTagDetector(){
        limelight = Globals.hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(POSITION_PIPELINE);
        botpose = null;
        timer = new ElapsedTime();
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void read() {
        //15 sec cooldown
        /*if (detectedPattern() || timer == null || timer.milliseconds() > 15000){
            timer = null;*/
            positionPipelineInterpretation();
        //} else {
            //patternPipelineInterpretation();
/*
            //need to check for pipeline switch
            if (detectedPattern() || timer == null || timer.milliseconds() > 15000){
                detectedMotif = true;
                limelight.pipelineSwitch(POSITION_PIPELINE);
            }
        }*/
    }

    public void positionPipelineInterpretation(){
        limelight.updateRobotOrientation(Math.toDegrees(Robot.getInstance().sixWheelDrivetrain.getPos().getH()));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            captureTime = result.getControlHubTimeStampNanos();
            Pose3D bot = result.getBotpose_MT2();
//            Globals.telemetry.addData("Limelight Heading", bot.getOrientation().getYaw(AngleUnit.RADIANS));
            //using same heading because tags are not for heading correction
            //multiplying by 1000/25.4 to account for unit change
            botpose = new Pose2d(bot.getPosition().x * 1000/25.4, bot.getPosition().y * 1000/25.4, Robot.getInstance().sixWheelDrivetrain.getPos().getH());
            validReadsThisLoop = true;
        } else {
//            Globals.telemetry.addData("Valid Reads", result.isValid());
//            Globals.telemetry.addLine("NO POSITION TAGS");
            validReadsThisLoop = false;
        }
    }

    public void patternPipelineInterpretation(){
        limelight.updateRobotOrientation(Math.toDegrees(Robot.getInstance().sixWheelDrivetrain.getPos().getH()));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> res = result.getFiducialResults();
            if (res.size() > 1){
                Log.e("Limelight", "Detected multiple pattern tags");
                //dont want to do anything with the data because its not good data
                return;
            }
            if (res.isEmpty()){
                //nothing in it, so just exit
                return;
            }
            //this loop should only run once, but preferring to use a loop here in case
            //having multiple pattern tags in view is fine
            for (LLResultTypes.FiducialResult tag:res){
                if (tag.getFiducialId() == 21){
                    ShooterMotifCoordinator.setMotif(MotifPattern.GPP);
                }
                if (tag.getFiducialId() == 22){
                    ShooterMotifCoordinator.setMotif(MotifPattern.PGP);
                }
                if (tag.getFiducialId() == 23){
                    ShooterMotifCoordinator.setMotif(MotifPattern.PPG);
                }
                detectedMotif = true;
            }
        } else {
//            Globals.telemetry.addLine("NO PATTERN TAGS");
        }
    }

    public long getTimeOfPhoto(){
        return captureTime;
    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }

    public boolean detectedPattern(){
        return detectedMotif;
    }

    public Pose2d getLLBotPosePoseHistory(){
        Vector2d oldVec = Robot.getInstance().positionHistory.getPoseAtTime(captureTime).getPose().vec();
        Vector2d offset = getLLXY().subtractNotInPlace(oldVec);
        //now that we know offsets we can assume we havent changed off that much
        return new Pose2d(Robot.getInstance().sixWheelDrivetrain.getPos().vec().addNotInPlace(offset), Robot.getInstance().sixWheelDrivetrain.getPos().getH());
    }
    public Pose2d getLLBotPose(){return botpose;}
    public Vector2d getLLXY(){return botpose.vec();}
    public boolean validLLReads(){
        return botpose != null;
    }
    public double getPipeline(){
        return limelight.getStatus().getPipelineIndex();
    }

    public double wrapAngle(double angle){
        while (angle <= -Math.PI){
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI){
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }
    public boolean hasUpdatedPosition(){
        return validReadsThisLoop;
    }

    public void switchToMotif(){
        timer = new ElapsedTime();
        timer.reset();
        limelight.pipelineSwitch(PATTERN_PIPELINE);
        detectedMotif = false;
    }

    public void switchToPosition(){
        timer = null;
        limelight.pipelineSwitch(POSITION_PIPELINE);
        detectedMotif = true;
    }

    @Override
    public void reset() {

    }
}
