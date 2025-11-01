package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;

import java.util.ArrayList;
import java.util.List;

public class LimelightObeliskTagDetector implements BluSubsystem {
    Limelight3A limelight;
    String[] pattern;
    int greenIndex;
    Pose2d botpose;
    public LimelightObeliskTagDetector(){
        limelight = Globals.hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        pattern = new String[]{"p","p","p"};
        botpose = new Pose2d(0,0,0);
    }

    @Override
    public void init() {

    }

    @Override
    public void read() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult res: tags){
                int id = res.getFiducialId();
                if (id >= 21 && id <= 23){
                    //pattern id
                    greenIndex = id-21;
                    pattern[greenIndex] = "g";
                } else {
                    //location tag
                    Pose3D bot = result.getBotpose();
                    botpose = new Pose2d(bot.getPosition().x, bot.getPosition().y, bot.getOrientation().getYaw(AngleUnit.RADIANS));
                }
            }
            }
    }

    @Override
    public void write() {

    }

    @Override
    public void telemetry(Telemetry telemetry) {

    }

    public String[] getPattern(){
        return pattern;
    }

    public Pose2d getLLBotPose(){return botpose;}


    @Override
    public void reset() {

    }
}
