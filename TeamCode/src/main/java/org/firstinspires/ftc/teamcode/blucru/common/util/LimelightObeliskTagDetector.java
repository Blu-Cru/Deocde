package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class LimelightObeliskTagDetector implements BluSubsystem, Subsystem {
    Limelight3A limelight;
    String[] pattern;
    int greenIndex;
    Pose2d botpose;
    double turretCenterToLimelightDist = 225.8324 / 25.4;
    double robotPosToTurretCenter = 72.35/25.4;
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
            if (tags.size() == 0){
                botpose = null;
                Globals.telemetry.addLine("NO TAGS DETECTED");
            }
            for (LLResultTypes.FiducialResult res: tags){
                int id = res.getFiducialId();
                if (id >= 21 && id <= 23){
                    //pattern id
                    greenIndex = id-21;
                    pattern[greenIndex] = "g";
                } else {
                    //location tag
                    Pose3D bot = res.getRobotPoseFieldSpace();
                    botpose = new Pose2d(bot.getPosition().x * 1000/25.4, bot.getPosition().y * 1000/25.4, bot.getOrientation().getYaw(AngleUnit.RADIANS));
                }
            }
        } else {
            Globals.telemetry.addLine("NO TAGS");
            botpose = null;
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
    public Vector2d getLLXY(){return botpose.vec();}
    public boolean validLLReads(){
        return botpose != null;
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


    @Override
    public void reset() {

    }
}
