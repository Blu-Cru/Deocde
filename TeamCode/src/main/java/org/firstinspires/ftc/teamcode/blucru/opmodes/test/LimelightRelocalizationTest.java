package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

import java.util.List;

@TeleOp
public class LimelightRelocalizationTest extends LinearOpMode {

    Limelight3A ll;
    DcMotorEx turretEncoder;
    public void runOpMode(){
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        turretEncoder = hardwareMap.get(DcMotorEx.class, Globals.blMotorName);
        turretEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        ll.setPollRateHz(100);
        ll.pipelineSwitch(0);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(138.5,94.05,DistanceUnit.MM);
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0));

        waitForStart();
        ll.start();

        while (opModeIsActive()){
            LLResult result = ll.getLatestResult();

            if (result != null && result.isValid()){
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags.isEmpty()){
                    telemetry.addLine("NO TAGS DETECTED");
                    continue;
                }
                for (LLResultTypes.FiducialResult res: tags){
                    int id = res.getFiducialId();
                    telemetry.addData("Tag ID", id);
                    if (id >= 21 && id <= 23){
                        //pattern id
                    } else {
                        //location tag
                        telemetry.addLine("here");
                        Pose3D bot = res.getRobotPoseFieldSpace();
                        Pose2d botpose = new Pose2d(bot.getPosition().x * 1000/25.4, bot.getPosition().y * 1000/25.4, bot.getOrientation().getYaw(AngleUnit.RADIANS));

                        telemetry.addData("Position", botpose);

                        //translating to my field coordinates
                        //botpose = new Pose2d(bot.getPosition().y * 1000 / 25.4, -bot.getPosition().x * 1000 / 25.4, bot.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI/2);
                    }
                }
            } else {
                telemetry.addData("Result", result);
                telemetry.addData("Result Valid?", result.isValid());
                telemetry.addLine("NO TAGS");
            }

            Pose2D botpose = pinpoint.getPosition();
            telemetry.addData("Pinpoint position", botpose);
            telemetry.update();
        }


    }

}
