package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class LocalizerTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        robot.clear();
        addSixWheel();
    }

    public void onStart(){
        sixWheel.setPosition(new Pose2d(0,0,0));
    }

    public void periodic(){
        if (gamepad1.a){
            sixWheel.setPosition(new Pose2d(0,0,0));
        }
    }

    public void telemetry(){
        telemetry.addData("Pos", sixWheel.getPos().toString());
    }
}
