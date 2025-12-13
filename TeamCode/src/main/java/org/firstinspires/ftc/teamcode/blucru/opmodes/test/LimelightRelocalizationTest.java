package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class LimelightRelocalizationTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addLLTagDetector();
        addSixWheel();
        addTurret();
    }

    public void periodic() {
        if (gamepad1.b) {
            telemetry.addLine("Updating Pos");
            sixWheel.setPosition(llTagDetector.getLLBotPose());
        }
    }

}
