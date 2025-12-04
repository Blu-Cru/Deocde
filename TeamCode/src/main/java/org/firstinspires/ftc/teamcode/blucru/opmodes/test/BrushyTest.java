package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorSensor;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class BrushyTest extends BluLinearOpMode {

    BluBrushlandLabsColorSensor brushlandLeft;
    BluBrushlandLabsColorSensor brushlandRight;

    @Override
    public void initialize(){
        brushlandLeft = new BluBrushlandLabsColorSensor("brushlandLeftPurple", "brushlandLeftGreen");
        brushlandRight = new BluBrushlandLabsColorSensor("brushlandRightPurple", "brushlandRightGreen");
        brushlandLeft.read();
        brushlandRight.read();
    }

    public void periodic(){
        brushlandLeft.read();
        brushlandRight.read();
    }

    public void telemetry(){
        telemetry.addData("Green Left?", brushlandLeft.greenBall());
        telemetry.addData("Green Right?", brushlandRight.greenBall());
        telemetry.addData("Purple Left?", brushlandLeft.purpleBall());
        telemetry.addData("Purple Right?", brushlandRight.purpleBall());
    }

}
