package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class BrushyTest extends BluLinearOpMode {

    BluBrushlandLabsColorSensor brushlandLeft;
    //BluBrushlandLabsColorSensor brushlandRight;

    @Override
    public void initialize(){
        brushlandLeft = new BluBrushlandLabsColorSensor("purpleLeftTop", "greenLeftTop");
        //brushlandRight = new BluBrushlandLabsColorSensor("brushlandRightPurple", "brushlandRightGreen");
        brushlandLeft.init();
        brushlandLeft.read();
        //brushlandRight.init();
        //brushlandRight.read();
    }

    public void periodic(){
        brushlandLeft.read();
        //brushlandRight.read();
    }

    public void telemetry(){
        telemetry.addData("Green Left?", brushlandLeft.greenBall());
        telemetry.addData("Purple Left?", brushlandLeft.purpleBall());
        telemetry.addData("Raw Pin0 (Purple)", brushlandLeft.getRawState1());
        telemetry.addData("Raw Pin1 (Green)", brushlandLeft.getRawState2());
        telemetry.addData("Ball Detected?", brushlandLeft.ballDetected());
        //telemetry.addData("Green Right?", brushlandRight.greenBall());
        //telemetry.addData("Purple Right?", brushlandRight.purpleBall());
    }

}
