package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorSensor;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class BrushyTest extends BluLinearOpMode {

    BluBrushlandLabsColorSensor brushlandLeft;
    DigitalChannel pin0, pin1;
    //BluBrushlandLabsColorSensor brushlandRight;

    @Override
    public void initialize(){
        brushlandLeft = new BluBrushlandLabsColorSensor("purpleLeftTop", "greenLeftTop");
        pin0 = hardwareMap.get(DigitalChannel.class, "purpleLeftTop");
        pin1 = hardwareMap.get(DigitalChannel.class, "greenLeftTop");
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);
        //brushlandRight = new BluBrushlandLabsColorSensor("brushlandRightPurple", "brushlandRightGreen");
        brushlandLeft.init();
        brushlandLeft.read();
        //brushlandRight.init();
        //brushlandRight.read();
        enableDash();
    }

    public void periodic(){
        brushlandLeft.read();
        Log.i("BRUSHLANDS","PIN 0:" + pin0.getState());
        Log.i("BRUSHLANDS","PIN 1:" + pin1.getState());
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
