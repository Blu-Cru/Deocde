package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorRangefinder;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class BrushyTest extends BluLinearOpMode {

    BluBrushlandLabsColorRangefinder brushlandLeft;

    @Override
    public void initialize(){
        brushlandLeft = new BluBrushlandLabsColorRangefinder("purpleLeftTop", "greenLeftTop");
        brushlandLeft.init();
        brushlandLeft.read();
        enableDash();
    }

    public void periodic(){
        brushlandLeft.read();
        Log.i("BRUSHLANDS","PIN 0:" + brushlandLeft.getRawState1());
        Log.i("BRUSHLANDS","PIN 1:" + brushlandLeft.getRawState2());
        //brushlandRight.read();
    }

    public void telemetry(){
        telemetry.addData("Green Left?", brushlandLeft.greenBall());
        telemetry.addData("Purple Left?", brushlandLeft.purpleBall());
        telemetry.addData("Raw Pin0 (Purple)", brushlandLeft.getRawState1());
        telemetry.addData("Raw Pin1 (Green)", brushlandLeft.getRawState2());
        telemetry.addData("Ball Detected?", brushlandLeft.ballDetected());
    }

}
