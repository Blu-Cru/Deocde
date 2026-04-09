package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorRangefinder;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class BrushyTest extends BluLinearOpMode {

    BluBrushlandLabsColorRangefinder brushlandLeftBottom;
    BluBrushlandLabsColorRangefinder brushlandLeftTop;

    @Override
    public void initialize(){
        brushlandLeftBottom = new BluBrushlandLabsColorRangefinder("purpleLeftBottom", "greenLeftBottom");
        brushlandLeftBottom.init();
        brushlandLeftBottom.read();
        brushlandLeftTop = new BluBrushlandLabsColorRangefinder("purpleLeftTop", "greenLeftTop");
        brushlandLeftTop.init();
        brushlandLeftTop.read();
        enableDash();
    }

    public void periodic(){
        brushlandLeftBottom.read();
        brushlandLeftTop.read();
        Log.i("BRUSHLAND LEFT BOTTOM","PIN 0:" + brushlandLeftBottom.getRawState1());
        Log.i("BRUSHLAND LEFT BOTTOM","PIN 1:" + brushlandLeftBottom.getRawState2());
        Log.i("BRUSHLAND LEFT Top","PIN 0:" + brushlandLeftTop.getRawState1());
        Log.i("BRUSHLAND LEFT Top","PIN 1:" + brushlandLeftTop.getRawState2());
    }

    public void telemetry(){
        telemetry.addData("Green LeftBottom?", brushlandLeftBottom.greenBall());
        telemetry.addData("Purple LeftBottom?", brushlandLeftBottom.purpleBall());
        telemetry.addData(" Bottom Raw Pin0 (Purple)", brushlandLeftBottom.getRawState1());
        telemetry.addData("Bottom Raw Pin1 (Green)", brushlandLeftBottom.getRawState2());
        telemetry.addData("BottomBall Detected?", brushlandLeftBottom.ballDetected());
        telemetry.addData("Green Left top?", brushlandLeftTop.greenBall());
        telemetry.addData("Purple Left Top?", brushlandLeftTop.purpleBall());
        telemetry.addData(" Top Raw Pin0 (Purple)", brushlandLeftTop.getRawState1());
        telemetry.addData("Top Raw Pin1 (Green)", brushlandLeftTop.getRawState2());
        telemetry.addData("Top Ball Detected?", brushlandLeftTop.ballDetected());
        telemetry.update();
    }

}
