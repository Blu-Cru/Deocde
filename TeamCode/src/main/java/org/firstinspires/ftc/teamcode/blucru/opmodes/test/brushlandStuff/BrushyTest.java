package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorRangefinder;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class BrushyTest extends BluLinearOpMode {

    BluBrushlandLabsColorRangefinder color;

    @Override
    public void initialize(){
        color = new BluBrushlandLabsColorRangefinder("six", "seven");
        color.init();
        color.read();
        enableDash();
        // Since we can't modify BluLinearOpMode, we override robot.telemetry to do nothing
        robot.clear(); // Ensure no subsystems are added
    }

    @Override
    public void periodic(){
        color.read();
        Log.i("BrushyTest", "Purple: " + color.purpleBall() + " Green: " + color.greenBall());
    }

    @Override
    public void telemetry(){
        telemetry.addData("purple", color.purpleBall());
        telemetry.addData("green", color.greenBall());
    }
}
