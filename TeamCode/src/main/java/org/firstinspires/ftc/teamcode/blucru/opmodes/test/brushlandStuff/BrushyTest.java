package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorRangefinder;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class BrushyTest extends BluLinearOpMode {

    BluBrushlandLabsColorRangefinder colorA;
    BluBrushlandLabsColorRangefinder colorB;
    BluBrushlandLabsColorRangefinder colorC;

    @Override
    public void initialize(){
        colorA = new BluBrushlandLabsColorRangefinder("ColorAPurple", "ColorAGreen");
        colorA.init();
        colorA.read();

        colorB = new BluBrushlandLabsColorRangefinder("ColorBPurple", "ColorBGreen");
        colorB.init();
        colorB.read();

        colorC = new BluBrushlandLabsColorRangefinder("ColorCPurple", "ColorCGreen");
        colorC.init();
        colorC.read();

        enableDash();
        // Since we can't modify BluLinearOpMode, we override robot.telemetry to do nothing
        robot.clear(); // Ensure no subsystems are added
    }

    @Override
    public void periodic(){
        colorA.read();
        colorB.read();
        colorC.read();
        Log.i("BrushyTest", "Color A Purple: " + colorA.purpleBall() + " Green: " + colorA.greenBall());
        Log.i("BrushyTest", "Color B Purple: " + colorB.purpleBall() + " Green: " + colorB.greenBall());
        Log.i("BrushyTest", "Color C Purple: " + colorC.purpleBall() + " Green: " + colorC.greenBall());
    }

    @Override
    public void telemetry(){
        telemetry.addData("purple A", colorA.purpleBall());
        telemetry.addData("green A", colorA.greenBall());
        telemetry.addData("raw state pin 1 A: ", colorA.getRawState2());
        telemetry.addData("purple B", colorB.purpleBall());
        telemetry.addData("green B", colorB.greenBall());
        telemetry.addData("raw state pin 1 B: ", colorB.getRawState2());
        telemetry.addData("purple C", colorC.purpleBall());
        telemetry.addData("green C", colorC.greenBall());
        telemetry.addData("raw state pin 1 C: ", colorC.getRawState2());
    }
}
