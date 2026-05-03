package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorRangefinder;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class AnalogBrushyTest extends BluLinearOpMode {

    BluBrushlandLabsColorRangefinder colorA;
    BluBrushlandLabsColorRangefinder colorB;

    AnalogInput analogA;
    double analogAVol;

    AnalogInput analogB;
    double analogBVol;

    @Override
    public void initialize(){
        colorA = new BluBrushlandLabsColorRangefinder("ColorAPurple", "ColorAGreen");
        colorA.init();
        colorA.read();

        colorB = new BluBrushlandLabsColorRangefinder("ColorBPurple", "ColorBGreen");
        colorB.init();
        colorB.read();

        analogA = hardwareMap.get(AnalogInput.class, "analogA");
        analogB = hardwareMap.get(AnalogInput.class, "analogB");

        enableDash();
        // Since we can't modify BluLinearOpMode, we override robot.telemetry to do nothing
        robot.clear(); // Ensure no subsystems are added
    }

    @Override
    public void periodic(){
        colorA.read();
        colorB.read();
        analogAVol = analogA.getVoltage();
        analogBVol = analogB.getVoltage();
        Log.i("Analog A", "Voltage: " + analogAVol + " Normalized: " + (analogAVol / 3.3 * 255)
                + " Purple: " + colorA.purpleBall() + " Green: " + colorA.greenBall());
        Log.i("Analog B", "Voltage: " + analogBVol + " Normalized: " + (analogBVol / 3.3 * 255)
                + " Purple: " + colorB.purpleBall() + " Green: " + colorB.greenBall());
    }

    @Override
    public void telemetry(){
        telemetry.addData("Analog A Voltage", analogAVol);
        telemetry.addData("Analog A Normalized", analogAVol / 3.3 * 255);
        telemetry.addData("purple A", colorA.purpleBall());
        telemetry.addData("green A", colorA.greenBall());
        telemetry.addData("raw state pin 1 A", colorA.getRawState2());

        telemetry.addData("Analog B Voltage", analogBVol);
        telemetry.addData("Analog B Normalized", analogBVol / 3.3 * 255);
        telemetry.addData("purple B", colorB.purpleBall());
        telemetry.addData("green B", colorB.greenBall());
        telemetry.addData("raw state pin 1 B", colorB.getRawState2());
    }
}
