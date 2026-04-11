package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorRangefinder;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class AnalogBrushyTest extends BluLinearOpMode {

    AnalogInput pin0;
    double pin0vol;

    @Override
    public void initialize(){
        pin0 = hardwareMap.analogInput.get("purpleLeftTop");
        enableDash();
    }

    public void periodic(){
        pin0vol = pin0.getVoltage();

    }

    public void telemetry(){
        telemetry.addData("pin 0 voltage", pin0vol);
        telemetry.addData("hue", (pin0.getVoltage() / 3.3 * 360));
        telemetry.update();
    }

}
