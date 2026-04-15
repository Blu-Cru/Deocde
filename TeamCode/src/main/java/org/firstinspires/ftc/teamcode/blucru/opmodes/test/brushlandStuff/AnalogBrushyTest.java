package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class AnalogBrushyTest extends BluLinearOpMode {

    AnalogInput pin0;
    double pin0vol;

    @Override
    public void initialize(){
        pin0 = hardwareMap.analogInput.get("analogtest");
        enableDash();
        // Since we can't modify BluLinearOpMode, we override robot.telemetry to do nothing
        robot.clear(); // Ensure no subsystems are added
    }

    @Override
    public void periodic(){
        pin0vol = pin0.getVoltage();
        Log.i("AnalogBrushyTest", "Voltage: " + pin0vol + " Green: " + (pin0vol / 3.3 * 255));
    }

    @Override
    public void telemetry(){
        telemetry.addData("pin 0 voltage", pin0vol);
        telemetry.addData("Green", (pin0vol / 3.3 * 255));
    }
}
