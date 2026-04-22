package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import android.util.Log;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class AnalogBrushyTest extends BluLinearOpMode {

    ColorRangefinder crfA;
    ColorRangefinder crfB;

    AnalogInput analogA;
    double analogAVol;

    AnalogInput analogB;
    double analogBVol;

    @Override
    public void initialize(){
        crfA = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "ColorA"));
        crfB = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "ColorB"));

        crfA.setLedBrightness(100);
        crfB.setLedBrightness(100);

        crfA.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
        crfA.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0, 20);

        crfB.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
        crfB.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0, 20);

        analogA = hardwareMap.get(AnalogInput.class, "analogA");
        analogB = hardwareMap.get(AnalogInput.class, "analogB");

        enableDash();
        // Since we can't modify BluLinearOpMode, we override robot.telemetry to do nothing
        robot.clear(); // Ensure no subsystems are added
    }

    @Override
    public void periodic(){
        analogAVol = analogA.getVoltage();
        analogBVol = analogB.getVoltage();
        Log.i("Analog A", "Voltage: " + analogAVol + " Normalized: " + (analogAVol / 3.3 * 255));
        Log.i("Analog B", "Voltage: " + analogBVol + " Normalized: " + (analogBVol / 3.3 * 255));
    }

    @Override
    public void telemetry(){
        telemetry.addData("Analog A Voltage", analogAVol);
        telemetry.addData("Analog B Voltage", analogBVol);
        telemetry.addData("Analog A Normalized", analogAVol / 3.3 * 255);
        telemetry.addData("Analog B Normalized", analogBVol / 3.3 * 255);
    }
}
