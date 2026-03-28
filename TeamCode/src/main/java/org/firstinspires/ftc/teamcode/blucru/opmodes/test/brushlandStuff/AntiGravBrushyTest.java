package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "BrushyTest", group = "test")
public class AntiGravBrushyTest extends LinearOpMode {

    private DigitalChannel pin0; // purple
    private DigitalChannel pin1; // green

    @Override
    public void runOpMode() throws InterruptedException {
        // We look for pins named "pin0" and "pin1" in the FTC Dashboard Hardware Map configuration
        pin0 = hardwareMap.get(DigitalChannel.class, "purpleLeftBottom");
        pin1 = hardwareMap.get(DigitalChannel.class, "greenLeftBottom");

        // Important: Always set custom digital pins to inputs properly before grabbing state 
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized, waiting for Start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read from the sensor 
            boolean isPurple = pin0.getState();
            boolean isGreen = pin1.getState();

            // Output data into telemetry
            telemetry.addData("Purple Pin (pin0)", isPurple);
            telemetry.addData("Green Pin (pin1)", isGreen);
            telemetry.update();
        }
    }
}
