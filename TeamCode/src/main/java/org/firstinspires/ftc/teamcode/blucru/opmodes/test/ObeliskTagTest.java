package org.firstinspires.ftc.teamcode.blucru.opmodes.test;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.ObeliskTagDetector;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

import java.util.Arrays;

@TeleOp(group = "test")
public class ObeliskTagTest extends BluLinearOpMode {

    public void initialize(){
        addObeliskTagDetector();
    }

    public void periodic(){
        telemetry.addData("Pattern", Arrays.toString(obeliskTagDetector.getPattern()));
        telemetry.addData("Enabled?", obeliskTagDetector.enabled());

        if (driver1.pressedA()){
            obeliskTagDetector.stopTagDetection();
        }

        if (driver1.pressedB()){
            obeliskTagDetector.detectTags();
        }
    }

}
