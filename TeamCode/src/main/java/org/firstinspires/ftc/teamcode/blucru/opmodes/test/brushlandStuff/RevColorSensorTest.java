package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public class RevColorSensorTest extends BluLinearOpMode {
    RevColorSensorV3 colorSensorV3;

    public void initialize(){
        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, "Color");
    }

    public void periodic(){
        telemetry.addData("Dist", colorSensorV3.getDistance(DistanceUnit.MM));
        telemetry.addData("Red", colorSensorV3.getNormalizedColors().red);
        telemetry.addData("Blue", colorSensorV3.getNormalizedColors().blue);
        telemetry.addData("Green", colorSensorV3.getNormalizedColors().green);
        telemetry.update();
    }

}
