package org.firstinspires.ftc.teamcode.blucru.opmodes.test.brushlandStuff;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "test")
public class BrushyTest extends BluLinearOpMode {

    BluBrushlandLabsColorSensor brushlandLeft;
    //BluBrushlandLabsColorSensor brushlandRight;

    @Override
    public void initialize(){
        Globals.multiTelemetry = new MultipleTelemetry(telemetry);
        brushlandLeft = new BluBrushlandLabsColorSensor("brushlandLeftPurple", "brushlandLeftGreen");
        //brushlandRight = new BluBrushlandLabsColorSensor("brushlandRightPurple", "brushlandRightGreen");
        brushlandLeft.read();
        //brushlandRight.read();
    }

    public void periodic(){
        brushlandLeft.read();
        //brushlandRight.read();
    }

    public void telemetry(){
        Globals.multiTelemetry.addData("Green Left?", brushlandLeft.greenBall());
        //telemetry.addData("Green Right?", brushlandRight.greenBall());
        Globals.multiTelemetry.addData("Purple Left?", brushlandLeft.purpleBall());
        //telemetry.addData("Purple Right?", brushlandRight.purpleBall());
    }

}
