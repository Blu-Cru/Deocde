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
        brushlandLeft = new BluBrushlandLabsColorSensor("purpleLeftBottom", "greenLeftBottom");
        //brushlandRight = new BluBrushlandLabsColorSensor("brushlandRightPurple", "brushlandRightGreen");
        brushlandLeft.init();
        brushlandLeft.read();
        //brushlandRight.init();
        //brushlandRight.read();
    }

    public void periodic(){
        brushlandLeft.read();
        //brushlandRight.read();
    }

    public void telemetry(){
        Globals.multiTelemetry.addData("Green Left?", brushlandLeft.greenBall());
        Globals.multiTelemetry.addData("Purple Left?", brushlandLeft.purpleBall());
        Globals.multiTelemetry.addData("Raw Pin0 (Purple)", brushlandLeft.getRawState1());
        Globals.multiTelemetry.addData("Raw Pin1 (Green)", brushlandLeft.getRawState2());
        Globals.multiTelemetry.addData("Ball Detected?", brushlandLeft.ballDetected());
        //telemetry.addData("Green Right?", brushlandRight.greenBall());
        //telemetry.addData("Purple Right?", brushlandRight.purpleBall());
    }

}
