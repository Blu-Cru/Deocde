package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
//@TeleOp
public class LoopTimeTest extends BluLinearOpMode {

    public void initialize(){

    }

    public void periodic(){
        sleep(100);
    }

    public void telemetry(){
        telemetry.addData("Match Time", Globals.matchTime.milliseconds());
    }
}
