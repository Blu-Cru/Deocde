package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.PurePursuitSegment;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp
public class PurePusuitTest extends BluLinearOpMode {
    Path currentPath;
    @Override
    public void initialize(){
        robot.clear();
        addSixWheel();
    }

    public void onStart(){
        sixWheel.setPosition(new Pose2d(0,0,0));
    }

    public void periodic(){
        if (driver1.pressedA()){
            currentPath = new TestPurePursuitPath().build().start();
        }
        if (currentPath != null){
            currentPath.run();
        }
    }

}
