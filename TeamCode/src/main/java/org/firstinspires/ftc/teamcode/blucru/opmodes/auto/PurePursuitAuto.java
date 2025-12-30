package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.PIDPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@Autonomous
public class PurePursuitAuto extends BluLinearOpMode {

    public class TestingPath extends SixWheelPIDPathBuilder{

        public TestingPath(){
            super();
            this.addPurePursuitPath(new Point2d[]{
                    new Point2d(-45, 52),
                    new Point2d(-20, 40)
            }, 5000);
        }
    }
    Path currentPath;

    public void initialize(){
        robot.clear();
        addSixWheel();
    }

    public void onStart(){
        sixWheel.setPosition(new Pose2d(-45, 52, Math.toRadians(127+180)));
        currentPath = new TestingPath().build().start();
    }

    public void periodic(){
        currentPath.run();
    }


}
