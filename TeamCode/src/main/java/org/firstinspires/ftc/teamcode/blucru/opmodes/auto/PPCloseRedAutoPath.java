package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@Autonomous
public class PPCloseRedAutoPath extends BluLinearOpMode {
    double closeTurretAngle = 30;

    public class TestingPath extends SixWheelPIDPathBuilder{

        public TestingPath(){
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(-45, 52),
                            new Point2d(-10, 17)
                    }, 5000)
                    .waitMilliseconds(500)
                    //SHOOT PRELOAD
                    //INTAKE FIRST SET
//                    .addTurnTo(0, 1000)
                    .addTurnTo(90, 5000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10, 17),
                            new Point2d(-10, 46)
                    }, 5000)
                    .waitMilliseconds(500)
                    //HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10, 46),
                            new Point2d(-10, 17)
                    }, 5000)
                    //SHOOT FIRST SET
                    .waitMilliseconds(1000)

                    //INTAKE SECOND SET
                    .addTurnTo(70, 2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10, 17),
                            new Point2d(12.5,46)
                    }, 4000)
                    .waitMilliseconds(1000)

                    //HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(12.5, 46),
                            new Point2d(-10, 17)
                    }, 5000)
                    .waitMilliseconds(1000)
                    //SHOOT SECOND SET

                    //PICKUP THIRD SET
                    .addTurnTo(45,1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10,17),
                            new Point2d(37,46)
                    }, 5000)
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(37,46),
                            new Point2d(-10,17)
                    }, 5000)
                    .waitMilliseconds(1000)
                    //SHOOT THIRD SET
                    .build();
        }
    }
    Path currentPath;

    public void initialize(){
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
