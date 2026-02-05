package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous
public class farRedPATH extends BluLinearOpMode {
    double closeTurretAngle = 30;

    public class TestingPath extends SixWheelPIDPathBuilder{

        public TestingPath(){
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(56, 22),
                            new Point2d(45,30),
                            //INTAKE FIRST SET
                            new Point2d(35, 38)//35,36 when measured with loco test
                    }, 5000)
                    .waitMilliseconds(500)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(35, 38),
                            //SHOOT FIRST SET
                            new Point2d(53, 22)
                    }, 5000)
                    .addTurnTo(90,1000)
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(53,22),
                            //INTAKE SECOND SET
                            new Point2d(55,57)
                    },5000)
                    .waitMilliseconds(500)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(55, 57),
                            //SHOOT SECOND SET
                            new Point2d(54, 22)
                    }, 5000)
                    .waitMilliseconds(1000)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(54,22),
                            //INTAKE THIRD SET
                            new Point2d(55,57)
                    },5000)
                    .waitMilliseconds(500)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(55, 57),
                            //SHOOT THIRD SET
                            new Point2d(54, 22)
                    }, 5000)
                    .waitMilliseconds(1000)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(54,22),
                            //INTAKE FOURTH SET
                            new Point2d(55,57)
                    },5000)
                    .waitMilliseconds(500)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(55, 57),
                            //SHOOT FOURTH SET
                            new Point2d(54, 22)
                    }, 5000)
                    .waitMilliseconds(1000)
                    .build();
        }
    }
    Path currentPath;

    public void initialize(){
        addSixWheel();
    }

    public void onStart(){
        sixWheel.setPosition(new Pose2d(56, 22, Math.toRadians(90)));
        currentPath = new TestingPath().build().start();
    }

    public void periodic(){
        currentPath.run();
    }


}
