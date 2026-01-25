package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import android.graphics.Point;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCloseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.PIDPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@Autonomous
public class PurePursuitAutoPath extends BluLinearOpMode {
    double closeTurretAngle = 30;

    public class TestingPath extends SixWheelPIDPathBuilder{

        public TestingPath(){
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(-45, 52),
                            new Point2d(-27, 44)
                    }, 5000)
                    .waitMilliseconds(500)
                    //SHOOT PRELOAD
                    //INTAKE FIRST SET
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-27, 44),
                            new Point2d(-10, 44)
                    }, 5000)
                    .waitMilliseconds(200)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10, 44),
                            new Point2d(0, 44),
                            new Point2d(4, 48)
                    }, 5000)
                    .waitMilliseconds(3000)
                    //HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-4, 48),
                            new Point2d(-4, 44),
                            new Point2d(-27, 44)
                    }, 5000)
                    .waitMilliseconds(1000)
                    //SHOOT FIRST SET

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-27, 44),
                            new Point2d(3,44)
                    }, 4000)
                    .addTurnTo(90, 2000)
                    //OPEN GATE
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(3, 44),
                            new Point2d(3,46)
                    }, 4000)
                    .waitMilliseconds(2000)


                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(3, 48),
                            new Point2d(3,44)
                    }, 4000)
                    .addTurnTo(0, 2000)
                    //INTAKE SECOND SET

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(3, 44),
                            new Point2d(12.5,44)
                    }, 4000)
                    .waitMilliseconds(1000)

                    //HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(12.5, 44),
                            new Point2d(-27, 44)
                    }, 5000)
                    .waitMilliseconds(500)
                    //SHOOT SECOND SET
//                    .a
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
