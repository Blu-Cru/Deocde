package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousAntiJamTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootAntiJamCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@Autonomous
public class PPCloseRedAuto extends BluLinearOpMode {
    double turretAngleFirst = 30;
    double turretAngleSecond = 30;
    double turretAngleThird = 30;
    double leftHood;
    double middleHood;
    double rightHood;


    public class TestingPath extends SixWheelPIDPathBuilder{

        public TestingPath(){
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(-45, 52),
                            new Point2d(-10, 17)
                    }, 5000)
                    .waitMilliseconds(500)
                    //SHOOT PRELOAD
                    .callback(()->{
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    //INTAKE FIRST SET
//                    .addTurnTo(0, 1000)
                    .waitMilliseconds(200)
                    .addTurnTo(90, 5000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10, 17),
                            new Point2d(-10, 50)
                    }, 5000)
                    .callback(()->{
                        new SequentialCommandGroup(
                            new SetShooterVelocityIndependentCommand(1250, 1250, 1250),
                            new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                            new WaitCommand(300),
                            new TurnTurretToPosCommand(turretAngleSecond)
                        ).schedule();
                    })
                    .waitMilliseconds(500)
                    //HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10, 50),
                            new Point2d(-10, 17)
                    }, 5000)
                    //SHOOT FIRST SET

                    .waitMilliseconds(1000)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    //INTAKE SECOND SET
                    .addTurnTo(70, 2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10, 17),
                            new Point2d(12.5,46)
                    }, 4000)
                    .waitMilliseconds(1000)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(1250, 1250, 1250),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(300),
                                new TurnTurretToPosCommand(turretAngleSecond)

                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    //HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(12.5, 46),
                            new Point2d(-10, 17)
                    }, 5000)
                    .waitMilliseconds(1000)
                    //SHOOT SECOND SET
                    .callback(()->{
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    //PICKUP THIRD SET
                    .addTurnTo(45,1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10,17),
                            new Point2d(37,46)
                    }, 5000)
                    .waitMilliseconds(1000)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(1250, 1250, 1250),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(300),
                                new TurnTurretToPosCommand(turretAngleSecond)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(37,46),
                            new Point2d(-10,17)
                    }, 5000)
                    .waitMilliseconds(1000)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new AutonomousShootAntiJamCommand()
                        ).schedule();
                    })
                    //SHOOT THIRD SET
                    .build();
        }
    }
    Path currentPath;

    public void initialize(){
        robot.clear();
        addSixWheel();
        addIntake();
        addElevator();
        addShooter();
        addTurret();
        addTransfer();
        shooter.setHoodAngleIndependent(38, 36, 38);
        elevator.setMiddle();
        elevator.write();
        transfer.setAllMiddle();
        transfer.write();
        turret.resetEncoder();
        turret.write();
        sixWheel.reset();
        sixWheel.write();
        intake.resetEncoder();
    }

    public void onStart(){
        shooter.shootWithVelocity(1250); //orig 850 before switching to triple shot
        turret.setAngle(0);
        sixWheel.setPosition(new Pose2d(-45, 52, Math.toRadians(127+180)));
        currentPath = new TestingPath().build().start();
    }

    public void periodic(){
        currentPath.run();
    }


}
