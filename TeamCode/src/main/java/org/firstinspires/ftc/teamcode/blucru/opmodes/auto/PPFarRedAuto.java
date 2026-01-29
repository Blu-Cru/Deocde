package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootFarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
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
public class PPFarRedAuto extends BluLinearOpMode {
    double turretAngle = 130; //TODO: tune
    double shootvelo = 1000; //TODO:tune
    double leftHood = 34;
    double middleHood = 34;
    double rightHood = 34;

    public class TestingPath extends SixWheelPIDPathBuilder{

        public TestingPath(){
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(56, 22),
                            new Point2d(56, 23)
                    }, 100)
                    .waitMilliseconds(2000) //TODO: TUNE
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootFarCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(56, 23),
                            new Point2d(45,30),
                            //INTAKE FIRST SET
                            new Point2d(35, 38)//35,36 when measured with loco test
                    }, 5000)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new AutonomousTransferCommand(shootvelo, leftHood, middleHood, rightHood, turretAngle)
                        ).schedule();
                    })
                    .waitMilliseconds(2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(35, 38),
                            //SHOOT FIRST SET
                            new Point2d(53, 22)
                    }, 5000)
                    .addTurnTo(90,1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootFarCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(2000)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(53,22),
                            //INTAKE SECOND SET
                            new Point2d(55,57)
                    },5000)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new AutonomousTransferCommand(shootvelo, leftHood, middleHood, rightHood, turretAngle)
                        ).schedule();
                    })
                    .waitMilliseconds(500)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(55, 57),
                            //SHOOT SECOND SET
                            new Point2d(54, 22)
                    }, 5000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootFarCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(2000)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(54,22),
                            //INTAKE THIRD SET
                            new Point2d(55,57)
                    },5000)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new AutonomousTransferCommand(shootvelo, leftHood, middleHood, rightHood, turretAngle)
                        ).schedule();
                    })
                    .waitMilliseconds(2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(55, 57),
                            //SHOOT THIRD SET
                            new Point2d(54, 22)
                    }, 5000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootFarCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(2000)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(54,22),
                            //INTAKE FOURTH SET
                            new Point2d(55,57)
                    },5000)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new AutonomousTransferCommand(shootvelo, leftHood, middleHood, rightHood, turretAngle)
                        ).schedule();
                    })
                    .waitMilliseconds(2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(55, 57),
                            //SHOOT FOURTH SET
                            new Point2d(54, 22)
                    }, 5000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootFarCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
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
        shooter.setHoodAngleIndependent(26, 26, 26); //orig 26 28 26 before switch to triple shot
        shooter.shootWithVelocity(1050); //orig 850 before switching to triple shot
        turret.setAngle(turretAngle);
        sixWheel.setPosition(new Pose2d(56, 22, Math.toRadians(90)));
        currentPath = new TestingPath().build().start();
    }

    public void periodic(){
        currentPath.run();
    }


}
