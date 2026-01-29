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
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
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
    double turretAngle = 105; //TODO: tune
    double shootVeloLeft = 1500; //TODO:tune
    double shootVeloMiddle = 1480;
    double shootVeloRight = 1500;
    double leftHood = 47;
    double middleHood = 45;
    double rightHood = 49;
    double pickupWallY = 57;

    public class TestingPath extends SixWheelPIDPathBuilder{

        public TestingPath(){
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(56, 22),
                            new Point2d(56, 23)
                    }, 100)
                    .waitMilliseconds(1000) //TODO: TUNE
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootFarCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(56, 23),
                            new Point2d(45,34),
                            //INTAKE FIRST SET
                            new Point2d(33, 38)//35,36 when measured with loco test
                    }, 5000)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood, turretAngle)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(35, 38),
                            //SHOOT FIRST SET
                            new Point2d(53, 22)
                    }, 5000)
                    .addTurnTo(90,1000)
                    .waitMilliseconds(400)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootFarCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(53,22),
                            //INTAKE SECOND SET
                            new Point2d(55,pickupWallY)
                    },1500)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood, turretAngle)
                        ).schedule();
                    })
                    .waitMilliseconds(500)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(55, pickupWallY),
                            //SHOOT SECOND SET
                            new Point2d(54, 22)
                    }, 3000)
                    .waitMilliseconds(400)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootFarCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(54,22),
                            //INTAKE THIRD SET
                            new Point2d(55,pickupWallY)
                    },1500)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood, turretAngle)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(55, pickupWallY),
                            //SHOOT THIRD SET
                            new Point2d(54, 22)
                    }, 5000)
                    .waitMilliseconds(400)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootFarCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(54,22),
                            //INTAKE FOURTH SET
                            new Point2d(55,pickupWallY)
                    },1500)
                    .callback(()->{
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood, turretAngle)                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(55, pickupWallY),
                            //SHOOT FOURTH SET
                            new Point2d(54, 22)
                    }, 5000)
                    .waitMilliseconds(400)
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
        shooter.setHoodAngleIndependent(47,45,49);
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
        shooter.shootWithVelocityIndependent(1500, 1480,1500);
        turret.setAngle(turretAngle);
        sixWheel.setPosition(new Pose2d(56, 22, Math.toRadians(90)));
        currentPath = new TestingPath().build().start();
    }

    public void periodic(){
        currentPath.run();
    }


}
