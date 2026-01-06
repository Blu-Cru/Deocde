package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

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
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;
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
                    new Point2d(-27, 44)
            }, 5000)
                    .waitMilliseconds(1000)
                    //SHOOT PRELOAD
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new MiddleTransferUpCommand(),
                                new RightTransferUpCommand(),
                                new WaitCommand(200),
                                new CenterTurretCommand(),
                                new WaitCommand(200),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand(),
                                new ElevatorDownCommand(),
                                new WaitCommand(200),
                                new CenterTurretCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1500)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-27, 44),
                            new Point2d(-10, 44)
                    }, 5000)
                    .waitMilliseconds(1500)
                    //INTAKE FIRST SET
                    .callback(() -> {
                        telemetry.addLine("Here");
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new IntakeStopCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new LockOnGoalCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1500)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-10, 42),
                            new Point2d(-27, 42)
                    }, 5000)
                    .waitMilliseconds(1500)
                    //SHOOT FIRST SET
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new MiddleTransferUpCommand(),
                                new RightTransferUpCommand(),
                                new WaitCommand(200),
                                new CenterTurretCommand(),
                                new WaitCommand(200),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand(),
                                new ElevatorDownCommand(),
                                new WaitCommand(200),
                                new CenterTurretCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-27, 42),
                            new Point2d(12.5,42)
                    }, 5000)
                    .waitMilliseconds(3000)
                    //INTAKE SECOND SET
                    .callback(() -> {
                        telemetry.addLine("Here");
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new IntakeStopCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new LockOnGoalCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(3000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(12.5, 42),
                            new Point2d(-27, 42)
                    }, 5000)
                    .waitMilliseconds(3000)
                    //SHOOT SECOND SET
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new MiddleTransferUpCommand(),
                                new RightTransferUpCommand(),
                                new WaitCommand(200),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand(),
                                new ElevatorDownCommand(),
                                new WaitCommand(200),
                                new CenterTurretCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(3000)
                    .callback(() -> {
                        telemetry.addLine("Path Ended");
                    })
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
    }

    public void onStart(){
        shooter.setHoodAngleIndependent(26, 28, 26);
        shooter.shootWithVelocity(850);
        turret.lockOnGoal();
        sixWheel.setPosition(new Pose2d(-45, 52, Math.toRadians(127+180)));
        currentPath = new TestingPath().build().start();
    }

    public void periodic(){
        currentPath.run();
    }


}
