package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;

@Autonomous(name = "Far auto blue (mirrored)", group = "auto")
public class TurretFarAutoBlue extends BluLinearOpMode {
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;
    protected Alliance alliance = Alliance.BLUE;

    @Override
    public void initialize() {
        Globals.setAlliance(alliance);
        robot.clear();
        addShooter();
        addIntake();
        addTransfer();
        addElevator();
        addTurret();

        // Mirror over x-axis: (x, y, heading) -> (x, -y, -heading)
        startPose = new Pose2d(60, -15, Math.toRadians(180));

        // NO mapRRPose2d
        drive = new TankDrive(hardwareMap, startPose);

        shooter.setHoodAngle(26);
        shooter.setMiddleHoodAngle(30);
        shooter.write();
        transfer.setAllMiddle();
        transfer.write();
        elevator.setDown();
        elevator.write();
        turret.resetEncoder();
        turret.write();

        // NO mapRRPose2d
        path = drive.actionBuilder(startPose)
                .setReversed(false)
                .splineTo(new Vector2d(57, -8), Math.toRadians(-200))
                .waitSeconds(1)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(500),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(500),
                                new RightTransferUpCommand(),
                                new WaitCommand(300),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(200),
                                new IdleShooterCommand(),
                                new CenterTurretCommand(),
                                new IntakeStartCommand(),
                                new WaitCommand(400),
                                new AllTransferDownCommand()
                        )
                ))
                .waitSeconds(1.5) // SHOOT PRELOAD

                .turnTo(Math.toRadians(-270))
                .setReversed(true)
                .splineTo(new Vector2d(62, -46), Math.toRadians(-75)) // INTAKE FIRST SET
                .waitSeconds(1)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new CenterTurretCommand(),
                                new IntakeSpitCommand(),
                                new WaitCommand(500),
                                new ElevatorUpCommand(),
                                new WaitCommand(400),
                                new ElevatorMiddleCommand(),
                                new ShootWithVelocityCommand(1450),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(700),
                                new IntakeStopCommand(),
                                new ParallelizeIntakeCommand(),
                                new TurnTurretToPosCommand(-60)
                        ), false
                ))
                .waitSeconds(0.2)
                .turnTo(Math.toRadians(-250))
                .setReversed(false)
                .lineToY(-20)
                .turnTo(Math.toRadians(-220))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(500),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(500),
                                new RightTransferUpCommand(),
                                new WaitCommand(300),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(200),
                                new IdleShooterCommand(),
                                new ElevatorDownCommand(),
                                new CenterTurretCommand(),
                                new IntakeStartCommand(),
                                new WaitCommand(400),
                                new AllTransferDownCommand()
                        )
                ))
                .waitSeconds(1.3) // SHOOT FIRST SET

                .turnTo(Math.toRadians(-270))
                .setReversed(true)
                .lineToY(-36)
                .turnTo(Math.toRadians(15))
                .lineToX(36)
                .waitSeconds(0.5)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new IntakeSpitCommand(),
                                new SetLeftHoodAngleCommand(42),
                                new SetMiddleHoodAngleCommand(43),
                                new SetRightHoodAngleCommand(42),
                                new WaitCommand(500),
                                new ElevatorUpCommand(),
                                new WaitCommand(500),
                                new ElevatorMiddleCommand(),
                                new ShootWithVelocityCommand(1450),
                                new WaitCommand(800),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand(),
                                new WaitCommand(500),
                                new ParallelizeIntakeCommand(),
                                new TurnTurretToPosCommand(-69)
                        ), false
                ))
                .waitSeconds(2)

                .turnTo(Math.toRadians(45))
                .setReversed(false)
                .splineTo(new Vector2d(45, -15), Math.toRadians(-270))
                .turnTo(Math.toRadians(-220))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(500),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(500),
                                new RightTransferUpCommand(),
                                new WaitCommand(300),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(200),
                                new IdleShooterCommand(),
                                new ElevatorDownCommand(),
                                new CenterTurretCommand(),
                                new IntakeStartCommand(),
                                new WaitCommand(400),
                                new AllTransferDownCommand()
                        )
                ))
                .waitSeconds(2) // SHOOT SECOND SET

                .turnTo(Math.toRadians(-270))
                .setReversed(true)
                .splineTo(new Vector2d(45, -46), Math.toRadians(-90))
                .waitSeconds(1) // INTAKE THIRD SET
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
//                                new ElevatorUpCommand(),
//                                new WaitCommand(500),
//                                new IntakeSpitCommand(),
//                                new ElevatorMiddleCommand(),
//                                new ShootWithVelocityCommand(1580),
//                                new WaitCommand(800),
//                                new AllTransferMiddleCommand(),
//                                new WaitCommand(300),
//                                new IntakeStopCommand(),
//                                new CenterTurretCommand()
//                                new TurnTurretToPosCommand(-60)
                        ), false
                ))
                .waitSeconds(1)//orig 2
//
//                .setReversed(false)
//                .splineTo(new Vector2d(48, -10), Math.toRadians(-270)) // SHOOT THIRD SET
//                .turnTo(Math.toRadians(-220))
//                .stopAndAdd(new FtclibCommandAction(
//                        new SequentialCommandGroup(
//                                new LeftTransferUpCommand(),
//                                new WaitCommand(300),
//                                new MiddleTransferUpCommand(),
//                                new WaitCommand(300),
//                                new RightTransferUpCommand(),
//                                new WaitCommand(300),
//                                new AllTransferMiddleCommand(),
//                                new WaitCommand(200),
//                                new IdleShooterCommand(),
//                                new CenterTurretCommand(),
//                                new IntakeStartCommand(),
//                                new WaitCommand(400),
//                                new AllTransferDownCommand()
//                        )
//                ))
//                .waitSeconds(2)

                .build();
    }

    @Override
    public void onStart() {
        turret.setAngle(-50);
        shooter.shootWithVelocity(1430);
        shooter.setHoodAngleIndependent(41, 43, 41);

        drive.lazyImu.get().resetYaw();
        drive.localizer.setPose(startPose);

        TelemetryPacket packet = new TelemetryPacket();
        com.acmerobotics.dashboard.FtcDashboard dash = com.acmerobotics.dashboard.FtcDashboard.getInstance();
        while (opModeIsActive() && !isStopRequested() && path.run(packet)) {
            robot.read();

            CommandScheduler.getInstance().run();
            robot.write();

            dash.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();

            telemetry.update();
            idle();
        }
    }

    @Override public void periodic() {}
    @Override public void telemetry() {}
}
