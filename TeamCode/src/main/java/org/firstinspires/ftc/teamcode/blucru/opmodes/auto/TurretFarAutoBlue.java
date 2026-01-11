package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;
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
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosCommand;
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
                .splineTo(new Vector2d(55, -8), Math.toRadians(-220))
                .waitSeconds(1)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(300),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(300),
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
                .splineTo(new Vector2d(58, -46), Math.toRadians(-90)) // INTAKE FIRST SET
                .waitSeconds(1)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new WaitCommand(500),
                                new IntakeSpitCommand(),
                                new ElevatorMiddleCommand(),
                                new ShootWithVelocityCommand(1580),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(700),
                                new IntakeStopCommand(),
                                new TurnTurretToPosCommand(-56)
                        ), false
                ))
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(-260))
                .setReversed(false)
                .lineToY(-20)
                .turnTo(Math.toRadians(-220))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(300),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(300),
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
                .lineToY(-40)
                .turnTo(Math.toRadians(0))
                .lineToX(32)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new WaitCommand(500),
                                new IntakeSpitCommand(),
                                new ElevatorMiddleCommand(),
                                new ShootWithVelocityCommand(1580),
                                new WaitCommand(800),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand(),
                                new TurnTurretToPosCommand(-55)
                        ), false
                ))
                .waitSeconds(2)

                .turnTo(Math.toRadians(45))
                .setReversed(false)
                .splineTo(new Vector2d(45, -12), Math.toRadians(-270))
                .turnTo(Math.toRadians(-220))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(300),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(300),
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
                                new ElevatorUpCommand(),
                                new WaitCommand(500),
                                new IntakeSpitCommand(),
                                new ElevatorMiddleCommand(),
                                new ShootWithVelocityCommand(1580),
                                new WaitCommand(800),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand(),
                                new TurnTurretToPosCommand(-60)
                        ), false
                ))
                .waitSeconds(1)//orig 2

                .setReversed(false)
                .splineTo(new Vector2d(48, -10), Math.toRadians(-270)) // SHOOT THIRD SET
                .turnTo(Math.toRadians(-220))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(300),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(300),
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
                .waitSeconds(2)

                .build();
    }

    @Override
    public void onStart() {
        turret.setAngle(-70);
        shooter.shootWithVelocity(1560);
        shooter.setHoodAngleIndependent(37, 37, 37);

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
