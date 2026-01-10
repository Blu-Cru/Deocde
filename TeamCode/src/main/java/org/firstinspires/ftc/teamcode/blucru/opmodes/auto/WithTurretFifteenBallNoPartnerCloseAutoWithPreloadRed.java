package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ResetForIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCloseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.blucru.opmodes.test.TurretLockOnGoalTest;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;


@Autonomous(name = "15 Ball Turret Close Auto With Preload No Partner Red", group = "auto")
public class WithTurretFifteenBallNoPartnerCloseAutoWithPreloadRed extends BluLinearOpMode {
    // TODO: Add trajectory sequence when rr package is configured
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;
    protected Alliance alliance = Alliance.RED;
    @Override
    public void initialize() {
        Globals.setAlliance(alliance);
        robot.clear();
        addShooter();
        addIntake();
        addTransfer();
        addElevator();
        addTurret();

        startPose = new Pose2d(-45, 52, Math.toRadians(127));

        drive = new TankDrive(hardwareMap, Globals.mapRRPose2d(startPose));
//        shooter.setRRPoseSupplier(() -> drive.localizer.getPose());
//        turret.setRRPoseSupplier(() -> drive.localizer.getPose());

        shooter.setHoodAngle(26);
        shooter.setMiddleHoodAngle(30);
        shooter.write();
        transfer.setAllMiddle();
        transfer.write();
        elevator.setDown();
        elevator.write();
        turret.resetEncoder();
        turret.write();

        path = drive.actionBuilder(Globals.mapRRPose2d(startPose))
                .setReversed(true)
                .splineTo(new Vector2d(-33, 40), Math.toRadians(0))
                .afterTime(0.1, new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(200),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(200),
                                new RightTransferUpCommand(),
                                new WaitCommand(500),
                                new CenterTurretCommand(),
                                new WaitCommand(500),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand(),
                                new ElevatorDownCommand()
                        ), false
                ))
                .waitSeconds(2) // SHOOT PRELOAD

                .setReversed(true)
                .splineTo(new Vector2d(-18, 45), Math.toRadians(0))
                .waitSeconds(2) // PICKUP FIRST SET
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand(),
                                new TurnTurretToPosCommand(60)
                        )
                ))

                .setReversed(false)
                .splineTo(new Vector2d(-30, 40), Math.toRadians(210))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(100),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(100),
                                new RightTransferUpCommand(),
                                new WaitCommand(500),
                                new CenterTurretCommand(),
                                new WaitCommand(500),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand(),
                                new ElevatorDownCommand()
                        ), false
                ))
                .waitSeconds(2) // SHOOT FIRST SET

                .setReversed(true)
                .splineTo(new Vector2d(5, 40), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand(),
                                new TurnTurretToPosCommand(60)
                        )
                ))
                .waitSeconds(2) // PICKUP SECOND SET

                .setReversed(false)
                .setTangent(Math.toRadians(180)) // Fixed: keeps path flat at Y=40
                .splineTo(new Vector2d(-25, 40), Math.toRadians(210))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(100),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(100),
                                new RightTransferUpCommand(),
                                new WaitCommand(500),
                                new CenterTurretCommand(),
                                new WaitCommand(500),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand(),
                                new ElevatorDownCommand()
                        ), false
                ))
                .waitSeconds(2) // SHOOT SECOND SET

                .setReversed(true)
                .splineTo(new Vector2d(-2, 50), Math.toRadians(90)) // Waypoint for stability
                .splineTo(new Vector2d(-2, 56), Math.toRadians(90)) // OPEN GATE
                .waitSeconds(1)

                .setReversed(false)
                .splineTo(new Vector2d(-7, 45), Math.toRadians(180))

                .setReversed(true)
                .splineTo(new Vector2d(30, 45), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand(),
                                new TurnTurretToPosCommand(60)
                        )
                ))
                .waitSeconds(2) // PICKUP THIRD SET

                .setReversed(false)
                .splineTo(new Vector2d(-30, 40), Math.toRadians(210))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(100),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(100),
                                new RightTransferUpCommand(),
                                new WaitCommand(500),
                                new CenterTurretCommand(),
                                new WaitCommand(500),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand(),
                                new ElevatorDownCommand()
                        ), false
                ))
                .waitSeconds(2) // SHOOT THIRD SET

                .setReversed(true)
                .splineTo(new Vector2d(53, 40), Math.toRadians(90))
                .splineTo(new Vector2d(53, 45), Math.toRadians(90))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new IntakeStopCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new TurnTurretToPosCommand(30)
                        )
                ))
                .waitSeconds(2) // PICKUP FOURTH SET

                .setReversed(false)
                .splineTo(new Vector2d(52.5, 13), Math.toRadians(270))
                .turnTo(Math.toRadians(160))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new LockOnGoalCommand(),
                                new AutoAimCommand(),
                                new AutonomousShootCommand()
                        )
                ))
                .waitSeconds(2)
                .build();


    }

    @Override
    public void onStart() {
        shooter.setHoodAngleIndependent(26, 26, 26); //orig 26 28 26 before switch to triple shot
        shooter.shootWithVelocity(900); //orig 850 before switching to triple shot
        turret.setAngle(45);
        TelemetryPacket packet = new TelemetryPacket();
        com.acmerobotics.dashboard.FtcDashboard dash = com.acmerobotics.dashboard.FtcDashboard.getInstance();
        while (opModeIsActive() && !isStopRequested() && path.run(packet)) {

            // Update FTCLib Subsystems
            robot.read();

            CommandScheduler.getInstance().run();
            robot.write();

            // 3. IMPORTANT: Send the packet to dashboard!
            // Without this, RoadRunner runs blind and you see no telemetry
            dash.sendTelemetryPacket(packet);

            // Reset packet for the next loop
            packet = new TelemetryPacket();

            // 4. Update standard telemetry to the driver station
            telemetry.addData("Path Running", "True");
            telemetry.update();

            idle();
        }
    }


    @Override
    public void periodic() {
        // If this auto is *just* the RR path, you can leave this empty.
        // If you want extra telemetry or non-RR logic during the match,
        // put it here.
    }
}
