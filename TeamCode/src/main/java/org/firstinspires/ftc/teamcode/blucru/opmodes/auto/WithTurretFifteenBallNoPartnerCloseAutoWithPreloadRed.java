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
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
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
                .splineTo(new Vector2d(-33, 47), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new AllTransferUpCommand(),
                                new WaitCommand(500),
                                new TurnTurretToPosCommand(0),
                                new WaitCommand(300),
                                new ElevatorDownCommand(),
                                new WaitCommand(200),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand()
                        )
                ))
                .waitSeconds(4) // SHOOT PRELOAD    `
                .splineTo(new Vector2d(-25, 47), Math.toRadians(0))// PICKUP FIRST SET
                .waitSeconds(2)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new IntakeStopCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new TurnTurretToPosCommand(30)
                        )
                ))
                .setReversed(false)
                .splineTo(new Vector2d(-33, 47), Math.toRadians(180))
                //SHOOT FIRST SET
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new TurnTurretToPosCommand(-30),
                                new WaitCommand(500),
                                new AllTransferUpCommand(),
                                new WaitCommand(300),
                                new CenterTurretCommand(),
                                new WaitCommand(300),
                                new ElevatorDownCommand(),
                                new WaitCommand(200),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand()

                        )
                ))
                .waitSeconds(2)

                .setReversed(true)
                .splineTo(new Vector2d(10, 47), Math.toRadians(0))
                //PICKUP SECOND SET
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new IntakeStopCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new TurnTurretToPosCommand(30)
                        )
                ))
                .waitSeconds(2)
                .setReversed(false)
                .splineTo(new Vector2d(-33, 47), Math.toRadians(180))
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new TurnTurretToPosCommand(-30),
                                new WaitCommand(500),
                                new AllTransferUpCommand(),
                                new WaitCommand(300),
                                new CenterTurretCommand(),
                                new WaitCommand(300),
                                new ElevatorDownCommand(),
                                new WaitCommand(200),
                                new AllTransferDownCommand(),
                                new IntakeStartCommand()

                        )
                ))
                .waitSeconds(2) // SHOOT SECOND SET

                .setReversed(true)
                .splineTo(new Vector2d(2, 53), Math.toRadians(90))

                .splineTo(new Vector2d(2, 56), Math.toRadians(90),
                        new TranslationalVelConstraint(10.0)) // OPEN GATE
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(-7, 45), Math.toRadians(180))

                .setReversed(true)
                .splineTo(new Vector2d(30, 47), Math.toRadians(0))
                .splineTo(new Vector2d(35, 47), Math.toRadians(0))  // PICKUP THIRD SET
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new IntakeStopCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new LockOnGoalCommand()
                        )
                ))
                .waitSeconds(2)
                .setReversed(true)
                .splineTo(new Vector2d(53, 13), Math.toRadians(-20))
                .waitSeconds(2) // SHOOT THIRD SET
                .turnTo(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(53,40), Math.toRadians(90))
                .splineTo(new Vector2d(53, 47), Math.toRadians(90))// PICKUP FOURTH SET
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new IntakeStopCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new LockOnGoalCommand()
                        )
                ))
                .waitSeconds(2)

                .setReversed(false)
                .splineTo(new Vector2d(52.5, 13), Math.toRadians(270))
                .turnTo(Math.toRadians(160))
                .stopAndAdd(
                        new FtclibCommandAction(
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
        shooter.shootWithVelocity(1000); //orig 850 before switching to triple shot
        turret.setAngle(30);
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
