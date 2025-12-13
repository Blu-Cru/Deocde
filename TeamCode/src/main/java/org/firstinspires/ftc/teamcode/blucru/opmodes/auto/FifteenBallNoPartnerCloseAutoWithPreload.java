package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

// your own commands:
import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;


@Autonomous(name = "15 Ball Close Auto With Preload No Partner", group = "auto")
public class FifteenBallNoPartnerCloseAutoWithPreload extends BluLinearOpMode {
    // TODO: Add trajectory sequence when rr package is configured
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;

    @Override
    public void initialize() {
        manageRobotLoop=false;

        addShooter();
        addIntake();
        addTransfer();
        addElevator();
        addTurret();
        Command pickupBalls = new SequentialCommandGroup(
                new IntakeCommand(),
                new WaitCommand(500),
                new TransferCommand(true)
        );

        startPose = new Pose2d(-45, 52, Math.toRadians(127));

        drive = new TankDrive(hardwareMap, startPose);
        transfer.setAllMiddle();
        elevator.setDown();
        elevator.write();

        path = drive.actionBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-28, 38), Math.toRadians(150+180))
                //.lineToX(-44)
                .stopAndAdd(new FtclibCommandAction(new ShootWithVelocityCommand(850)))
                .afterTime(0.1, new FtclibCommandAction(new CenterTurretCommand()))
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCommand()))//SHOOT PRELOAD
                .waitSeconds(3) // SHOOT PRELOAD
                .turnTo(Math.toRadians(-90))
                .setReversed(true)
                .afterTime(0.1, new FtclibCommandAction(new SequentialCommandGroup(new IntakeStartCommand(), new ElevatorDownCommand(), new CenterTurretCommand())))
                .splineTo(new Vector2d(-20, 47), Math.toRadians(0))  // PICKUP FIRST SET
                .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  // PICKUP FIRST SET
                .waitSeconds(2)
                .stopAndAdd(new FtclibCommandAction(new AutonomousTransferCommand(850, 26, 28, 26)))
                .setReversed(false)
                .turnTo(Math.toRadians(200))

                .splineTo(new Vector2d(-28, 38), Math.toRadians(135))
                .waitSeconds(2)
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCommand())) //SHOOT FIRST SET

                .setReversed(true)
                .splineTo(new Vector2d(0, 47), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeStartCommand(),
                        new ElevatorDownCommand(),
                        new CenterTurretCommand()
                )))
                .splineTo(new Vector2d(10, 47), Math.toRadians(0))  // PICKUP SECOND SET
                .waitSeconds(2)
                //.stopAndAdd(new FtclibCommandAction(new AutonomousTransferCommand(850, 26, 28, 26)))
                .setReversed(false)
                .splineTo(new Vector2d(-28, 38), Math.toRadians(140))
                //.stopAndAdd(new FtclibCommandAction(new AutonomousShootCommand()))
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
                /**.stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(
                        new IntakeStartCommand(),
                        new ElevatorDownCommand(),
                        new CenterTurretCommand()
                )))*/
                .splineTo(new Vector2d(35, 47), Math.toRadians(0))  // PICKUP THIRD SET
                /**.stopAndAdd(new FtclibCommandAction(new AutonomousTransferCommand(1200, 50, 50, 50)))
                .waitSeconds(0.5)*/
//                .turnTo(Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(53, 13), Math.toRadians(-20))
                //.stopAndAdd(new FtclibCommandAction(new AutonomousShootCommand()))
                .waitSeconds(2) // SHOOT THIRD SET
                .turnTo(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(53,40), Math.toRadians(90))
                .stopAndAdd(new FtclibCommandAction(new SequentialCommandGroup(new IntakeStartCommand(), new ElevatorDownCommand())))
                .splineTo(new Vector2d(53, 47), Math.toRadians(90), new TranslationalVelConstraint(5.0))   // PICKUP FOURTH SET
                .waitSeconds(0.5)
                //.stopAndAdd(new FtclibCommandAction(new AutonomousTransferCommand(1200, 50, 50, 50)))

                .setReversed(false)
                .splineTo(new Vector2d(52.5, 13), Math.toRadians(270))
                .turnTo(Math.toRadians(160))
                //.stopAndAdd(new FtclibCommandAction(new AutonomousShootCommand()))


                .waitSeconds(2)
                .build();
        //telemetry.addLine("Here");
        //telemetry.update();
    }

    @Override
    public void onStart() {
        // 1. Get the dashboard instance so we can see what's happening
        com.acmerobotics.dashboard.FtcDashboard dash = com.acmerobotics.dashboard.FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();

        // 2. Run the loop
        // We add !isStopRequested() to ensure we exit cleanly if you press stop
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
        }
    }


    @Override
    public void periodic() {
        // If this auto is *just* the RR path, you can leave this empty.
        // If you want extra telemetry or non-RR logic during the match,
        // put it here.
    }
}
