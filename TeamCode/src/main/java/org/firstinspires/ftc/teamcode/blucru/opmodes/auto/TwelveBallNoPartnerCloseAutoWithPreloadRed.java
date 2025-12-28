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
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;


@Autonomous(name = "12 Ball Close Auto With Preload No Partner Red", group = "auto")
public class TwelveBallNoPartnerCloseAutoWithPreloadRed extends BluLinearOpMode {
    // TODO: Add trajectory sequence when rr package is configured
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;
    protected Alliance alliance = Alliance.RED;
    @Override
    public void initialize() {
        Globals.setAlliance(alliance);
        manageRobotLoop=false;

        addShooter();
        addIntake();
        addTransfer();
        addElevator();
        addTurret();
        Command pickupBalls = new SequentialCommandGroup(
                new ResetForIntakeCommand(),
                new WaitCommand(500),
                new TransferCommand(true)
        );

        startPose = new Pose2d(-45, 52, Math.toRadians(127));

        drive = new TankDrive(hardwareMap, Globals.mapRRPose2d(startPose));
        shooter.setHoodAngle(26);
        shooter.setMiddleHoodAngle(30);
        shooter.write();
        transfer.setAllMiddle();
        transfer.write();
        elevator.setUp();
        elevator.write();
        elevator.setDown();
        elevator.write();
        turret.resetEncoder();

        path = drive.actionBuilder(Globals.mapRRPose2d(startPose))
                .setReversed(true)
                .splineTo(Globals.mapRRVector(new Vector2d(-32, 42)), Globals.mapAngle(Math.toRadians(135+180)))
                .waitSeconds(0.4)
                //.lineToX(-44)
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand()))//SHOOT PRELOAD
                .waitSeconds(1.2) // SHOOT PRELOAD
                .afterTime(0.1, new FtclibCommandAction(new SequentialCommandGroup(new IntakeStartCommand(), new ElevatorDownCommand())))

                .turnTo(Globals.mapAngle(Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(Globals.mapRRVector(new Vector2d(-20, 48)), Globals.mapAngle(Math.toRadians(0)))  // PICKUP FIRST SET
                .splineTo(Globals.mapRRVector(new Vector2d(-15, 48)), Globals.mapAngle(Math.toRadians(0)))  // PICKUP FIRST SET
                .waitSeconds(0.3)
                .stopAndAdd(new FtclibCommandAction(new AutonomousTransferCommand(820, 26, 30, 26)))
                .setReversed(false)
                .turnTo(Globals.mapAngle(Math.toRadians(200)))
                .stopAndAdd(new FtclibCommandAction(new ElevatorDownCommand()))
                .splineTo(Globals.mapRRVector(new Vector2d(-28, 38)), Globals.mapAngle(Math.toRadians(120)))
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand())) //SHOOT FIRST SET
                .waitSeconds(1.2)

                .setReversed(true)
                .afterTime(0.1, new FtclibCommandAction(new IntakeStartCommand()))
                .splineTo(Globals.mapRRVector(new Vector2d(5, 46)), Globals.mapAngle(Math.toRadians(0)))
                .afterTime(0.1, new FtclibCommandAction(
                        new ElevatorDownCommand()
                ))
                .lineToX(12.5)  // PICKUP SECOND SET
                .waitSeconds(0.1)
                .setReversed(false)
                .afterTime(0.3, new FtclibCommandAction(
                        new AutonomousTransferCommand(830, 26, 30, 29)
                ))

                .splineTo(Globals.mapRRVector(new Vector2d(-28, 38)), Globals.mapAngle(Math.toRadians(130)))
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand()))
                .waitSeconds(1.2) // SHOOT SECOND SET

                .setReversed(true)
                .splineTo(Globals.mapRRVector(new Vector2d(2, 53)), Globals.mapAngle(Math.toRadians(90)))

                .splineTo(Globals.mapRRVector(new Vector2d(2, 59)), Globals.mapAngle(Math.toRadians(90))) // OPEN GATE
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(Globals.mapRRVector(new Vector2d(-4, 45)), Globals.mapAngle(Math.toRadians(180)))
                .afterTime(0.1, new FtclibCommandAction(new SequentialCommandGroup(
                        new ElevatorDownCommand(),
                        new IntakeStartCommand()
                )))
                .setReversed(true)

                .splineTo(Globals.mapRRVector(new Vector2d(28, 46)), Globals.mapAngle(Math.toRadians(0)), new TranslationalVelConstraint(30))
                .splineTo(Globals.mapRRVector(new Vector2d(37, 46)), Globals.mapAngle(Math.toRadians(0)))  // PICKUP THIRD SET
                .waitSeconds(0.1)
                .setReversed(false)
                .afterTime(0.3, new FtclibCommandAction(
                        new AutonomousTransferCommand(830, 26, 33, 28
                )))
                .splineTo(Globals.mapRRVector(new Vector2d(-28, 38)), Globals.mapAngle(Math.toRadians(150)))
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand()))

                .build();


        elevator.setDown();
        elevator.write();
    }

    @Override
    public void onStart() {
        // 1. Get the dashboard instance so we can see what's happening
        com.acmerobotics.dashboard.FtcDashboard dash = com.acmerobotics.dashboard.FtcDashboard.getInstance();
        shooter.shootWithVelocity(830);
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
