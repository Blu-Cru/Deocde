package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;


@Autonomous(name = "Red Close Auto Path", group = "auto")
public class RedCloseAutoPath extends BluLinearOpMode {
    // TODO: Add trajectory sequence when rr package is configured
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;
    protected Alliance alliance = Alliance.RED;
    @Override
    public void initialize() {
        Globals.setAlliance(alliance);
        robot.clear();


        startPose = new Pose2d(-45, 52, Math.toRadians(127));

        drive = new TankDrive(hardwareMap, Globals.mapRRPose2d(startPose));
//        shooter.setRRPoseSupplier(() -> drive.localizer.getPose());
//        turret.setRRPoseSupplier(() -> drive.localizer.getPose());


        path = drive.actionBuilder(Globals.mapRRPose2d(startPose))
                .setReversed(true)
                .splineTo(new Vector2d(-33, 40), Math.toRadians(0))
//                        .afterTime(0.1, new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new LeftTransferUpCommand(),
//                                        new WaitCommand(200),
//                                        new MiddleTransferUpCommand(),
//                                        new WaitCommand(200),
//                                        new RightTransferUpCommand(),
//                                        new WaitCommand(500),
//                                        new CenterTurretCommand(),
//                                        new WaitCommand(500),
//                                        new AllTransferDownCommand(),
//                                        new IntakeStartCommand(),
//                                        new ElevatorDownCommand()
//                                ), false
//                        ))
                .waitSeconds(2) // SHOOT PRELOAD

                .setReversed(true)
                .splineTo(new Vector2d(-18, 45), Math.toRadians(0))
                .waitSeconds(1.3) // PICKUP FIRST SET
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new ElevatorUpCommand(),
//                                        new WaitCommand(200),
//                                        new IntakeStopCommand(),
//                                        new ElevatorMiddleCommand(),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(200),
//                                        new TurnTurretToPosCommand(75)
//                                )
//                        ))

                .setReversed(false)
                .splineTo(new Vector2d(-33, 40), Math.toRadians(180))
                .waitSeconds(0.3)
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new LeftTransferUpCommand(),
//                                        new WaitCommand(100),
//                                        new MiddleTransferUpCommand(),
//                                        new WaitCommand(100),
//                                        new RightTransferUpCommand(),
//                                        new WaitCommand(500),
//                                        new CenterTurretCommand(),
//                                        new WaitCommand(500),
//                                        new AllTransferDownCommand(),
//                                        new IntakeStartCommand(),
//                                        new ElevatorDownCommand()
//                                ), false
//                        ))
                .waitSeconds(2) // SHOOT FIRST SET

                .setReversed(true)
                .splineTo(new Vector2d(5, 37), Math.toRadians(0))
                .waitSeconds(2)
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new ElevatorUpCommand(),
//                                        new WaitCommand(300),
//                                        new ElevatorMiddleCommand(),
//                                        new WaitCommand(100),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(300),
//                                        new IntakeStopCommand(),
//                                        new TurnTurretToPosCommand(90)
//                                ), false
//                        ))
                .waitSeconds(1) // PICKUP SECOND SET

                .setReversed(false)
                .setTangent(Math.toRadians(180)) // Fixed: keeps path flat at Y=40
                .splineTo(new Vector2d(-33, 40), Math.toRadians(180))
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new LeftTransferUpCommand(),
//                                        new WaitCommand(100),
//                                        new MiddleTransferUpCommand(),
//                                        new WaitCommand(100),
//                                        new RightTransferUpCommand(),
//                                        new WaitCommand(500),
//                                        new CenterTurretCommand(),
//                                        new WaitCommand(500),
//                                        new AllTransferDownCommand(),
//                                        new IntakeStartCommand(),
//                                        new ElevatorDownCommand()
//                                ), false
//                        ))
                .waitSeconds(2) // SHOOT SECOND SET

                .setReversed(true)
                .turnTo(Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(-2, 50), Math.toRadians(90)) // Waypoint for stability
                .splineTo(new Vector2d(-2, 56), Math.toRadians(90)) // OPEN GATE
                .waitSeconds(1)

                .setReversed(false)
                .splineTo(new Vector2d(-7, 45), Math.toRadians(180))

                .setReversed(true)
                .splineTo(new Vector2d(30, 45), Math.toRadians(0))
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new ElevatorUpCommand(),
//                                        new WaitCommand(300),
//                                        new IntakeStopCommand(),
//                                        new ElevatorMiddleCommand(),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(200),
//                                        new TurnTurretToPosCommand(70)
//                                )
//                        ))
                .waitSeconds(2) // PICKUP THIRD SET

                .setReversed(false)
                .splineTo(new Vector2d(-33, 40), Math.toRadians(180))
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new LeftTransferUpCommand(),
//                                        new WaitCommand(200),
//                                        new MiddleTransferUpCommand(),
//                                        new WaitCommand(200),
//                                        new RightTransferUpCommand(),
//                                        new WaitCommand(500),
//                                        new CenterTurretCommand(),
//                                        new WaitCommand(500),
//                                        new AllTransferDownCommand(),
//                                        new IntakeStartCommand(),
//                                        new ElevatorDownCommand()
//                                ), false
//                        ))
                .waitSeconds(2) // SHOOT THIRD SET

                .setReversed(true)
                .splineTo(new Vector2d(53, 40), Math.toRadians(90))
                .splineTo(new Vector2d(53, 45), Math.toRadians(90))
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new ElevatorUpCommand(),
//                                        new IntakeStopCommand(),
//                                        new WaitCommand(300),
//                                        new ElevatorMiddleCommand(),
//                                        new WaitCommand(100),
//                                        new AllTransferMiddleCommand(),
//                                        new WaitCommand(300),
//                                        new TurnTurretToPosCommand(30)
//                                )
//                        ))
                .waitSeconds(2) // PICKUP FOURTH SET

                .setReversed(false)
                .splineTo(new Vector2d(52.5, 13), Math.toRadians(270))
                .turnTo(Math.toRadians(160))
//                        .stopAndAdd(new FtclibCommandAction(
//                                new SequentialCommandGroup(
//                                        new LockOnGoalCommand(),
//                                        new AutoAimCommand(),
//                                        new AutonomousShootCommand()
//                                )
//                        ))
                .waitSeconds(2)
                .build();


    }

    @Override
    public void onStart() {
//        shooter.shootWithVelocity(1050); //orig 850 before switching to triple shot
//        turret.setAngle(25);
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
