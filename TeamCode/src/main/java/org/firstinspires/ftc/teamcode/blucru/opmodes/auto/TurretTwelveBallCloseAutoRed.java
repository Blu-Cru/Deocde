package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.SetAllianceCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
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
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;


@Autonomous(name = "12 Ball Turret Close Auto With Preload No Partner Red", group = "auto")
public class TurretTwelveBallCloseAutoRed extends BluLinearOpMode {
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
                .splineTo(new Vector2d(-30, 40), Math.toRadians(0))
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
                .waitSeconds(1.6) // SHOOT PRELOAD

                .setReversed(true)
                .splineTo(new Vector2d(-18, 45), Math.toRadians(0))
                .waitSeconds(1) // PICKUP FIRST SET
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new WaitCommand(200),
                                new IntakeSpitCommand(),
                                new ElevatorMiddleCommand(),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(700),
                                new IntakeStopCommand(),
                                new TurnTurretToPosCommand(65)
                        ), false
                ))

                .setReversed(false)
                .splineTo(new Vector2d(-27, 37), Math.toRadians(205))
                .waitSeconds(0.8)
                .stopAndAdd(new FtclibCommandAction(
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
                .waitSeconds(1.5) // SHOOT FIRST SET

                .setReversed(true)
                .splineTo(new Vector2d(5, 43), Math.toRadians(0))
                .waitSeconds(1.5)                // PICKUP SECOND SET

                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new IntakeSpitCommand(),
                                new WaitCommand(300),
                                new ElevatorUpCommand(),
                                new WaitCommand(300),
                                new IntakeSpitCommand(),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(700),
                                new IntakeStopCommand(),
                                new TurnTurretToPosCommand(83)
                        ), false
                ))

                .setReversed(false)
                .setTangent(Math.toRadians(180)) // Fixed: keeps path flat at Y=40
                .splineTo(new Vector2d(-19, 32), Math.toRadians(190))
                .waitSeconds(1)
                .stopAndAdd(new FtclibCommandAction(
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
                .waitSeconds(2) // SHOOT SECOND SET

                .setReversed(true)
                .turnTo(Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(0, 43), Math.toRadians(90)) // Waypoint for stability
                .splineTo(new Vector2d(0, 55), Math.toRadians(90)) // OPEN GATE
                .waitSeconds(1.5)

                .setReversed(false)
                .splineTo(new Vector2d(-7, 45), Math.toRadians(180))

                .setReversed(true)
                .splineTo(new Vector2d(25, 47), Math.toRadians(0))
                .waitSeconds(0.5)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new IntakeSpitCommand(),
                                new WaitCommand(500),
                                new ElevatorUpCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(700),
                                new IntakeStopCommand(),
                                new WaitCommand(500),
                                new TurnTurretToPosCommand(22)

                        )
                ))
                .waitSeconds(0.6) // PICKUP THIRD SET

                .setReversed(false)
                .splineTo(new Vector2d(-22, 30), Math.toRadians(160))
                .waitSeconds(0.5)
                .stopAndAdd(new FtclibCommandAction(
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
                                new ElevatorDownCommand(),
                                new SetAllianceCommand(Alliance.RED)
                        ), false
                ))
                .waitSeconds(2) // SHOOT THIRD SET

//                .setReversed(true)
//                .splineTo(new Vector2d(53, 40), Math.toRadians(90))
//                .splineTo(new Vector2d(53, 45), Math.toRadians(90))
//                .stopAndAdd(new FtclibCommandAction(
//                        new SequentialCommandGroup(
//                                new ElevatorUpCommand(),
//                                new IntakeStopCommand(),
//                                new WaitCommand(300),
//                                new ElevatorMiddleCommand(),
//                                new WaitCommand(100),
//                                new AllTransferMiddleCommand(),
//                                new WaitCommand(300),
//                                new TurnTurretToPosCommand(30)
//                        )
//                ))
//                .waitSeconds(2) // PICKUP FOURTH SET
//
//                .setReversed(false)
//                .splineTo(new Vector2d(52.5, 13), Math.toRadians(270))
//                .turnTo(Math.toRadians(160))
//                .stopAndAdd(new FtclibCommandAction(
//                        new SequentialCommandGroup(
//                                new LockOnGoalCommand(),
//                                new AutoAimCommand(),
//                                new AutonomousShootCommand()
//                        )
//                ))
                .waitSeconds(2)
                .build();


    }

    @Override
    public void onStart() {
        shooter.setHoodAngleIndependent(26, 26, 26); //orig 26 28 26 before switch to triple shot
        shooter.shootWithVelocity(900); //orig 850 before switching to triple shot
        turret.setAngle(30);


        drive.lazyImu.get().resetYaw();                 // IMU yaw = 0
        drive.localizer.setPose(Globals.mapRRPose2d(startPose));  // RR pose heading = startPose.heading

        TelemetryPacket packet = new TelemetryPacket();
        com.acmerobotics.dashboard.FtcDashboard dash = com.acmerobotics.dashboard.FtcDashboard.getInstance();
        while (opModeIsActive() && !isStopRequested() && path.run(packet)) {

            // Update FTCLib Subsystems
            robot.read();
            double headingDeg =
                    Math.toDegrees(drive.localizer.getPose().heading.toDouble());

            CommandScheduler.getInstance().run();
            robot.write();
            
            // Draw field visualization (only if pose history has data)
            if (drive.poseHistory.size() > 1) {
                Canvas c = packet.fieldOverlay();
                
                // Draw current robot position (blue circle with heading line)
                c.setStroke("#3F51B5");
                Drawing.drawRobot(c, drive.localizer.getPose());
                
                // Draw pose history (blue line trail) - use efficient polyline drawing
                c.setStrokeWidth(1);
                c.setStroke("#3F51B5");
                double[] xPoints = new double[drive.poseHistory.size()];
                double[] yPoints = new double[drive.poseHistory.size()];
                int idx = 0;
                for (Pose2d pose : drive.poseHistory) {
                    xPoints[idx] = pose.position.x;
                    yPoints[idx] = pose.position.y;
                    idx++;
                }
                c.strokePolyline(xPoints, yPoints);
            }
            
            // 3. IMPORTANT: Send the packet to dashboard!
            // Without this, RoadRunner runs blind and you see no telemetry
            dash.sendTelemetryPacket(packet);

            // Reset packet for the next loop
            packet = new TelemetryPacket();

            // 4. Update standard telemetry to the driver station
            telemetry.addData("Heading (deg)", headingDeg);
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
    @Override
    public void telemetry(){

    }
}
