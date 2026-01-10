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
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;

@Autonomous(name = "12 Ball Turret Close Auto Blue", group = "auto")
public class TurretTwelveBallCloseAutoBlue extends BluLinearOpMode {
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

        startPose = new Pose2d(-52, -48, -Math.toRadians(127));

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
                .splineTo(new Vector2d(-33, -40), -Math.toRadians(0))
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
                .waitSeconds(1.6)

                .setReversed(true)
                .splineTo(new Vector2d(-18, -45), -Math.toRadians(0))
                .waitSeconds(1)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new WaitCommand(200),
                                new IntakeStopCommand(),
                                new ElevatorMiddleCommand(),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(200),
                                new TurnTurretToPosCommand(-75)
                        ), false
                ))

                .setReversed(false)
                .splineTo(new Vector2d(-27, -37), -Math.toRadians(210))
                .waitSeconds(0.3)
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
                .waitSeconds(1.5)

                .setReversed(true)
                .splineTo(new Vector2d(5, -38), -Math.toRadians(0))
                .waitSeconds(2)

                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new ElevatorUpCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand(),
                                new TurnTurretToPosCommand(-90)
                        ), false
                ))

                .setReversed(false)
                .setTangent(-Math.toRadians(180))
                .splineTo(new Vector2d(-19, -32), -Math.toRadians(190))
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
                .waitSeconds(2)

                .setReversed(true)
                .turnTo(-Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(0, -45), -Math.toRadians(90))
                .splineTo(new Vector2d(0, -53), -Math.toRadians(90))
                .waitSeconds(1.5)

                .setReversed(false)
                .splineTo(new Vector2d(-7, -45), -Math.toRadians(180))

                .setReversed(true)
                .splineTo(new Vector2d(25, -45), -Math.toRadians(0))
                .waitSeconds(0.5)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ElevatorUpCommand(),
                                new WaitCommand(300),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(100),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand(),
                                new WaitCommand(500),
                                new TurnTurretToPosCommand(-85)
                        )
                ))
                .waitSeconds(1.2)

                .setReversed(false)
                .splineTo(new Vector2d(-20, -37), -Math.toRadians(180))
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
                .waitSeconds(2)

                .build();
    }

    @Override
    public void onStart() {
        shooter.setHoodAngleIndependent(26, 26, 26);
        shooter.shootWithVelocity(900);
        turret.setAngle(-40);

        drive.lazyImu.get().resetYaw();                 // IMU yaw = 0
        drive.localizer.setPose(Globals.mapRRPose2d(startPose));  // RR pose heading = startPose.heading


        TelemetryPacket packet = new TelemetryPacket();
        com.acmerobotics.dashboard.FtcDashboard dash = com.acmerobotics.dashboard.FtcDashboard.getInstance();

        while (opModeIsActive() && !isStopRequested() && path.run(packet)) {
            robot.read();

            double headingDeg =
                    Math.toDegrees(drive.localizer.getPose().heading.toDouble());

            CommandScheduler.getInstance().run();
            robot.write();

            dash.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();

            telemetry.addData("Heading (deg)", headingDeg);
            telemetry.addData("Path Running", "True");
            telemetry.update();

            idle();
        }
    }

    @Override public void periodic() {}
    @Override public void telemetry() {}
}
