package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetRightHoodAngleCommand;
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


@Autonomous(name = "Far auto red", group = "auto")
public class TurretFarAutoRed extends BluLinearOpMode {
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

        startPose = new Pose2d(60, 20, Math.toRadians(180));

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
                .setTangent(Math.toRadians(180))

                .setReversed(false)
                .lineToX(53)
                .turnTo(Math.toRadians(270))
                .splineTo(new Vector2d(49, 6), Math.toRadians(220))
                .waitSeconds(0.3)
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
                .waitSeconds(2)//SHOOT PRELOAD
                .turnTo(Math.toRadians(250))
                .setReversed(true)
                .splineTo(new Vector2d(56, 37), Math.toRadians(90))//INTAKE FIRST SET
                .splineTo(new Vector2d(56, 46), Math.toRadians(90), new TranslationalVelConstraint(10.0))//INTAKE FIRST SET

                .waitSeconds(1)
                .turnTo(Math.toRadians(260))

                .stopAndAdd(new FtclibCommandAction(
                                new SequentialCommandGroup(
                                        new CenterTurretCommand(),
                                        new IntakeSpitCommand(),
                                        new WaitCommand(500),
                                        new ElevatorUpCommand(),
                                        new WaitCommand(400),
                                        new ElevatorMiddleCommand(),
                                        new ShootWithVelocityCommand(1390),
                                        new AllTransferMiddleCommand(),
                                        new WaitCommand(700),
                                        new IntakeStopCommand(),
                                        new ParallelizeIntakeCommand(),
                                        new TurnTurretToPosCommand(59)//61 old
                                ), false

                        ))
                .setReversed(false)
                .lineToY(15)
                .turnTo(Math.toRadians(220))
                .waitSeconds(0.2)
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
                .waitSeconds(2)//SHOOT FIRST SET
                .turnTo(Math.toRadians(270))
                .setReversed(true)
                .lineToY(29)
                .turnTo(Math.toRadians(0))
                .waitSeconds(2.3)
//                        .splineTo(new Vector2d(25, 37), Math.toRadians(180))
                .lineToX(32)
                .waitSeconds(0.5)
                .stopAndAdd(new FtclibCommandAction(
                        new SequentialCommandGroup(
                                new IntakeSpitCommand(),
                                new SetLeftHoodAngleCommand(43),
                                new SetMiddleHoodAngleCommand(44),
                                new SetRightHoodAngleCommand(43),
                                new WaitCommand(500),
                                new ElevatorUpCommand(),
                                new WaitCommand(500),
                                new ElevatorMiddleCommand(),
                                new ShootWithVelocityCommand(1390),
                                new WaitCommand(800),
                                new AllTransferMiddleCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand(),
                                new WaitCommand(500),
                                new ParallelizeIntakeCommand(),
                                new TurnTurretToPosCommand(54)
                        ), false

                ))
                .waitSeconds(1)
                //INTAKE SECOND SET
                .turnTo(Math.toRadians(-45))

                .setReversed(false)
                .splineTo(new Vector2d(42, 10), Math.toRadians(270))
                .turnTo(Math.toRadians(220))
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
                .waitSeconds(2)//SHOOT SECOND SET
                .turnTo(Math.toRadians(270))

                .setReversed(true)
                .splineTo(new Vector2d(45, 46), Math.toRadians(90))
                .waitSeconds(1)//INTAKE THIRD SET
//                .stopAndAdd(new FtclibCommandAction(
//                        new SequentialCommandGroup(
//                                new ElevatorUpCommand(),
//                                new WaitCommand(200),
//                                new IntakeSpitCommand(),
//                                new ElevatorMiddleCommand(),
//                                new ShootWithVelocityCommand(1580),
//                                new WaitCommand(800),
//                                new AllTransferMiddleCommand(),
//                                new WaitCommand(300),
//                                new IntakeStopCommand(),
//                                new TurnTurretToPosCommand(60)
//                        ), false
//                ))
//                .waitSeconds(2)
//                .setReversed(false)
//                .splineTo(new Vector2d(48, 10), Math.toRadians(270)) //SHOOT THIRD SET
//                .turnTo(Math.toRadians(220))
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
                .waitSeconds(2)


//
//                        //INTAKE FOURTH SET
//                        .turnTo(Math.toRadians(90))
//
//                        .setReversed(true)
//                        .splineTo(new Vector2d(50, 47), Math.toRadians(90))
//                        .waitSeconds(1)
//                        //SHOOT FOURTH SET
//                        .setReversed(false)
//                        .splineTo(new Vector2d(48, 10), Math.toRadians(270))
//                        .turnTo(Math.toRadians(220))

                .build();


    }

    @Override
    public void onStart() {

        turret.setAngle(63);
        shooter.shootWithVelocity(1360);
        shooter.setHoodAngleIndependent(41, 43, 41);

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
