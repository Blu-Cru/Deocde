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
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
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
                new TransferCommand()
        );

        startPose = new Pose2d(-45, 52, Math.toRadians(127));

        drive = new TankDrive(hardwareMap, startPose);

        path = drive.actionBuilder(startPose)
                .setReversed(true)

                .splineTo(new Vector2d(-35, 43), Math.toRadians(160+180))
                .afterTime(0.1, new FtclibCommandAction(new ShootWithVelocityCommand(1000)))

                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCommand())) //SHOOT PRELOAD


                .setReversed(true)
                .splineTo(new Vector2d(-20, 47), Math.toRadians(0))
                .stopAndAdd(new FtclibCommandAction(new IntakeStartCommand()))
                .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  // PICKUP FIRST SET
                .stopAndAdd(new FtclibCommandAction(new AutonomousTransferCommand()))
                .stopAndAdd(new FtclibCommandAction(new ShootWithVelocityCommand(1500))) // instant
                .setReversed(false)
                .turnTo(Math.toRadians(200))

                .splineTo(new Vector2d(-35, 43), Math.toRadians(135))
                .waitSeconds(2) // SHOOT FIRST SET
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCommand()))
                .setReversed(true)
                .splineTo(new Vector2d(0, 47), Math.toRadians(0))
                .splineTo(new Vector2d(10, 47), Math.toRadians(0))  // PICKUP SECOND SET
                .waitSeconds(2)
                .setReversed(false)
                .splineTo(new Vector2d(-35, 43), Math.toRadians(140))
                .waitSeconds(2) // SHOOT SECOND SET

                .setReversed(true)
                .splineTo(new Vector2d(2, 50), Math.toRadians(90))

                .splineTo(new Vector2d(2, 56), Math.toRadians(90),
                        new TranslationalVelConstraint(5.0)) // OPEN GATE
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(-7, 45), Math.toRadians(180))

                .setReversed(true)
                .splineTo(new Vector2d(30, 47), Math.toRadians(0))
                .splineTo(new Vector2d(35, 47), Math.toRadians(0))  // PICKUP THIRD SET
                .waitSeconds(2)
//                .turnTo(Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(53, 13), Math.toRadians(-20))
                .waitSeconds(2) // SHOOT THIRD SET
                .turnTo(Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(53,40), Math.toRadians(90))
                .splineTo(new Vector2d(53, 47), Math.toRadians(90), new TranslationalVelConstraint(5.0))   // PICKUP FOURTH SET
                .waitSeconds(2)

                .setReversed(false)
                .splineTo(new Vector2d(52.5, 13), Math.toRadians(270))
                .turnTo(Math.toRadians(160))


                .waitSeconds(2)
                .build();
    }

    @Override
    public void onStart() {
        TelemetryPacket packet = new TelemetryPacket();
        while (opModeIsActive() && !isStopRequested() && path.run(packet)) {
            robot.read();
            CommandScheduler.getInstance().run();
            robot.write();
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
