//package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCloseCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;
//import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
//import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
//import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
//import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
//
//
//@Autonomous(name = "12 Ball Close Auto With Preload No Partner Blue", group = "auto")
//public class TwelveBallNoPartnerCloseAutoWithPreloadBlue extends TwelveBallNoPartnerCloseAutoWithPreloadRed {
//    public TwelveBallNoPartnerCloseAutoWithPreloadBlue(){
//        alliance = Alliance.BLUE;
//    }
//}

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

import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
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

@Autonomous(name = "12 Ball Close Auto BLUE", group = "auto")
public class TwelveBallNoPartnerCloseAutoWithPreloadBlue extends BluLinearOpMode {

    private TankDrive drive;
    private Pose2d startPose;
    private Action path;
    protected Alliance alliance = Alliance.BLUE;

    @Override
    public void initialize() {
        Globals.setAlliance(alliance);
        manageRobotLoop = false;

        addShooter();
        addIntake();
        addTransfer();
        addElevator();
        addTurret();

        // MANUALLY MIRRORED START POSE
        // Red: (-45, 52, 127) -> Blue: (-45, -52, -127)
        startPose = new Pose2d(-45, -52, Math.toRadians(-127));

        drive = new TankDrive(hardwareMap, startPose);

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

        path = drive.actionBuilder(startPose)
                .setReversed(true)
                // Spline 1: Preload
                // Red: (-32, 42), Tangent 315 -> Blue: (-32, -42), Tangent -315 (or 45)
                .splineTo(new Vector2d(-24, -36), Math.toRadians(-310))
                .waitSeconds(0.4)

                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand())) // SHOOT PRELOAD
                .waitSeconds(1.2)
                .afterTime(0.1, new FtclibCommandAction(new SequentialCommandGroup(new IntakeStartCommand(), new ElevatorDownCommand())))

                // Turn 1
                // Red: -90 -> Blue: 90
                .turnTo(Math.toRadians(90))

                .setReversed(true)
                // Pickup First Set
                // Red: (-20, 48), Tangent 0 -> Blue: (-20, -48), Tangent 0
                .splineTo(new Vector2d(-15, -48), Math.toRadians(0))
                // Red: (-15, 48), Tangent 0 -> Blue: (-15, -48), Tangent 0
                .splineTo(new Vector2d(-10, -48), Math.toRadians(0))
                .waitSeconds(0.3)

                .stopAndAdd(new FtclibCommandAction(new AutonomousTransferCommand(820, 26, 30, 26)))
                .setReversed(false)

                // Turn 2
                // Red: 200 -> Blue: -200 (or 160)
                .turnTo(Math.toRadians(-200))

                .stopAndAdd(new FtclibCommandAction(new ElevatorDownCommand()))

                // Shoot First Set
                // Red: (-28, 38), Tangent 120 -> Blue: (-28, -38), Tangent -120
                .splineTo(new Vector2d(-26, -38), Math.toRadians(-135))
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand()))
                .waitSeconds(1.2)

                .setReversed(true)
                .afterTime(0.1, new FtclibCommandAction(new IntakeStartCommand()))

                // Pickup Second Set
                // Red: (5, 46), Tangent 0 -> Blue: (5, -46), Tangent 0
                .splineTo(new Vector2d(4, -47), Math.toRadians(0))
                .afterTime(0.1, new FtclibCommandAction(new ElevatorDownCommand()))
                .lineToX(16)
                .waitSeconds(0.1)

                .setReversed(false)
                .afterTime(0.3, new FtclibCommandAction(new AutonomousTransferCommand(830, 26, 30, 29)))

                // Shoot Second Set
                // Red: (-28, 38), Tangent 130 -> Blue: (-28, -38), Tangent -130
                .splineTo(new Vector2d(-24, -38), Math.toRadians(-150))
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand()))
                .waitSeconds(1.2)

                .setReversed(true)
                // Gate / Open Space
                // Red: (2, 53), Tangent 90 -> Blue: (2, -53), Tangent -90
                .splineTo(new Vector2d(2, -53), Math.toRadians(-90))
                // Red: (2, 59), Tangent 90 -> Blue: (2, -59), Tangent -90
                .splineTo(new Vector2d(2, -64), Math.toRadians(-90))
                .waitSeconds(1)

                .setReversed(false)
                // Return / Pickup Third
                // Red: (-4, 45), Tangent 180 -> Blue: (-4, -45), Tangent -180 (same as 180)
                .splineTo(new Vector2d(-4, -45), Math.toRadians(180))
                .afterTime(0.1, new FtclibCommandAction(new SequentialCommandGroup(
                        new ElevatorDownCommand(),
                        new IntakeStartCommand()
                )))

                .setReversed(true)
                // Red: (28, 46), Tangent 0 -> Blue: (28, -46), Tangent 0
                .splineTo(new Vector2d(28, -46), Math.toRadians(0), new TranslationalVelConstraint(30))
                // Red: (37, 46), Tangent 0 -> Blue: (37, -46), Tangent 0
                .splineTo(new Vector2d(37, -46), Math.toRadians(0))
                .waitSeconds(0.1)

                .setReversed(false)
                .afterTime(0.3, new FtclibCommandAction(new AutonomousTransferCommand(870, 26, 36, 29)))

                // Shoot Third Set
                // Red: (-28, 38), Tangent 150 -> Blue: (-28, -38), Tangent -150
                .splineTo(new Vector2d(-23, -37), Math.toRadians(-180))
                .stopAndAdd(new FtclibCommandAction(new AutonomousShootCloseCommand()))

                .build();

        elevator.setDown();
        elevator.write();
    }

    @Override
    public void onStart() {
        com.acmerobotics.dashboard.FtcDashboard dash = com.acmerobotics.dashboard.FtcDashboard.getInstance();
        shooter.shootWithVelocity(830);
        TelemetryPacket packet = new TelemetryPacket();

        while (opModeIsActive() && !isStopRequested() && path.run(packet)) {
            robot.read();
            CommandScheduler.getInstance().run();
            robot.write();

            dash.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();

            telemetry.addData("Path Running", "True");
            telemetry.update();
        }
    }

    @Override
    public void periodic() {}
}
