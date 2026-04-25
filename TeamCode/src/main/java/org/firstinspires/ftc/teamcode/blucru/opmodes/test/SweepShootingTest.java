package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootFlipTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootFlipTurretSweepCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalSweepCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretTo180DegreeTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Config
@TeleOp(name = "Sweep Shooting Test", group = "test")
public class SweepShootingTest extends BluLinearOpMode {

    public static double leftVel = 1420;
    public static double middleVel = 1450;
    public static double rightVel = 1430;
    public static double hood = 50;

    public static double testPoseX = 48;
    public static double testPoseY = -9;
    public static double testPoseHeadingDeg = -80;
    public static double manualTurretPowerScale = 0.3;
    public static double teleDriveTurnDeadband = 0.0001;

    @Override
    public void initialize() {
        robot.clear();
        addSixWheel();
        addIntake();
        addElevator();
        addTransfer();
        addShooter();
        addTurret();
        robot.addTurretCam();
        enableDash();

        Globals.setAlliance(Alliance.BLUE);
        turret.resetEncoder();
        sixWheel.setPosition(getTestPose());
        shooter.setHoodAngle(hood);
        shooter.write();
        elevator.setMiddle();
        elevator.write();
        transfer.setAllMiddle();
        transfer.write();
        turret.setAngle(0);
        turret.write();
    }

    @Override
    public void periodic() {
        sixWheel.teleDrive(gamepad1, teleDriveTurnDeadband);

        if (driver1.pressedA()) {
            sixWheel.setPosition(getTestPose());
        }

        if (driver1.pressedLeftBumper()) {
            buildPrepSequence(new LockOnGoalCommand()).schedule();
        }

        if (driver1.pressedRightBumper()) {
            buildPrepSequence(new LockOnGoalSweepCommand()).schedule();
        }

        if (driver1.pressedX()) {
            new AutonomousShootFlipTurretCommand().schedule();
        }

        if (driver1.pressedY()) {
            new AutonomousShootFlipTurretSweepCommand().schedule();
        }

        if (driver1.pressedDpadUp()) {
            new CenterTurretCommand().schedule();
        }

        if (driver1.pressedDpadDown()) {
            new MoveTurretTo180DegreeTransferCommand().schedule();
        }

        if (driver1.pressedDpadLeft()) {
            new AllTransferMiddleCommand().schedule();
        }

        if (driver1.pressedDpadRight()) {
            new AllTransferDownCommand().schedule();
        }

        if (driver2.pressedLeftBumper()) {
            Globals.setAlliance(Alliance.BLUE);
        }

        if (driver2.pressedRightBumper()) {
            Globals.setAlliance(Alliance.RED);
        }

        if (driver2.pressedA()) {
            new SetShooterVelocityIndependentCommand(leftVel, middleVel, rightVel).schedule();
        }

        if (driver2.pressedB()) {
            new IdleShooterCommand().schedule();
        }

        if (driver2.pressedX()) {
            turret.resetEncoder();
            turret.toggleManual();
        }

        if (turret.isManual()) {
            turret.setPower(gamepad2.right_stick_x * manualTurretPowerScale);
        }
    }

    @Override
    public void telemetry() {
        Pose2d pose = sixWheel.getPos();
        telemetry.addLine("A: reset pose | LB: prep normal | RB: prep sweep");
        telemetry.addLine("X: normal shoot | Y: sweep shoot | Dpad Up: center");
        telemetry.addLine("Dpad Down: turret 180 | Dpad Left: transfer middle | Dpad Right: transfer down");
        telemetry.addLine("Driver2 LB/RB: blue/red | A: spin shooters | B: idle shooter | X: turret manual");
        telemetry.addData("Alliance", Globals.alliance);
        telemetry.addData("Pose", pose);
        telemetry.addData("Turret Angle", turret.getAngle());
        telemetry.addData("Turret Target", turret.getTargetPosition());
        telemetry.addData("Turret Manual", turret.isManual());
        telemetry.addData("Shots Detected", shooter.detectedShots);
        telemetry.addData("Transfer Left", transfer.getLeftState());
        telemetry.addData("Transfer Middle", transfer.getMiddleState());
        telemetry.addData("Transfer Right", transfer.getRightState());
        if (robot.turretCam.getDetection() != null) {
            telemetry.addData("Tag X", robot.turretCam.getDetection().center.x);
            telemetry.addData("Tag Dist", robot.turretCam.getDistance());
        } else {
            telemetry.addLine("Tag: none");
        }
    }

    private SequentialCommandGroup buildPrepSequence(Command lockCommand) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.resetShotCounter()),
                new SetShooterVelocityIndependentCommand(leftVel, middleVel, rightVel),
                new SetHoodAngleCommand(hood),
                new AllTransferMiddleCommand(),
                lockCommand
        );
    }

    private Pose2d getTestPose() {
        return new Pose2d(testPoseX, testPoseY, Math.toRadians(testPoseHeadingDeg));
    }
}
