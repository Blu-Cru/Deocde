package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCloseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

// your own commands:
import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FtclibCommandAction;
import org.firstinspires.ftc.teamcode.blucru.common.util.AutoPathInterpreter;

import java.util.HashMap;

@Autonomous(name = "15 Ball Close Auto With Preload No Partner", group = "auto")
public class FifteenBallNoPartnerCloseAutoWithPreload extends BluLinearOpMode {
    // TODO: Add trajectory sequence when rr package is configured
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;
    double startingVel;
    double startingTurretAngle;

    @Override
    public void initialize() {
        manageRobotLoop = false;

        addShooter();
        addIntake();
        addTransfer();
        addElevator();
        addTurret();

        // Initialize from external file
        // IMPORTANT: You must 'adb push TeamCode/15ball_auto.json
        // /sdcard/FIRST/15ball_auto.json on the robot
        // controller.
        String filePath = "/sdcard/FIRST/15ball_auto.json";

        // Try to create the interpreter
        drive = new TankDrive(hardwareMap, startPose);
        AutoPathInterpreter interpreter = null;
        try {
            interpreter = new AutoPathInterpreter(filePath, drive);
        } catch (Exception e) {
            telemetry.addData("Path Load Failed", e.getMessage());
        }

        //dont need to worry that interpreter might be null bc its wrapped in a try catch
        startPose = interpreter.getStartPose();

        path = interpreter.buildPathFromJSON(startPose);

        startingVel = interpreter.getStartingShooterVel();

        double[] startingHoodAngles = interpreter.getStartingHoodAngles();
        shooter.setLeftHoodAngle(startingHoodAngles[0]);
        shooter.setMiddleHoodAngle(startingHoodAngles[1]);
        shooter.setRightHoodAngle(startingHoodAngles[2]);
        shooter.write();

        startingTurretAngle = interpreter.getStartingTurretAngle();

        transfer.setAllMiddle();
        transfer.write();

        elevator.setUp();
        elevator.write();

        elevator.setDown();
        elevator.write();

        turret.resetEncoder();
    }

    @Override
    public void onStart() {
        // 1. Get the dashboard instance so we can see what's happening
        com.acmerobotics.dashboard.FtcDashboard dash = com.acmerobotics.dashboard.FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        shooter.shootWithVelocity(startingVel);
        turret.setAngle(startingTurretAngle);

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
