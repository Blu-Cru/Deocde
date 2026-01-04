package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ResetForIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
@Autonomous
public class RRAutoPath extends BluLinearOpMode {
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;
    public AccelConstraint SLOW_ACCEL = new ProfileAccelConstraint(-20,20);


    // dashboard handle
    private FtcDashboard dashboard;
    public void initialize(){
        enableDash();                      // <- from BluLinearOpMode

        dashboard = FtcDashboard.getInstance();


        startPose = new Pose2d(-45, 52, Math.toRadians(127));

        // TankDrive should already be using your Pinpoint localizer internally
        drive = new TankDrive(hardwareMap, startPose);

        path = drive.actionBuilder(Globals.mapRRPose2d(startPose))
                .setReversed(true)
                .splineTo(Globals.mapRRVector(new Vector2d(-24, 44)), Globals.mapAngle(Math.toRadians(0)))
                .build();

        robot.clear();
        addIntake();
        addElevator();
        addShooter();
        addTurret();
        addTransfer();
        elevator.setMiddle();
        elevator.write();
        transfer.setAllMiddle();
        transfer.write();
        turret.resetEncoder();
        turret.write();
    }

    public void onStart(){
        shooter.setHoodAngleIndependent(26, 28, 26);
        shooter.shootWithVelocity(850);
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
}
