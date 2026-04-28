package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
@TeleOp
public class TrackWidthTicksManualTuner extends BluLinearOpMode {

    TankDrive drive;
    Action path;

    public void initialize(){
        addSixWheel();
        Globals.multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new TankDrive(hardwareMap, new Pose2d(0,0,0));
        path = drive.actionBuilder(new Pose2d(0,0,0))
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .turnTo(Math.toRadians(180))
                .turnTo(0)
                .build();
    }

    public void onStart(){
        Actions.runBlocking(path);
    }

    public void telemetry(){
        Globals.multiTelemetry.addData("Robot Heading", sixWheel.getPos().getH());
        Globals.multiTelemetry.update();
    }


}
