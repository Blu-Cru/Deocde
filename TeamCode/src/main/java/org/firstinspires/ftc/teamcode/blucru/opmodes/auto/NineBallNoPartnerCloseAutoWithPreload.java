package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;

@Autonomous(name = "9 Ball Close Auto With Preload No Partner", group = "auto")
public class NineBallNoPartnerCloseAutoWithPreload extends BluLinearOpMode {
    // TODO: Add trajectory sequence when rr package is configured
    private TankDrive drive;
    private Pose2d startPose;
    private Action path;

    @Override
    public void initialize() {
        addDrivetrain();   // optional, if you still use your drivetrain subsystem

        startPose = new Pose2d(-45, 52, Math.toRadians(307));

        drive = new TankDrive(hardwareMap, startPose);

        path = drive.actionBuilder(startPose)
                .waitSeconds(3) // for viewing on meepmeep purposes, avoid lag
                .splineTo(new Vector2d(-30, 40), Math.toRadians(45))

                .waitSeconds(1)//SHOOT PRELOAD
                .splineTo(new Vector2d(-20, 47), Math.toRadians(0))
                .splineTo(new Vector2d(-15, 47), Math.toRadians(0))  //PICKUP FIRST BALL

                // tell Road Runner that the *next* path segment is driven backwards
                .setReversed(true)
                .splineTo(new Vector2d(-30, 40), Math.toRadians(225))
                .waitSeconds(1)//SHOOT FIRST
                .setReversed(false)
                .splineTo(new Vector2d(15, 47), Math.toRadians(0))  //PICKUP SECOND BALL
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-30, 40), Math.toRadians(225))
                .waitSeconds(1)//SHOOT SECOND
                .setReversed(false)

                .build();
    }

    @Override
    public void onStart() {
        // BluLinearOpMode already did waitForStart() for you,
        // so this is the tutorial's "waitForStart(); Actions.runBlocking(path);"
        Actions.runBlocking(path);
    }

    @Override
    public void periodic() {
        // If this auto is *just* the RR path, you can leave this empty.
        // If you want extra telemetry or non-RR logic during the match,
        // put it here.
        telemetry.update();
    }
}
