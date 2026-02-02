//package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.acmerobotics.roadrunner.Pose2d;
//
//import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
//import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
//
//@Autonomous(name = "Far Gate Auto With Preload No Partner", group = "auto")
//public class NoPartnerFarGateAutoWithPreload extends BluLinearOpMode {
//    // TODO: Add trajectory sequence when rr package is configured
//    private TankDrive drive;
//    private Pose2d startPose;
//    private Action path;
//
//    @Override
//    public void initialize() {
//
//        startPose = new Pose2d(66, 22, Math.toRadians(180));
//
//        drive = new TankDrive(hardwareMap, startPose);
//
//        path = drive.actionBuilder(startPose)
//                .waitSeconds(1)
//                .lineToX(56)
//                .turnTo(Math.toRadians(160))//turn to shoot
//                //SHOOT PRELOAD
//                .waitSeconds(1)
//                //PICKUP FIRST SET OF BALLS
//                .turnTo(Math.toRadians(90))
//                .lineToY(46)
//                .turnTo(Math.toRadians(180))
//                .lineToX(35)
//                .turnTo(Math.toRadians(135))
//                .lineToX(60)
//                .turnTo(Math.toRadians(160))
//                //SHOOT FIRST SET
//                .waitSeconds(1)
//                //PICKUP SECOND SET OF BALLS
//                .turnTo(Math.toRadians(142)) //turning to setup for pickup second set
//
//                .lineToY(46)
//                .turnTo(Math.toRadians(180))
//                .lineToX(10)
//                .turnTo(Math.toRadians(150))
//                .lineToX(56)
//                .turnTo(Math.toRadians(160))//turn to shoot
//                //SHOOT SECOND SET
//                .waitSeconds(1)
//                //PICKUP BALLS ON WALL
//                .turnTo(Math.toRadians(85))
//                .lineToY(65)
//                .waitSeconds(1)
//                .lineToY(22)
//                .build();
//    }
//
//    @Override
//    public void onStart() {
//        // BluLinearOpMode already did waitForStart() for you,
//        // so this is the tutorial's "waitForStart(); Actions.runBlocking(path);"
//        Actions.runBlocking(path);
//    }
//
//    @Override
//    public void periodic() {
//        // If this auto is *just* the RR path, you can leave this empty.
//        // If you want extra telemetry or non-RR logic during the match,
//        // put it here.
//    }
//}
