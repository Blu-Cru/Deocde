package org.firstinspires.ftc.teamcode.blucru.opmodes.auto.old;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;

@Autonomous(name = "TankDrive Forward 2s Test", group = "test")
public class TankDriveForward2sTest extends BluLinearOpMode {

    private TankDrive drive;

    @Override
    public void initialize() {
        // starting pose doesn't really matter for this test
        drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void onStart() {
        // move forward with 5 in/s (x = 5, y = 0), no rotation
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(5, 0), 0
        ));

        sleep(2000); // keep driving for 2 seconds

        // stop
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0, 0), 0
        ));
    }

    @Override
    public void periodic() {
        // Optional: add telemetry if you want
        // telemetry.addData("pose", drive.localizer.getPose());
        // telemetry.update();
    }
}
