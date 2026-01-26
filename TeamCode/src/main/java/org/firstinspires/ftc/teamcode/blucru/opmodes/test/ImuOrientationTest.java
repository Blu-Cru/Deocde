package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMU Init + YawPitchRoll Test", group = "test")
public class ImuOrientationTest extends LinearOpMode {

    // Change these to match your Control Hub mount
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO, USB)));

        telemetry.addLine("Press START. Rotate robot: Yaw should change.");
        telemetry.addLine("Press A to reset yaw to 0.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a)
                imu.resetYaw();

            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("LOGO", LOGO);
            telemetry.addData("USB", USB);
            telemetry.addData("Yaw (deg)", ypr.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (deg)", ypr.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (deg)", ypr.getRoll(AngleUnit.DEGREES));
            telemetry.addData("AngVel Z (deg/s)", av.zRotationRate);
            telemetry.update();
        }
    }
}
