package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;

/**
 * Diagnostic OpMode to verify:
 * 1. Motor Mapping (A=FL, B=FR, etc)
 * 2. Motor Direction (Positive power = Forward motion?)
 * 3. Localization/Heading Direction (CCW Turn = Positive Heading change?)
 */
@TeleOp(group = "test")
public class DrivetrainDiagnostic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Set to manual caching for this test loop if desired, or rely on Auto
        // drive initializes to AUTO caching by default in standard RR

        telemetry.addLine("--- DRIVETRAIN DIAGNOSTIC ---");
        telemetry.addLine("A: Spin FL (Check if FL wheel spins)");
        telemetry.addLine("X: Spin BL");
        telemetry.addLine("B: Spin FR");
        telemetry.addLine("Y: Spin BR");
        telemetry.addLine("Joy: Drive Manually");
        telemetry.addLine("-----------------------------");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            double flPower = 0;
            double blPower = 0;
            double frPower = 0;
            double brPower = 0;

            if (gamepad1.a)
                flPower = 0.3;
            if (gamepad1.x)
                blPower = 0.3;
            if (gamepad1.b)
                frPower = 0.3;
            if (gamepad1.y)
                brPower = 0.3;

            // Manual Drive Override
            if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                // Tank Arcade
                double drivePower = -gamepad1.left_stick_y * 0.5;
                double turnPower = -gamepad1.right_stick_x * 0.5;

                // Simple tank mixing
                flPower = drivePower + turnPower;
                blPower = drivePower + turnPower;
                frPower = drivePower - turnPower;
                brPower = drivePower - turnPower;
            }

            // Apply Powers
            drive.leftMotors.get(1).setPower(flPower); // Front Left
            drive.leftMotors.get(0).setPower(blPower); // Back Left
            drive.rightMotors.get(1).setPower(frPower); // Front Right
            drive.rightMotors.get(0).setPower(brPower); // Back Right

            telemetry.addLine("=== Heading Check ===");
            telemetry.addLine("Rotate Robot LEFT (CCW). Heading should INCREASE.");
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            telemetry.addLine("\n=== Motor Check ===");
            telemetry.addData("FL Power", "%.2f (Enc: %d)", flPower, drive.leftMotors.get(1).getCurrentPosition());
            telemetry.addData("BL Power", "%.2f (Enc: %d)", blPower, drive.leftMotors.get(0).getCurrentPosition());
            telemetry.addData("FR Power", "%.2f (Enc: %d)", frPower, drive.rightMotors.get(1).getCurrentPosition());
            telemetry.addData("BR Power", "%.2f (Enc: %d)", brPower, drive.rightMotors.get(0).getCurrentPosition());

            telemetry.update();
        }
    }
}
