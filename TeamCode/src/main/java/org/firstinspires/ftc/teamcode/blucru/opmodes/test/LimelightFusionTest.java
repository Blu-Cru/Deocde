package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.Pinpoint;

@TeleOp(name = "Limelight Fusion Test", group = "Test")
public class LimelightFusionTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        // Initialize subsystems
        addDrivetrain(); // Adds Mecanum Drivetrain (DriveBase)
        addTurret();
        addShooter();
        
        telemetry.addLine("Subsystems Initialized");
        telemetry.addLine("Press Start to test Auto-Aim");
        telemetry.update();
    }

    @Override
    public void periodic() {
        // Drive Control (Field Centric if localization works)
        // Use the DriveBase's teleOpDrive method
        if (drivetrain != null) {
            drivetrain.teleOpDrive(gamepad1);
        }

        // Test Auto-Aim
        if (gamepad1.a) {
            turret.lockOnGoal();
            telemetry.addLine("State: Locking on Goal");
        } else {
            turret.toggleManual(); // Or idle
             // Manual control for testing
            double turretPower = gamepad2.left_stick_x * 0.5;
            if(Math.abs(turretPower) > 0.1) turret.setPower(turretPower);
        }

        if (gamepad1.b) {
            shooter.autoAim();
            telemetry.addLine("State: Shooter Auto-Aim (Revving)");
        } else {
            shooter.idle();
        }

        // Telemetry
        telemetry.addLine("=== LOCALIZATION ===");
        if (drivetrain != null) {
             // Access LimelightLocalizer via drivetrain if possible, or just print pose
            telemetry.addData("Robot Pose", drivetrain.getCurrPose().toString());
        }
        
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Turret Angle", turret.getAngle());
        
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Shooter Vel", shooter.getVel());
    }
}
