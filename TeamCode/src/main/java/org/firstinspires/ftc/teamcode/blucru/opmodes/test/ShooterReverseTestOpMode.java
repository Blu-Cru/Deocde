package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(name = "Shooter Reverse Test", group = "test")
public class ShooterReverseTestOpMode extends BluLinearOpMode {

    private boolean revVelActive = false;
    private boolean slowPowerActive = false;
    private double reverseVel = 200.0; // default magnitude

    @Override
    public void initialize() {
        addShooter();
        addTransfer();
        // Optional: enable dashboard telemetry
        enableDash();
        transfer.setAllMiddle();
    }

    @Override
    public void onStart() {
        telemetry.addLine("Shooter Reverse Test started");
        telemetry.addData("Controls", "A: toggle reverse-velocity | B: toggle slow reverse power | X: stop | DPad Up/Down: +/- vel");
        telemetry.update();
    }

    @Override
    public void periodic() {
        // adjust reverse velocity
        if (driver1.pressedDpadUp()) {
            reverseVel += 25.0;
        }
        if (driver1.pressedDpadDown()) {
            reverseVel = Math.max(25.0, reverseVel - 25.0);
        }

        // toggle reverse velocity mode
        if (driver1.pressedA()) {
            revVelActive = !revVelActive;
            if (revVelActive) {
                slowPowerActive = false;
                shooter.shootReverseWithVelocity(reverseVel);
            } else {
                shooter.rampDownShooter();
            }
        }

        // toggle slow reverse power
        if (driver1.pressedB()) {
            slowPowerActive = !slowPowerActive;
            if (slowPowerActive) {
                revVelActive = false;
                shooter.spinReverseSlowPower();
            } else {
                shooter.rampDownShooter();
            }
        }

        // emergency stop
        if (driver1.pressedX()) {
            revVelActive = false;
            slowPowerActive = false;
            shooter.rampDownShooter();
        }

        // telemetry
        telemetry.addData("revVelActive", revVelActive);
        telemetry.addData("slowPowerActive", slowPowerActive);
        telemetry.addData("reverseVel", reverseVel);
        telemetry.addData("shooterVel", shooter.getLeftVel());
        telemetry.addData("shooterPower", shooter.getLeftPower());
        telemetry.update();
    }
}
