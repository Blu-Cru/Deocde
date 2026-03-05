package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class shooterEncoderTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        addShooter();
    }

    @Override
    public void periodic() {
        Globals.telemetry.addData("Left Vel", shooter.getLeftVel());
        Globals.telemetry.addData("Middle Vel", shooter.getMiddleVel());
        Globals.telemetry.addData("Right Vel", shooter.getRightVel());
    }
}
