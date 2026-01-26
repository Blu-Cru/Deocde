package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class SixWheelDrivetrainSetPowerCommand extends InstantCommand {
    public SixWheelDrivetrainSetPowerCommand(double power){
        super(() -> {
            Robot.getInstance().sixWheelDrivetrain.setDrivePower(power);
        });

        addRequirements(Robot.getInstance().sixWheelDrivetrain);
    }
}
