package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class MecanumDrivetrainSetPowerCommand extends InstantCommand {
    public MecanumDrivetrainSetPowerCommand(double power){
        super(() -> {
            Robot.getInstance().mecanumDrivetrain.setDrivePower(power);
        });

        addRequirements(Robot.getInstance().mecanumDrivetrain);
    }
}
