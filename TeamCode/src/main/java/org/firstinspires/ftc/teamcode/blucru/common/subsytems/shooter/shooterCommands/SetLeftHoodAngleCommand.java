package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class SetLeftHoodAngleCommand extends InstantCommand {

    public SetLeftHoodAngleCommand(double hoodAngle){

        super(() -> {
            Robot.getInstance().shooter.setLeftHoodAngle(hoodAngle);
        });

        addRequirements(Robot.getInstance().shooter);
    }

}
