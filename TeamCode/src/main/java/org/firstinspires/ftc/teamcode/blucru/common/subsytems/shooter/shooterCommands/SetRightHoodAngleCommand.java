package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class SetRightHoodAngleCommand extends InstantCommand {

    public SetRightHoodAngleCommand(double hoodAngle){

        super(() -> {
            Robot.getInstance().shooter.setRightHoodAngle(hoodAngle);
        });

        addRequirements(Robot.getInstance().shooter);
    }

}
