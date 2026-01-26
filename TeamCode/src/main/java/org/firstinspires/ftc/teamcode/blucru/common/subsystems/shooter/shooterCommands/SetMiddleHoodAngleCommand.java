package org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class SetMiddleHoodAngleCommand extends InstantCommand {

    public SetMiddleHoodAngleCommand(double hoodAngle){

        super(() -> {
            Robot.getInstance().shooter.setMiddleHoodAngle(hoodAngle);
        });

        addRequirements(Robot.getInstance().shooter);
    }

}
