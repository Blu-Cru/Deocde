package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class SetMiddleHoodAngleCommand extends InstantCommand {

    public SetMiddleHoodAngleCommand(double hoodAngle){

        super(() -> {
            Robot.getInstance().shooter.setMiddleHoodAngle(hoodAngle);
        });

        addRequirements(Robot.getInstance().shooter);
    }

}
