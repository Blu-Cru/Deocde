package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class SetHoodAngleCommand extends InstantCommand {

    public SetHoodAngleCommand(double hoodAngle){

        super(() -> {
            Robot.getInstance().shooter.setHoodAngle(hoodAngle);
        });

        addRequirements(Robot.getInstance().shooter);
    }

}
