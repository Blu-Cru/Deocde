package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;


public class AutoAimCommand extends InstantCommand {

    public AutoAimCommand() {
        super(() -> Robot.getInstance().shooter.autoAim());
        addRequirements(Robot.getInstance().shooter);
    }
}
