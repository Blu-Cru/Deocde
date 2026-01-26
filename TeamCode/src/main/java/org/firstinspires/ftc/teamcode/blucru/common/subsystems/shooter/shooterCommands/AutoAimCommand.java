package org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;


public class AutoAimCommand extends InstantCommand {

    public AutoAimCommand() {
        super(() -> Robot.getInstance().shooter.autoAim());
        addRequirements(Robot.getInstance().shooter);
    }
}
