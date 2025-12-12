package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class AutoAimCommand extends InstantCommand {

    public AutoAimCommand(){
        super(() -> {
            new RunCommand(
                    () -> Robot.getInstance().shooter.autoAim(),
                    Robot.getInstance().shooter
            ).schedule();
        });
    }
}