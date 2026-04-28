package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class ReadBallColorsCommand extends InstantCommand {
    public ReadBallColorsCommand() {
        super(() -> {
            Robot.getInstance().elevator.updateLeftBallColor();
            Robot.getInstance().elevator.updateMiddleBallColor();
            Robot.getInstance().elevator.updateRightBallColor();
        });
    }
}
