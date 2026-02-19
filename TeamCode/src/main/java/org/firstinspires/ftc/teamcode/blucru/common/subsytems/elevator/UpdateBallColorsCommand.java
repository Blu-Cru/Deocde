package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class UpdateBallColorsCommand extends InstantCommand {

    public UpdateBallColorsCommand(){
        super(() -> {
            Robot.getInstance().elevator.updateLeftBallColor();
            Robot.getInstance().elevator.updateMiddleBallColor();
            Robot.getInstance().elevator.updateRightBallColor();
        });
    }
}
