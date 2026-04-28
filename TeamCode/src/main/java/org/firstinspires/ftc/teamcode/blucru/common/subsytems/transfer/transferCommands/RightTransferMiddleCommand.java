package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class RightTransferMiddleCommand extends InstantCommand {

    public RightTransferMiddleCommand(){
        super(() -> {
            Robot.getInstance().transfer.leftSetMiddle();
        });
    }

}
