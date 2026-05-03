package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class RightTransferDownCommand extends InstantCommand {
    public RightTransferDownCommand(){
        super(()->{
            Robot.getInstance().transfer.rightSetDown();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
