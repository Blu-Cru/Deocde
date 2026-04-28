package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class AllTransferDownCommand extends InstantCommand {
    public AllTransferDownCommand(){
        super(()->{
            Robot.getInstance().transfer.setAllDown();
        });

        addRequirements(Robot.getInstance().transfer);
    }
}
