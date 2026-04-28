package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class AllTransferUpCommand extends InstantCommand {
    public AllTransferUpCommand(){
        super(()->{
            Robot.getInstance().transfer.setAllUp();
        });

        addRequirements(Robot.getInstance().transfer);
    }
}
