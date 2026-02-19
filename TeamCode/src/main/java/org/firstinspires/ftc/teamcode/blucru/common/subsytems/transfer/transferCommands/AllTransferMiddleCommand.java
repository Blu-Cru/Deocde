package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class AllTransferMiddleCommand extends InstantCommand {

    public AllTransferMiddleCommand(){
        super(()->{
            Robot.getInstance().transfer.setAllMiddle();
        });

        addRequirements(Robot.getInstance().transfer);
    }

}
