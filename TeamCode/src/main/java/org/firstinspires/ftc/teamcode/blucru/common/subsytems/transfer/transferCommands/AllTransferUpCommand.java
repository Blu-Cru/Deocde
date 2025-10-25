package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class AllTransferUpCommand extends InstantCommand {
    public AllTransferUpCommand(){
        super(()->{
            new ParallelCommandGroup(
                    new MiddleTransferUpCommand(),
                    new LeftTransferUpCommand(),
                    new RightTransferUpCommand()
            ).schedule();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
