package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

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
