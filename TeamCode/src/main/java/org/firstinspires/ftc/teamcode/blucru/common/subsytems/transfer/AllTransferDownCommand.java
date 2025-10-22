package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class AllTransferDownCommand extends InstantCommand {
    public AllTransferDownCommand(){
        super(()->{
            new ParallelCommandGroup(
                    new MiddleTransferDownCommand(),
                    new LeftTransferDownCommand(),
                    new RightTransferDownCommand()
            ).schedule();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
