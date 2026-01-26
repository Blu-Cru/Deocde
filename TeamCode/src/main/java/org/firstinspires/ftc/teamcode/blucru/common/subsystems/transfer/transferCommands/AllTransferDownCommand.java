package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class AllTransferDownCommand extends InstantCommand {
    public AllTransferDownCommand(){
        super(()->{
            Robot.getInstance().transfer.setAllDown();
        });

        addRequirements(Robot.getInstance().transfer);
    }
}
