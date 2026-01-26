package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class AllTransferUpCommand extends InstantCommand {
    public AllTransferUpCommand(){
        super(()->{
            Robot.getInstance().transfer.setAllUp();
        });

        addRequirements(Robot.getInstance().transfer);
    }
}
