package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class AllTransferMiddleCommand extends InstantCommand {

    public AllTransferMiddleCommand(){
        super(()->{
            Robot.getInstance().transfer.setAllMiddle();
        });

        addRequirements(Robot.getInstance().transfer);
    }

}
