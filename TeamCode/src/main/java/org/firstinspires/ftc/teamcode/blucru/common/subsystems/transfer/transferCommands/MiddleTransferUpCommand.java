package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class MiddleTransferUpCommand extends InstantCommand {
    public MiddleTransferUpCommand(){
        super(()->{
            Robot.getInstance().transfer.middleSetUp();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
