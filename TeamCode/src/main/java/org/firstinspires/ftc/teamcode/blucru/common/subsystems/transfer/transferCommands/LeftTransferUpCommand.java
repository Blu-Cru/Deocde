package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class LeftTransferUpCommand extends InstantCommand {
    public LeftTransferUpCommand(){
        super(()->{
            Robot.getInstance().transfer.leftSetUp();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
