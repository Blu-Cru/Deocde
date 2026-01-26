package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class RightTransferUpCommand extends InstantCommand {
    public RightTransferUpCommand(){
        super(()->{
            Robot.getInstance().transfer.rightSetUp();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
