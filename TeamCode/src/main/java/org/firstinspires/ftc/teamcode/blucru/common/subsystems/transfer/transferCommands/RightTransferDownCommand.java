package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class RightTransferDownCommand extends InstantCommand {
    public RightTransferDownCommand(){
        super(()->{
            Robot.getInstance().transfer.rightSetDown();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
