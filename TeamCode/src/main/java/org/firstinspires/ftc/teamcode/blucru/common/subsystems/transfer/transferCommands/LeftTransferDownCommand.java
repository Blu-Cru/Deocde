package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class LeftTransferDownCommand extends InstantCommand {
    public LeftTransferDownCommand(){
        super(()->{
            Robot.getInstance().transfer.leftSetDown();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
