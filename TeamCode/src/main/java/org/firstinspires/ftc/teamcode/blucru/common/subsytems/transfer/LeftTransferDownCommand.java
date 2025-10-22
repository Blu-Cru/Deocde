package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class LeftTransferDownCommand extends InstantCommand {
    public LeftTransferDownCommand(){
        super(()->{
            Robot.getInstance().transfer.leftSetDown();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
