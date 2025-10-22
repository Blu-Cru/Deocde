package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class MiddleTransferDownCommand extends InstantCommand {
    public MiddleTransferDownCommand(){
        super(()->{
            Robot.getInstance().transfer.middleSetDown();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
