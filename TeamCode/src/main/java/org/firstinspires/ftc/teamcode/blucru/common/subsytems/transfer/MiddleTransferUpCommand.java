package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class MiddleTransferUpCommand extends InstantCommand {
    public MiddleTransferUpCommand(){
        super(()->{
            Robot.getInstance().transfer.middleSetUp();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
