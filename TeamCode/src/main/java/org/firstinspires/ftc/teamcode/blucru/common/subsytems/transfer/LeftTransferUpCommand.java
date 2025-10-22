package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class LeftTransferUpCommand extends InstantCommand {
    public LeftTransferUpCommand(){
        super(()->{
            Robot.getInstance().transfer.leftSetUp();
        });
        addRequirements(Robot.getInstance().transfer);
    }
}
