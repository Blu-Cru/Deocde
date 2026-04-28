package org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class LeftTransferMiddleCommand extends InstantCommand {

    public LeftTransferMiddleCommand(){
        super(() -> {
            Robot.getInstance().transfer.leftSetMiddle();
        });
    }

}
