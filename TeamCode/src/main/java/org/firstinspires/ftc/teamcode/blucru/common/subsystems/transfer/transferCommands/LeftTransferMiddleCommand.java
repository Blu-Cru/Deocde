package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class LeftTransferMiddleCommand extends InstantCommand {

    public LeftTransferMiddleCommand(){
        super(() -> {
            Robot.getInstance().transfer.leftSetMiddle();
        });
    }

}
