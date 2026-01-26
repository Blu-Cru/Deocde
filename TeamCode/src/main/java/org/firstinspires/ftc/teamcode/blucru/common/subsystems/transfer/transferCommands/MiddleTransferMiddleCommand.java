package org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class MiddleTransferMiddleCommand extends InstantCommand {

    public MiddleTransferMiddleCommand(){
        super(() -> {
            Robot.getInstance().transfer.leftSetMiddle();
        });
    }

}
