package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;

public class ShootBallsCommand extends InstantCommand {

    public ShootBallsCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new AllTransferUpCommand(),
                        new WaitCommand(400),
                        new AllTransferDownCommand()
                ).schedule();}
        );
    }

}
