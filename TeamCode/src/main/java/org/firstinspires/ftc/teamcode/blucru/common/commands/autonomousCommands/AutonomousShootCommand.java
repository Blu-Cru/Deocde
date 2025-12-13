package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;

public class AutonomousShootCommand extends InstantCommand {

    public AutonomousShootCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new LeftTransferUpCommand(),
                        new WaitCommand(250),
                        new MiddleTransferUpCommand(),
                        new WaitCommand(250),
                        new RightTransferUpCommand(),
                        new WaitCommand(400),
                        new AllTransferDownCommand()
                ).schedule();}
        );
    }

}
