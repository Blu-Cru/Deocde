package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;

public class UnshootCommand extends InstantCommand {

    public UnshootCommand(){
        super(() ->{
            new SequentialCommandGroup(
                    new AutoAimCommand(),
                    new AllTransferMiddleCommand()
            ).schedule();}
        );
    }

}
