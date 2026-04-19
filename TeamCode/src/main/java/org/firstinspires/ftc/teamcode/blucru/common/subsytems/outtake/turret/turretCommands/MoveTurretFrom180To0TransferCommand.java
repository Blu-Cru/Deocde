package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class MoveTurretFrom180To0TransferCommand extends InstantCommand {
    public MoveTurretFrom180To0TransferCommand(){
        super(() -> {new SequentialCommandGroup(
                new TurnTurretToPosCommand(-90),
                new WaitCommand(100),
                new TurnTurretToPosCommand(0)
        ).schedule();});
    }
}
