package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class MoveTurretTo180DegreeTransferCommand extends InstantCommand {

    public MoveTurretTo180DegreeTransferCommand(){
        super(() -> {new SequentialCommandGroup(
                new TurnTurretToPosCommand(0)
        ).schedule();});
    }

}
