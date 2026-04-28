package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class MoveTurretTo180DegreeTransferCommand extends SequentialCommandGroup {

    public MoveTurretTo180DegreeTransferCommand(){
        addCommands(
                new TurnTurretToPosCommand(-182)
        );

        addRequirements(Robot.getInstance().turret);
    }

}
