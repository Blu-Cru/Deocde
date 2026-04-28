package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class FlipTurretCommand extends InstantCommand {

    public FlipTurretCommand(){
        super (() -> {
            new SequentialCommandGroup(
                    new TurnTurretToPosCommand(-90),
                    new WaitCommand(100),
                    new TurnTurretToPosCommand(-180)
            ).schedule();
        });

        addRequirements(Robot.getInstance().turret);
    }

}
