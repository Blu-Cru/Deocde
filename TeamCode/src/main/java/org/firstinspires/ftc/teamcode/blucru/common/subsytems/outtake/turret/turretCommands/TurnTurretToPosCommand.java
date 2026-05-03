package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnTurretToPosCommand extends InstantCommand {

    public TurnTurretToPosCommand(double angle){
        super(() -> {Robot.getInstance().turret.setAngle(angle);}
        );

        addRequirements(Robot.getInstance().turret);
    }


}
