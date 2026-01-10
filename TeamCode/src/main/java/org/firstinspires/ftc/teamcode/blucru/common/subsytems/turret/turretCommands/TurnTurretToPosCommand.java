package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnTurretToPosCommand extends InstantCommand {

    public TurnTurretToPosCommand(double angle){
        super(() -> {Robot.getInstance().turret.setAngle(angle);}
        );

        addRequirements(Robot.getInstance().turret);
    }


}
