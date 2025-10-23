package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnTurretToPosCommand extends InstantCommand {

    public TurnTurretToPosCommand(double pos){
        super (() -> {
            Robot.getInstance().turret.setAngle(pos);
        });
        
        addRequirements(Robot.getInstance().turret);
    }

}
