package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class CenterTurretCommand extends InstantCommand {

    public CenterTurretCommand(){
        super (() -> {
            Robot.getInstance().turret.setAngle(0);
        });

        addRequirements(Robot.getInstance().turret);
    }

}
