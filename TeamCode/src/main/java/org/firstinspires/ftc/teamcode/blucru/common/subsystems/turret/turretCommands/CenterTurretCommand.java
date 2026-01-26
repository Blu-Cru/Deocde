package org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class CenterTurretCommand extends InstantCommand {

    public CenterTurretCommand(){
        super (() -> {
            Robot.getInstance().turret.setAngle(0);
        });

        addRequirements(Robot.getInstance().turret);
    }

}
