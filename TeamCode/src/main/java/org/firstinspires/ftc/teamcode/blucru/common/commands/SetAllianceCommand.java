package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class SetAllianceCommand extends InstantCommand {

    public SetAllianceCommand(Alliance alliance){
        super( () -> {
            Globals.setAlliance(alliance);
            if (alliance == Alliance.RED){
                Robot.getInstance().shooter.redAlliance = true;
            } else {
                Robot.getInstance().shooter.redAlliance = false;
            }

        });
    }

}
