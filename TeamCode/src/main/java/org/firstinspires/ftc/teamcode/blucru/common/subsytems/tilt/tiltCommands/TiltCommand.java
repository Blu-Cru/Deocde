package org.firstinspires.ftc.teamcode.blucru.common.subsytems.tilt.tiltCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TiltCommand extends InstantCommand {

    public TiltCommand(){
        super(() -> {
            Robot.getInstance().tilt.setDown();
        });
    }
}
