package org.firstinspires.ftc.teamcode.blucru.common.subsytems.tilt.tiltCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class ResetTiltCommand extends InstantCommand {

    public ResetTiltCommand(){
        super(() -> {
            Robot.getInstance().tilt.setUp();
        });
    }
}
