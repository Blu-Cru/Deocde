package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class SwitchToMotifModeCommand extends InstantCommand {

    public SwitchToMotifModeCommand(){
        super(() -> {
            Robot.getInstance().llTagDetector.switchToMotif();
        });
    }

}
