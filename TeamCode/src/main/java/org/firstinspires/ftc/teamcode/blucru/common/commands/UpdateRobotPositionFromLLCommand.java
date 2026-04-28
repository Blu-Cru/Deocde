package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class UpdateRobotPositionFromLLCommand extends InstantCommand {

    public UpdateRobotPositionFromLLCommand(){
        super (() -> {
            Robot.getInstance().drivetrain.setCurrentPose(Robot.getInstance().llTagDetector.getLLBotPose());
        });

        addRequirements(Robot.getInstance().drivetrain, Robot.getInstance().llTagDetector);
    }

}
