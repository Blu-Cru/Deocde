package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class UpdateRobotPositionFromLLCommand extends InstantCommand {

    public UpdateRobotPositionFromLLCommand(){
        super (() -> {
            Robot.getInstance().drivetrain.setCurrentPose(Robot.getInstance().llTagDetector.getLLBotPose());
        });

        addRequirements(Robot.getInstance().drivetrain, Robot.getInstance().llTagDetector);
    }

}
