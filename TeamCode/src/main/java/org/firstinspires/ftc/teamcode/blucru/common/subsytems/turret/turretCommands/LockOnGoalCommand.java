package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class LockOnGoalCommand extends InstantCommand {

    public LockOnGoalCommand(){
        super (() -> {
            Robot.getInstance().turret.lockOnGoal();
        });

        addRequirements(Robot.getInstance().turret);
    }

}
