package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class LockOnGoalCommand extends InstantCommand {

    public LockOnGoalCommand(){
        super(() -> Robot.getInstance().turret.lockOnGoal());

        addRequirements(Robot.getInstance().turret);
    }
}
