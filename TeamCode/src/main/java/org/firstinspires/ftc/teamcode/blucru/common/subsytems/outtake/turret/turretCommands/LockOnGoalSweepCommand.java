package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class LockOnGoalSweepCommand extends InstantCommand {

    public LockOnGoalSweepCommand() {
        super(() -> Robot.getInstance().turret.lockOnGoalWithSweep());

        addRequirements(Robot.getInstance().turret);
    }
}
