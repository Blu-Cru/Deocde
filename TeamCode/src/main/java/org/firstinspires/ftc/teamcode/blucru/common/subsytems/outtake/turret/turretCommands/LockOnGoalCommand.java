package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class LockOnGoalCommand extends InstantCommand {

    public LockOnGoalCommand(){
        super(() -> {
            new RunCommand(
                    () -> Robot.getInstance().turret.lockOnGoal(),
                    Robot.getInstance().turret
            ).schedule();
        });
    }


}
