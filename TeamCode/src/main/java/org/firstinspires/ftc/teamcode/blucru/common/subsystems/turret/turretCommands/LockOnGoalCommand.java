package org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

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
