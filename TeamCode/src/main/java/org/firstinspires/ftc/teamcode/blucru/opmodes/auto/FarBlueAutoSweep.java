package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootFlipTurretSweepCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalSweepCommand;

public class FarBlueAutoSweep extends FarBlueAuto {

    @Override
    protected Command getGoalLockCommand() {
        return new LockOnGoalSweepCommand();
    }

    @Override
    protected Command getGoalShootCommand() {
        return new AutonomousShootFlipTurretSweepCommand();
    }
}
