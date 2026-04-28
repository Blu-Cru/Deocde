package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class ShootReverseWithVelocityCommand extends InstantCommand {
    public ShootReverseWithVelocityCommand(double vel){
        super (() -> Robot.getInstance().shooter.shootReverseWithVelocity(vel));

        addRequirements(Robot.getInstance().shooter);
    }
}
