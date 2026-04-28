package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnOnShooterCommand extends InstantCommand {

    public TurnOnShooterCommand(double power) {
        super(() -> {
            Robot.getInstance().shooter.shoot(power);
        });

        addRequirements(Robot.getInstance().shooter);
    }
}
