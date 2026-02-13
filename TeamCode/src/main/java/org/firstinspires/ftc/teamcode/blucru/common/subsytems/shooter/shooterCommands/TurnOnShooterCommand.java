package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnOnShooterCommand extends InstantCommand {

    public TurnOnShooterCommand(double power) {
        super(() -> {
            Robot.getInstance().shooter.shoot(power);
        });

        addRequirements(Robot.getInstance().shooter);
    }
}
