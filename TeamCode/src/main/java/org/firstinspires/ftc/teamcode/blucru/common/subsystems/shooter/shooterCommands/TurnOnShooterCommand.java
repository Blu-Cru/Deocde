package org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TurnOnShooterCommand extends InstantCommand {

    public TurnOnShooterCommand(double power) {
        super(() -> {
            Robot.getInstance().shooter.shoot(power);
        });

        addRequirements(Robot.getInstance().shooter);
    }
}
