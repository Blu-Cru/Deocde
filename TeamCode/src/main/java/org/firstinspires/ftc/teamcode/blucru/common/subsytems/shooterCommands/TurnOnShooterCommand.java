package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnOnShooterCommand extends InstantCommand {

    public TurnOnShooterCommand(double power) {
        super(() -> {
            Robot.getInstance().shooter.shoot(1);
        });

        addRequirements(Robot.getInstance().shooter);
    }
}
