package org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TurnOffShooterCommand extends InstantCommand {
    public TurnOffShooterCommand(){
        super (() -> Robot.getInstance().shooter.rampDownShooter());

        addRequirements(Robot.getInstance().shooter);
    }

}
