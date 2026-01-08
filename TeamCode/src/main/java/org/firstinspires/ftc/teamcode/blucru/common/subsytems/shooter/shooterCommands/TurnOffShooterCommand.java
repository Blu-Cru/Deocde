package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnOffShooterCommand extends InstantCommand {
    public TurnOffShooterCommand(){
        super (() -> Robot.getInstance().shooter.rampDownShooter());

        addRequirements(Robot.getInstance().shooter);
    }

}
