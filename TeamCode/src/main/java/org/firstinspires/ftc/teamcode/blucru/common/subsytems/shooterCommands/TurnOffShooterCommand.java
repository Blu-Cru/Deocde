package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnOffShooterCommand extends InstantCommand {
    public TurnOffShooterCommand(){
        super (() -> Robot.getInstance().shooter.shoot(0));

        addRequirements(Robot.getInstance().shooter);
    }

}
