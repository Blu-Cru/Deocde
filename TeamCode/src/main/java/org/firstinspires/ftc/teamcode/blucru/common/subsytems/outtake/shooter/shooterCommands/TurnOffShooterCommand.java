package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnOffShooterCommand extends InstantCommand {
    public TurnOffShooterCommand(){
        super (() -> Robot.getInstance().shooter.rampDownShooter());

        addRequirements(Robot.getInstance().shooter);
    }

}
