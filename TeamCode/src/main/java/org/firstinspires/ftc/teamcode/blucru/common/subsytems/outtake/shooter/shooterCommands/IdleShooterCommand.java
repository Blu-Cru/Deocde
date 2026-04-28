package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class IdleShooterCommand extends InstantCommand {

    public IdleShooterCommand(){

        super(() -> {
            Robot.getInstance().shooter.idle();
        });

        addRequirements(Robot.getInstance().shooter);
    }

}
