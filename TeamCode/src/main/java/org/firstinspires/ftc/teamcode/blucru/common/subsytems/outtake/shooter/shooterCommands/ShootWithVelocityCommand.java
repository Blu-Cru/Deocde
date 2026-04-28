package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class ShootWithVelocityCommand extends InstantCommand {
    public ShootWithVelocityCommand(double vel){
        super (() -> Robot.getInstance().shooter.shootWithVelocity(vel));

        addRequirements(Robot.getInstance().shooter);
    }
}
