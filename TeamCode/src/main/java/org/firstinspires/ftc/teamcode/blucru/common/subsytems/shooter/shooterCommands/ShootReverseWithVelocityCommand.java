package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class ShootReverseWithVelocityCommand extends InstantCommand {
    public ShootReverseWithVelocityCommand(double vel){
        super (() -> Robot.getInstance().shooter.shootReverseWithVelocity(vel));

        addRequirements(Robot.getInstance().shooter);
    }
}
