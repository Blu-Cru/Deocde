package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class SetShooterVelocityIndependentCommand extends InstantCommand {

    public SetShooterVelocityIndependentCommand(double leftVel, double middleVel, double rightVel){
        super(() -> {
            Robot.getInstance().shooter.shootWithVelocityIndependent(leftVel, middleVel, rightVel);
        });
    }

}
