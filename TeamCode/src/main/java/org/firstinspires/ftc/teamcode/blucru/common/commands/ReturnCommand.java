package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands.CenterTurretCommand;

public class ReturnCommand extends InstantCommand {

    public ReturnCommand(){
        super(() ->{
            new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        Robot.getInstance().sixWheelDrivetrain.makeMotorsBeInFloat();
                    }),
                    new AllTransferDownCommand(),
                    new CenterTurretCommand(),
                    new IdleShooterCommand(),
                    new ResetForIntakeCommand()
            ).schedule();}
        );
    }

}
