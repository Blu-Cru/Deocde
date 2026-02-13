package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;

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
