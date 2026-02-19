package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;

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
