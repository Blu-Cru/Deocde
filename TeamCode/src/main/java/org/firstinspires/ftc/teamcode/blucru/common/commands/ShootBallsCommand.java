package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;

public class ShootBallsCommand extends InstantCommand {

    public ShootBallsCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new LeftTransferUpCommand(),
                        new RightTransferUpCommand(),
                        new WaitCommand(50),
                        new MiddleTransferUpCommand(),
                        new WaitCommand(400),
                        new CenterTurretCommand(),
                        new IdleShooterCommand(),
                        new WaitCommand(500),
                        new AllTransferDownCommand(),
                        new ResetForIntakeCommand()
                ).schedule();}
        );
    }

}
