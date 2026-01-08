package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;

public class ShootBallsCommand extends InstantCommand {

    public ShootBallsCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new LeftTransferUpCommand(),
                        new WaitCommand(200),
                        new MiddleTransferUpCommand(),
                        new WaitCommand(200),
                        new RightTransferUpCommand(),
                        new WaitCommand(200),
                        new AllTransferDownCommand(),
                        new CenterTurretCommand(),
                        new IdleShooterCommand(),
                        new ResetForIntakeCommand()
                ).schedule();}
        );
    }

}
