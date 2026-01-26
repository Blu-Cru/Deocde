package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.TurnOffShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands.LockOnGoalCommand;

public class AutonomousShootCommand extends InstantCommand {

    public AutonomousShootCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new AutoAimCommand(),
                        new WaitCommand(2000),
                        new LeftTransferUpCommand(),
                        new WaitCommand(250),
                        new MiddleTransferUpCommand(),
                        new WaitCommand(250),
                        new RightTransferUpCommand(),
                        new WaitCommand(400),
                        new AllTransferDownCommand()
                ).schedule();}
        );
    }

}
