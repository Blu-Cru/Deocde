package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.LeftTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.LeftTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.MiddleTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.MiddleTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.RightTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.RightTransferUpCommand;

public class AutonomousShootCloseCommand extends InstantCommand {

    public AutonomousShootCloseCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new LeftTransferUpCommand(),
                        new WaitCommand(250),

                        new MiddleTransferUpCommand(),
                        new LeftTransferMiddleCommand(),
                        new WaitCommand(250),

                        new RightTransferUpCommand(),
                        new MiddleTransferMiddleCommand(),
                        new WaitCommand(250),

                        new RightTransferMiddleCommand(),
                        new LeftTransferUpCommand(),
                        new WaitCommand(250),

                        new MiddleTransferUpCommand(),
                        new WaitCommand(250),

                        new RightTransferUpCommand(),
                        new WaitCommand(250),
                        new AllTransferDownCommand()

                ).schedule();}
        );
    }

}
