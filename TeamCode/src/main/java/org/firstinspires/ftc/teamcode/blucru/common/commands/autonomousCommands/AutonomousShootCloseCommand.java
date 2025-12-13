package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;

public class AutonomousShootCloseCommand extends InstantCommand {

    public AutonomousShootCloseCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new SetLeftHoodAngleCommand(26),
                        new SetRightHoodAngleCommand(26),
                        new SetMiddleHoodAngleCommand(28),
                        new WaitCommand(500),
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
