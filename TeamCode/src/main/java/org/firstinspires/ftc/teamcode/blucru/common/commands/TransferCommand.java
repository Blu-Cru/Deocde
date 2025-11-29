package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;

public class TransferCommand extends InstantCommand {

    public TransferCommand(){
        super(() -> {
            new SequentialCommandGroup(
                    new AllTransferDownCommand(),
                    new WaitCommand(200),
                    new ElevatorUpCommand(),
                    new IntakeStopCommand(),
                    new WaitCommand(500),
                    new ElevatorDownCommand(),
                    new WaitCommand(900),
                    new ShootWithVelocityCommand(1500),
                    new SetHoodAngleCommand(30)
            ).schedule();
        });
    }

}
