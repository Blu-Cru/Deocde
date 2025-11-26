package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;

public class TransferCommand extends InstantCommand {

    public TransferCommand(){
        super(() -> {
            new SequentialCommandGroup(
                    new ElevatorUpCommand(),
                    new IntakeStopCommand(),
                    new AllTransferDownCommand(),
                    new WaitCommand(900),
                    new LockOnGoalCommand(),
                    new ShootWithVelocityCommand(2500)
            ).schedule();
        });
    }

}
