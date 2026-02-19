package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;

public class IdleCommand extends InstantCommand {

    public IdleCommand(){
        super (() -> {
            new SequentialCommandGroup(
                    new IdleShooterCommand(),
                    new ElevatorDownCommand(),
                    new AllTransferDownCommand(),
                    new IntakeStopCommand()
            ).schedule();
        });

    }
}
