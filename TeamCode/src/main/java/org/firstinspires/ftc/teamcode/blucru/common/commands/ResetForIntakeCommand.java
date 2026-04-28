package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;

public class ResetForIntakeCommand extends InstantCommand {

    public ResetForIntakeCommand(){
        super (() -> {
            new SequentialCommandGroup(
                    new IdleShooterCommand(),
                    new ElevatorDownCommand(),
                    new AllTransferDownCommand(),
                    new CenterTurretCommand()
            ).schedule();
        });

    }
}
