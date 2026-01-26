package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.TurnOffShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands.TurnTurretToPosCommand;

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
