package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.TurnOffShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosCommand;

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
