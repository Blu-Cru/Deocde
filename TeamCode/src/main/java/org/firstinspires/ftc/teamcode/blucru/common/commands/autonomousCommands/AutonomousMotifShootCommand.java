package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;

public class AutonomousMotifShootCommand extends InstantCommand{
    public AutonomousMotifShootCommand(Alliance alliance){
        super(() -> {
            new SequentialCommandGroup(
                ShooterMotifCoordinator.getOrderToShoot(alliance),
                new WaitCommand(300),
                new IdleShooterCommand(),
                new CenterTurretCommand(),
                new WaitCommand(400),
                new IntakeStartCommand(),
                new ElevatorDownCommand(),
                new AllTransferDownCommand()).schedule();
        });
    }
}
