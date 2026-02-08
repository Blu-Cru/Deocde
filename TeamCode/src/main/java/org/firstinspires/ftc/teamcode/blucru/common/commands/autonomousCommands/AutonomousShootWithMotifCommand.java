package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

/**
 * Autonomous command that shoots balls in the correct order based on motif pattern.
 * Safely handles missing motif patterns or undetected balls by falling back to shooting all 3.
 */
public class AutonomousShootWithMotifCommand extends InstantCommand {

    public AutonomousShootWithMotifCommand() {
        this(Globals.alliance);
    }

    public AutonomousShootWithMotifCommand(Alliance alliance) {
        super(() -> {
            // Safe ball detection check (handle potential nulls)
            boolean anyBallsDetected = 
                (ShooterMotifCoordinator.getLeftColor() != BallColor.UNKNOWN && ShooterMotifCoordinator.getLeftColor() != null) ||
                (ShooterMotifCoordinator.getMiddleColor() != BallColor.UNKNOWN && ShooterMotifCoordinator.getMiddleColor() != null) ||
                (ShooterMotifCoordinator.getRightColor() != BallColor.UNKNOWN && ShooterMotifCoordinator.getRightColor() != null);
            
            new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        // If balls are detected, try to use motif logic.
                        // getOrderToShoot now safely handles null motif by returning AllTransferUpCommand
                        if (anyBallsDetected) {
                            Command shootOrderCommand = ShooterMotifCoordinator.getOrderToShoot(alliance);
                            shootOrderCommand.schedule();
                        } else {
                            // Fallback: no balls detected, just fire all three
                            new AllTransferUpCommand().schedule();
                        }
                    }),
                    new WaitCommand(1300),
                    new IdleShooterCommand(),
                    new CenterTurretCommand(),
                    new WaitCommand(400),
                    new ElevatorDownCommand(),
                    new AllTransferDownCommand(),
                    new WaitCommand(400),
                    new IntakeStartCommand()
            ).schedule();
        });
    }
}
