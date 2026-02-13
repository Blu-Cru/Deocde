package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
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
 * Anti-jam version of motif shooting command.
 * Safely handles missing motif patterns or undetected balls by falling back to shooting all 3.
 */
public class AutonomousShootWithMotifAntiJamCommand extends InstantCommand {

    public AutonomousShootWithMotifAntiJamCommand() {
        this(Globals.alliance);
    }

    public AutonomousShootWithMotifAntiJamCommand(Alliance alliance) {
        super(() -> {
            boolean anyBallsDetected = 
                (ShooterMotifCoordinator.getLeftColor() != BallColor.UNKNOWN && ShooterMotifCoordinator.getLeftColor() != null) ||
                (ShooterMotifCoordinator.getMiddleColor() != BallColor.UNKNOWN && ShooterMotifCoordinator.getMiddleColor() != null) ||
                (ShooterMotifCoordinator.getRightColor() != BallColor.UNKNOWN && ShooterMotifCoordinator.getRightColor() != null);
            
            new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        if (anyBallsDetected) {
                            Command shootOrderCommand = ShooterMotifCoordinator.getOrderToShoot(alliance);
                            shootOrderCommand.schedule();
                        } else {
                            new AllTransferUpCommand().schedule();
                        }
                    }),
                    new WaitCommand(1300), // Matching the wait time usage
                    new IdleShooterCommand(),
                    new CenterTurretCommand(),
                    new WaitCommand(400),
                    new IntakeStartCommand(),
                    new ElevatorMiddleCommand(), // Anti-jam specific: stay middle
                    new AllTransferDownCommand()
            ).schedule();
        });
    }
}
