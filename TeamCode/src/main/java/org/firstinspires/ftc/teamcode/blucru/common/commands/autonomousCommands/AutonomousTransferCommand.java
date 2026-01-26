package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands.CenterTurretCommand;

@Config
public class AutonomousTransferCommand extends InstantCommand {
    public AutonomousTransferCommand(double vel, double leftAngle, double middleAngle, double rightAngle){
        super(() -> {
            new SequentialCommandGroup(
                    new IntakeSpitCommand(),
                    new WaitCommand(300),
                    new ElevatorUpCommand(),
                    new ParallelizeIntakeCommand(),
                    new ShootWithVelocityCommand(vel),
                    new WaitCommand(500),
                    new ElevatorDownCommand(),
                    new AllTransferMiddleCommand(),
                    new SetLeftHoodAngleCommand(leftAngle),
                    new SetRightHoodAngleCommand(middleAngle),
                    new SetMiddleHoodAngleCommand(rightAngle),
                    new IntakeSpitCommand()
            ).schedule();
        });
    }

}
