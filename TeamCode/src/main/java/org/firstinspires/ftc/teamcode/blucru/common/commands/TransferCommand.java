package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;
@Config
public class TransferCommand extends InstantCommand {
    public static double vel = 1500;
    public static double angle = 40;
    public TransferCommand(){
        super(() -> {
            new SequentialCommandGroup(
                    new AllTransferDownCommand(),
                    new WaitCommand(200),
                    new ElevatorUpCommand(),
                    new ParallelizeIntakeCommand(),
                    new ShootWithVelocityCommand(vel),
                    new WaitCommand(500),
                    new ElevatorDownCommand(),
                    new WaitCommand(500),
                    new AllTransferMiddleCommand(),
                    new SetHoodAngleCommand(angle)
            ).schedule();
        });
    }

}
