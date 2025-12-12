package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;

@Config
public class AutonomousTransferCommand extends InstantCommand {
    public static double vel = 1500;
    public static double angle = 40;
    public AutonomousTransferCommand(){
        super(() -> {
            new SequentialCommandGroup(
                    new IntakeStopCommand(),
                    new ElevatorUpCommand(),
                    new ParallelizeIntakeCommand(),
                    new WaitCommand(500),
                    new ElevatorDownCommand(),
                    new WaitCommand(500),
                    new AllTransferMiddleCommand(),
                    new SetHoodAngleCommand(angle)
            ).schedule();
        });
    }

}
