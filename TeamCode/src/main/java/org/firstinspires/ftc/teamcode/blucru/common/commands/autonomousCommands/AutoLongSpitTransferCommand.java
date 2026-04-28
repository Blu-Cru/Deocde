package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;

@Config
public class AutoLongSpitTransferCommand extends InstantCommand {
    public AutoLongSpitTransferCommand(double hood){
        super(() -> {
            new SequentialCommandGroup(
                    new IntakeStopCommand(),
                    new WaitCommand(200),
                    new IntakeSpitCommand(),
                    new WaitCommand(1000),
                    new IntakeStopCommand(),
                    new ParallelizeIntakeCommand(),
                    new ElevatorUpCommand(),
                    new WaitCommand(500),
                    new ElevatorMiddleCommand(),
                    new WaitCommand(150),
                    new AllTransferMiddleCommand(),
                    new SetHoodAngleCommand(hood)
//                    new WaitCommand(200), //TODO: TUNE WAIT



            ).schedule();
        });
    }

}
