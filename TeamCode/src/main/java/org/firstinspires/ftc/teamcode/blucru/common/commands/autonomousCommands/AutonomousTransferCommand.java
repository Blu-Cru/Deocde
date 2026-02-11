package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;

@Config
public class AutonomousTransferCommand extends InstantCommand {
    public AutonomousTransferCommand(double leftAngle, double middleAngle, double rightAngle){
        super(() -> {
            new SequentialCommandGroup(
                    new IntakeStopCommand(),
                    new WaitCommand(200),
                    new IntakeSpitCommand(),
                    new WaitCommand(1000),
                    new ElevatorUpCommand(),
                    new IntakeStopCommand(),
                    new ParallelizeIntakeCommand(),
                    new WaitCommand(400),
                    new ElevatorMiddleCommand(),
                    new WaitCommand(150),
                    new AllTransferMiddleCommand(),
                    new SetLeftHoodAngleCommand(leftAngle),
                    new SetRightHoodAngleCommand(middleAngle),
                    new SetMiddleHoodAngleCommand(rightAngle)
//                    new WaitCommand(200), //TODO: TUNE WAIT



            ).schedule();
        });
    }

}
