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
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;

@Config
public class Far18TransferCommand extends InstantCommand {
    public Far18TransferCommand(double leftAngle, double middleAngle, double rightAngle){
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
                    new SetLeftHoodAngleCommand(leftAngle),
                    new SetRightHoodAngleCommand(middleAngle),
                    new SetMiddleHoodAngleCommand(rightAngle),
                    new WaitCommand(300),
                    new TurnTurretToPosCommand(102)
//                    new WaitCommand(200), //TODO: TUNE WAIT



            ).schedule();
        });
    }

}
