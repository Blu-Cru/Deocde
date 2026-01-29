package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosFieldCentricCommand;

@Config
public class AutonomousAntiJamTransferCommand extends InstantCommand {
    //used so that it only lowers elevator down fully once the bot leaves intaking zone, prevents jams
    public AutonomousAntiJamTransferCommand(double leftAngle, double middleAngle, double rightAngle, double FieldCentricTurretAngle){
        super(() -> {
            new SequentialCommandGroup(
                    new ElevatorDownCommand(),
                    new WaitCommand(1600),
                    new ElevatorUpCommand(),
                    new IntakeStopCommand(),
                    new IntakeSpitCommand(),
                    new WaitCommand(200),
                    new ElevatorMiddleCommand(),
                    new WaitCommand(150),
                    new AllTransferMiddleCommand(),
                    new SetLeftHoodAngleCommand(leftAngle),
                    new SetRightHoodAngleCommand(middleAngle),
                    new SetMiddleHoodAngleCommand(rightAngle),
                    new WaitCommand(700), //TODO: TUNE WAIT
                    new IntakeStopCommand(),
                    new ParallelizeIntakeCommand(),
                    new WaitCommand(400),
                    new TurnTurretToPosCommand(FieldCentricTurretAngle)

            ).schedule();
        });
    }

}
