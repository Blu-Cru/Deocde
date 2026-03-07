package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;

public class FarAutoTransferCommand extends InstantCommand {
    public FarAutoTransferCommand(double hood, double turretAngle){
        super(() -> {
            new SequentialCommandGroup(
                    new CenterTurretCommand(),
                    new IntakeSpitCommand(),
                    new WaitCommand(800),
                    new ElevatorUpCommand(),
                    new IntakeStopCommand(),
                    new ParallelizeIntakeCommand(),
                    new WaitCommand(300),
                    new ElevatorMiddleCommand(),
                    new WaitCommand(150),
                    new AllTransferMiddleCommand(),
                    new SetHoodAngleCommand(hood),
                    new WaitCommand(200),
                    new TurnTurretToPosCommand(turretAngle)



            ).schedule();
        });
    }

}
