package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;

public class AutonomousShootAntiJamCommand extends InstantCommand {

    public AutonomousShootAntiJamCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new AllTransferUpCommand(),
                        new WaitCommand(300),
                        new IdleShooterCommand(),
                        new CenterTurretCommand(),
                        new ElevatorMiddleCommand(),
                        new AllTransferDownCommand(),
                        new WaitForTurretNearTargetCommand(),
                        new IntakeStartCommand()
                ).schedule();}
        );
    }

}
