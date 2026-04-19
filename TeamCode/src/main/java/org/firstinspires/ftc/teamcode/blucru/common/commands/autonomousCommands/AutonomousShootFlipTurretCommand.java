package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretTo180DegreeTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;

public class AutonomousShootFlipTurretCommand extends SequentialCommandGroup {

    public AutonomousShootFlipTurretCommand(){
        addCommands(
                new AllTransferUpCommand(),
                new WaitCommand(100),
                new IdleShooterCommand(),
                new MoveTurretTo180DegreeTransferCommand(),
                new ElevatorDownCommand(),
                new AllTransferDownCommand(),
                new WaitCommand(600),
                new IntakeStartCommand(),
                new InstantCommand(() -> Robot.getInstance().shooter.resetShotCounter())
        );
    }
}
