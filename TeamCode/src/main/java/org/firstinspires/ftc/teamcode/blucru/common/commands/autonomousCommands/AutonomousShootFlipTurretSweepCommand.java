package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.GoalSweepShootAllCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretTo180DegreeTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;

@Config
public class AutonomousShootFlipTurretSweepCommand extends SequentialCommandGroup {

    public static int postShotPauseMs = 150;
    public static int intakeRestartWaitMs = 400;

    public AutonomousShootFlipTurretSweepCommand() {
        addCommands(
                new GoalSweepShootAllCommand(
                        new WaitCommand(postShotPauseMs),
                        new IdleShooterCommand(),
                        new MoveTurretTo180DegreeTransferCommand(),
                        new ElevatorDownCommand(),
                        new AllTransferDownCommand(),
                        new WaitForTurretNearTargetCommand(),
                        new IntakeStartCommand()
                )
        );
    }
}
