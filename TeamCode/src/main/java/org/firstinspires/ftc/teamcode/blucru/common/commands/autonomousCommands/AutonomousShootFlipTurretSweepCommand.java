package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.TimedWaitUntilCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretTo180DegreeTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;

@Config
public class AutonomousShootFlipTurretSweepCommand extends SequentialCommandGroup {

    public static int turretMoveTimeoutMs = 100;
    public static int settleBeforeShotMs = 20;
    public static int shotDetectTimeoutMs = 100;
    public static int postShotPauseMs = 120;
    public static int intakeRestartWaitMs = 400;

    public AutonomousShootFlipTurretSweepCommand() {
        addRequirements(
                Robot.getInstance().turret,
                Robot.getInstance().transfer,
                Robot.getInstance().shooter
        );

        addCommands(
                new InstantCommand(() -> {
                    Robot.getInstance().shooter.resetShotCounter();
                    Robot.getInstance().turret.beginGoalSweep();
                }),
                buildSweepShotStep(
                        Turret.GoalSweepStage.LEFT_SHOT,
                        new LeftTransferUpCommand(),
                        1
                ),
                buildSweepShotStep(
                        Turret.GoalSweepStage.MIDDLE_SHOT,
                        new MiddleTransferUpCommand(),
                        2
                ),
                buildSweepShotStep(
                        Turret.GoalSweepStage.RIGHT_SHOT,
                        new RightTransferUpCommand(),
                        3
                ),
                new WaitCommand(postShotPauseMs),
                new InstantCommand(() -> Robot.getInstance().turret.disableGoalSweep()),
                new IdleShooterCommand(),
                new MoveTurretTo180DegreeTransferCommand(),
                new ElevatorDownCommand(),
                new AllTransferDownCommand(),
                new WaitCommand(intakeRestartWaitMs),
                new IntakeStartCommand(),
                new InstantCommand(() -> Robot.getInstance().shooter.resetShotCounter())
        );
    }

    private SequentialCommandGroup buildSweepShotStep(Turret.GoalSweepStage stage,
                                                      Command fireCommand,
                                                      int expectedShots) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> Robot.getInstance().turret.aimGoalSweepStage(stage)),
                new TimedWaitUntilCommand(
                        turretMoveTimeoutMs,
                        () -> Robot.getInstance().turret.isGoalSweepStageAtTarget()
                ),
                new WaitCommand(settleBeforeShotMs),
                fireCommand,
                new TimedWaitUntilCommand(
                        shotDetectTimeoutMs,
                        () -> Robot.getInstance().shooter.hasShot(expectedShots)
                )
        );
    }
}
