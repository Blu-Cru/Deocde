package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;

@Config
public class GoalSweepShootAllCommand extends SequentialCommandGroup {

    public static double shotDelayMs = 50;
    public static int turretMoveTimeoutMs = 300;
    public static int sweepFireTimeoutMs = 200;
    public static double rightShotToleranceDeg = 1.0;
    public static int shotDetectTimeoutMs = 200;

    public GoalSweepShootAllCommand(Command... postShotCommands) {
        addRequirements(
                Robot.getInstance().turret,
                Robot.getInstance().transfer,
                Robot.getInstance().shooter
        );

        addCommands(
                new InstantCommand(() -> Robot.getInstance().shooter.resetShotCounter()),
                new InstantCommand(() -> Robot.getInstance().turret.beginGoalSweep()),
                new TimedWaitUntilCommand(
                        turretMoveTimeoutMs,
                        () -> Robot.getInstance().turret.isGoalSweepStageAtTarget()
                ),
                new LeftTransferUpCommand(),
                new TimedWaitUntilCommand(
                        shotDetectTimeoutMs,
                        () -> Robot.getInstance().shooter.hasShot(1)
                ),
                new InstantCommand(() -> Robot.getInstance().turret.aimGoalSweepStage(Turret.GoalSweepStage.RIGHT_SHOT)),
                new TimedWaitUntilCommand(
                        sweepFireTimeoutMs,
                        () -> predictedReachedSweepStage(Turret.GoalSweepStage.MIDDLE_SHOT)
                ),
                new MiddleTransferUpCommand(),
                new TimedWaitUntilCommand(
                        shotDetectTimeoutMs,
                        () -> Robot.getInstance().shooter.hasShot(2)
                ),
                new TimedWaitUntilCommand(
                        shotDetectTimeoutMs,
                        () -> nearSweepStage(Turret.GoalSweepStage.RIGHT_SHOT, rightShotToleranceDeg)
                ),
                new RightTransferUpCommand(),
                new TimedWaitUntilCommand(
                        shotDetectTimeoutMs,
                        () -> Robot.getInstance().shooter.hasShot(3)
                ),
                new InstantCommand(() -> Robot.getInstance().turret.disableGoalSweep()),
                new InstantCommand(() -> Robot.getInstance().shooter.resetShotCounter())
        );

        addCommands(postShotCommands);
    }

    private boolean predictedReachedSweepStage(Turret.GoalSweepStage stage) {
        double currentAngle = Robot.getInstance().turret.getAngle();
        double omegaDegPerSec = Robot.getInstance().turret.getAngularVelocityDegPerSec();
        double predicted = currentAngle + omegaDegPerSec * (shotDelayMs / 1000.0);
        double target = Robot.getInstance().turret.getGoalSweepStageAngle(stage);
        boolean sweepingPositive =
                Turret.rightShotSweepAngleOffsetDeg > Turret.leftShotSweepAngleOffsetDeg;
        return sweepingPositive ? (predicted >= target) : (predicted <= target);
    }

    private boolean nearSweepStage(Turret.GoalSweepStage stage, double toleranceDeg) {
        double currentAngle = Robot.getInstance().turret.getAngle();
        double target = Robot.getInstance().turret.getGoalSweepStageAngle(stage);
        boolean sweepingPositive =
                Turret.rightShotSweepAngleOffsetDeg > Turret.leftShotSweepAngleOffsetDeg;
        return sweepingPositive
                ? (currentAngle >= target - toleranceDeg)
                : (currentAngle <= target + toleranceDeg);
    }
}
