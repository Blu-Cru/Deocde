package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.acmerobotics.dashboard.config.Config;
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

    public static double shotDelayMs = 50;
    public static int turretMoveTimeoutMs = 100;
    public static int sweepFireTimeoutMs = 50;
    public static double rightShotToleranceDeg = 1.0;
    public static int shotDetectTimeoutMs = 150;
    public static int postShotPauseMs = 150;
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
                        turretMoveTimeoutMs,
                        () -> nearSweepStage(Turret.GoalSweepStage.RIGHT_SHOT, rightShotToleranceDeg)
                ),
                new RightTransferUpCommand(),
                new TimedWaitUntilCommand(
                        shotDetectTimeoutMs,
                        () -> Robot.getInstance().shooter.hasShot(3)
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
