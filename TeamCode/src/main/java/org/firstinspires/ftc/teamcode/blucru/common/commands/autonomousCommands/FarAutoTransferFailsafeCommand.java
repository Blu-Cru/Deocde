package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;



public class FarAutoTransferFailsafeCommand extends InstantCommand {
    public FarAutoTransferFailsafeCommand(double hood, double turretAngle) {
        super(() -> {
            new SequentialCommandGroup(
                    // ===== INITIAL TRANSFER ATTEMPT =====
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

                    // ===== CHECK & RETRY IF NEEDED =====
                    // Read color sensors to see if all 3 slots are filled
                    new InstantCommand(() -> {
                        Robot.getInstance().elevator.updateLeftBallColor();
                        Robot.getInstance().elevator.updateMiddleBallColor();
                        Robot.getInstance().elevator.updateRightBallColor();
                    }),
                    new ConditionalCommand(
                            // TRANSFER SUCCEEDED (no balls detected = they made it into turret)
                            new SequentialCommandGroup(
                                    new ParallelizeIntakeCommand(),
                                    new TurnTurretToPosCommand(turretAngle)),

                            // TRANSFER FAILED (balls still detected = stuck in elevator)
                            // Retry sequence:
                            new SequentialCommandGroup(
                                    new IntakeSpitCommand(), // spit out extra balls
                                    new CenterTurretCommand(), // ensure turret is centered
                                    new ElevatorDownCommand(), // drop elevator back down
                                    new AllTransferDownCommand(), // reset transfers
                                    new WaitCommand(1000), // let spit clear jams
                                    new ParallelizeIntakeCommand(), // parallelize so turret can turn
                                    new ElevatorUpCommand(), // try elevator up again
                                    new WaitCommand(300),
                                    new ElevatorMiddleCommand(), // drop balls onto transfer
                                    new WaitCommand(150),
                                    new AllTransferMiddleCommand(),
                                    new SetHoodAngleCommand(hood),
                                    new WaitCommand(200),
                                    new TurnTurretToPosCommand(turretAngle)),

                            // Condition: true = NO balls detected = transfer succeeded
                            // false = balls still detected = transfer failed, retry
                            () -> ShooterMotifCoordinator.getLeftColor() == BallColor.UNKNOWN &&
                                    ShooterMotifCoordinator.getMiddleColor() == BallColor.UNKNOWN &&
                                    ShooterMotifCoordinator.getRightColor() == BallColor.UNKNOWN))
                    .schedule();
        });
    }
}
