package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;

/**
 * Performs the autonomous transfer sequence and then immediately locks the turret
 * onto the goal — all within a single properly-composed SequentialCommandGroup.
 *
 * Because LockOnGoalCommand is a direct child of this group (not wrapped in a
 * lambda .schedule() call), the command scheduler correctly tracks turret
 * ownership and prevents race conditions with other turret commands.
 */
public class AutonomousTransferThenLockOnCommand extends SequentialCommandGroup {

    public AutonomousTransferThenLockOnCommand() {
        addCommands(
                new IntakeSpitCommand(),
                new WaitCommand(300),
                new ElevatorUpCommand(),
                new IntakeStopCommand(),
                new ParallelizeIntakeCommand(),
                new WaitCommand(300),
                new ElevatorMiddleCommand(),
                new WaitCommand(150),
                new AllTransferMiddleCommand(),
                new LockOnGoalCommand()
        );
    }

    public AutonomousTransferThenLockOnCommand(double hood) {
        addCommands(
                new IntakeSpitCommand(),
                new WaitCommand(300),
                new ElevatorUpCommand(),
                new IntakeStopCommand(),
                new ParallelizeIntakeCommand(),
                new WaitCommand(300),
                new ElevatorMiddleCommand(),
                new WaitCommand(150),
                new AllTransferMiddleCommand(),
                new SetHoodAngleCommand(hood),
                new LockOnGoalCommand()
        );
    }
}
