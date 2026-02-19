package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

// IMPORTS... (Keep your existing subsystem imports)
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

@Config
public class TransferCommand extends InstantCommand { // 1. Extend SequentialCommandGroup

    public static double vel = 900;
    public static double angle = 40;

    public TransferCommand(boolean turreting) {
        super( () -> {new SequentialCommandGroup(
                new ParallelizeIntakeCommand(),
                new AllTransferDownCommand(),
                new CenterTurretCommand(),
                new AutoAimCommand(),
                new WaitCommand(100),
                new ElevatorUpCommand(),
                new WaitCommand(400),
                new ElevatorMiddleCommand(),
                new WaitUntilCommand(new ParallelArmsBooleanSupplier()),
                new AllTransferMiddleCommand(),
                new InstantCommand( () -> {
                    if (turreting){
                        new LockOnGoalCommand().schedule();
                    } else {
                        Globals.telemetry.addLine("CENTER TURRET");
                        new CenterTurretCommand().schedule();;
                    }
                })
                ).schedule();
        }
        );
    }
}
