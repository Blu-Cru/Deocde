package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

// IMPORTS... (Keep your existing subsystem imports)
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;

@Config
public class TransferCommand extends SequentialCommandGroup { // 1. Extend SequentialCommandGroup

    public static double vel = 900;
    public static double angle = 40;

    public TransferCommand(boolean turreting) {
        addCommands(
                new AllTransferDownCommand(),
                new CenterTurretCommand(),
                new AutoAimCommand(),
                new WaitCommand(100),
                new ElevatorUpCommand(),
                new ParallelizeIntakeCommand(),
                new WaitCommand(400),
                new ElevatorDownCommand(),
                new WaitCommand(150),
                new AllTransferMiddleCommand(),
                new WaitCommand(300),

                new ConditionalCommand(
                        new LockOnGoalCommand(),
                        new CenterTurretCommand(),
                        () -> turreting
                )
        );
    }
}