package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

// IMPORTS... (Keep your existing subsystem imports)
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.DisableElevatorCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;
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