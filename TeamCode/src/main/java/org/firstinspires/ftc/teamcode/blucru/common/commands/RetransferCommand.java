package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;

@Config
public class RetransferCommand extends InstantCommand {
    public static double vel = 900;
    public static double angle = 40;
    public RetransferCommand(boolean turreting){
        super(() -> {
            new SequentialCommandGroup(
                    new ElevatorDownCommand(),
//                    new IntakeSpitCommand(),
                    new CenterTurretCommand(),
                    //TODO: conditional wait based on turret
                    new WaitCommand(300),
                    new AllTransferDownCommand(),
                    new WaitCommand(300),
                    new ElevatorUpCommand(),
                    new WaitCommand(300),
                    new ElevatorMiddleCommand(),
                    new ConditionalCommand(
                            new LockOnGoalCommand(),
                            new CenterTurretCommand(),
                            () -> turreting
                    )
            ).schedule();
        });
    }

}