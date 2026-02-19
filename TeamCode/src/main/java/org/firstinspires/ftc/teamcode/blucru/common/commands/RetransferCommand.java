package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalCommand;

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
