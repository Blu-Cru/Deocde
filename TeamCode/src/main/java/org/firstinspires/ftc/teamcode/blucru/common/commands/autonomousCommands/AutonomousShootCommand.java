package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;

public class AutonomousShootCommand extends InstantCommand {

    public AutonomousShootCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new AllTransferUpCommand(),
                        new WaitCommand(300),
                        new IdleShooterCommand(),
                        new CenterTurretCommand(),
                        new WaitCommand(400),
                        new ElevatorDownCommand(),
                        new AllTransferDownCommand(),
                        new WaitCommand(300),
//                        new WaitCommand(300),
                        new IntakeStartCommand()
                ).schedule();}
        );
    }

}
