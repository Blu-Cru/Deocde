package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.TurnOnShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;

public class AutonomousShootCommand extends InstantCommand {

    public AutonomousShootCommand(){
        super(() ->{
                new SequentialCommandGroup(
                        new AllTransferUpCommand(),
                        new WaitCommand(200),
                        new IdleShooterCommand(),
                        new CenterTurretCommand(),
                        //new WaitCommand(200),
                        new ElevatorDownCommand(),
                        new AllTransferDownCommand(),
                        new WaitCommand(200),
//                        new WaitCommand(300),
                        new IntakeStartCommand(),
                        new InstantCommand(() -> Robot.getInstance().shooter.resetShotCounter())
                ).schedule();}
        );
    }
    public AutonomousShootCommand(boolean idleShooter){
        super(() ->{
                    new SequentialCommandGroup(
                            new AllTransferUpCommand(),
                            new WaitCommand(150),
                            new InstantCommand(() -> {
                                if (idleShooter){
                                    new IdleShooterCommand().schedule();
                                }
                            }),
                            new CenterTurretCommand(),
                            //new WaitCommand(200),
                            new ElevatorDownCommand(),
                            new AllTransferDownCommand(),
                            new WaitCommand(400),
//                        new WaitCommand(300),
                            new IntakeStartCommand(),
                            new InstantCommand(() -> Robot.getInstance().shooter.resetShotCounter())
                    ).schedule();}
        );
    }

    public AutonomousShootCommand(double endTurretAngle){
        super(() ->{
                    new SequentialCommandGroup(
                            new AllTransferUpCommand(),
                            new WaitCommand(150),
                            new TurnTurretToPosCommand(endTurretAngle),
                            //new WaitCommand(200),
                            new ElevatorDownCommand(),
                            new AllTransferDownCommand(),
                            new WaitCommand(400),
//                        new WaitCommand(300),
                            new IntakeStartCommand(),
                            new InstantCommand(() -> Robot.getInstance().shooter.resetShotCounter())
                    ).schedule();}
        );
    }



}
