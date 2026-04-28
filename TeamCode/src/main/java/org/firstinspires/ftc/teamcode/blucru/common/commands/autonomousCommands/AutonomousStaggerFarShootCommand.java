//package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;
//
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
//
//public class AutonomousStaggerFarShootCommand extends InstantCommand {
//
//    public AutonomousStaggerFarShootCommand(double TargetFieldCentricAngle){
//        super(() ->{
//                new SequentialCommandGroup(
//                        new LeftTransferUpCommand(),
//                        new WaitCommand(200),
//                        new TurnTurretToPosFieldCentricCommand(TargetFieldCentricAngle),
//                        new WaitCommand(500),
//                        new MiddleTransferUpCommand(),
//                        new WaitCommand(200),
//                        new TurnTurretToPosFieldCentricCommand(TargetFieldCentricAngle+ 2.8),
//                        new WaitCommand(500),
//                        new RightTransferUpCommand(),
//                        new WaitCommand(200),
//                        new CenterTurretCommand(),
//                        new IdleShooterCommand(),
//                        new WaitCommand(400),
//                        new ElevatorDownCommand(),
//                        new AllTransferDownCommand(),
//                        new WaitCommand(300),
////                        new WaitCommand(300),
//                        new IntakeStartCommand()
//                ).schedule();}
//        );
//    }
//
//}
