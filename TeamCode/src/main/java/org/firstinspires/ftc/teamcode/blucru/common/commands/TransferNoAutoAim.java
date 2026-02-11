package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

// IMPORTS... (Keep your existing subsystem imports)
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;

@Config
public class TransferNoAutoAim extends SequentialCommandGroup { // 1. Extend SequentialCommandGroup

    public static double leftVel = 900;
    public static double rightVel = 900;
    public static double middleVel = 900;
    public static double leftHood = 40;
    public static double middleHood = 40;
    public static double rightHood = 40;
    public static long elevatorUpWait = 400;

    public TransferNoAutoAim(boolean turreting) {
        addCommands(
                new ParallelizeIntakeCommand(),
                new AllTransferDownCommand(),
                new CenterTurretCommand(),
                new SetShooterVelocityIndependentCommand(leftVel, middleVel, rightVel),
                new SetLeftHoodAngleCommand(leftHood),
                new SetMiddleHoodAngleCommand(middleHood),
                new SetRightHoodAngleCommand(rightHood),
                new WaitCommand(100),
                new ElevatorUpCommand(),
                new WaitCommand(elevatorUpWait),
                new ElevatorMiddleCommand(),
                new WaitUntilCommand(new ParallelArmsBooleanSupplier()),
                new AllTransferMiddleCommand(),
                new ConditionalCommand(
                        new LockOnGoalCommand(),
                        new CenterTurretCommand(),
                        () -> turreting
                )
        );
    }
}