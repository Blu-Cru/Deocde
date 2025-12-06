package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.SetCustomIntakePowerCommand;
@Config
public class ParallelizeIntakeCommand extends InstantCommand {
    public static double topPower = 0.4;
    public static double bottomPower = 0.38;
    public ParallelizeIntakeCommand(){
        new SequentialCommandGroup(
                new SetCustomIntakePowerCommand(topPower),
                new WaitCommand(20),
                new SetCustomIntakePowerCommand(bottomPower)
        ).schedule();
    }
}
