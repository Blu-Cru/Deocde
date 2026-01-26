package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.SetCustomIntakePowerCommand;
@Config
public class ParallelizeIntakeCommand extends InstantCommand {
    public ParallelizeIntakeCommand(){
        super( () -> {
            Robot.getInstance().intake.setPID();
        });
    }
}
