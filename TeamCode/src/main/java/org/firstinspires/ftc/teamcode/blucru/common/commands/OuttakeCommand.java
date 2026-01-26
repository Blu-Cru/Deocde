package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeStartCommand;

public class OuttakeCommand extends InstantCommand {

    public OuttakeCommand(){
        super(() -> {
            new SequentialCommandGroup(
                    new IntakeSpitCommand(),
                    new WaitCommand(500),
                    new IntakeStartCommand()
            ).schedule();
        });
    }
}
