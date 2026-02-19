package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;

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
