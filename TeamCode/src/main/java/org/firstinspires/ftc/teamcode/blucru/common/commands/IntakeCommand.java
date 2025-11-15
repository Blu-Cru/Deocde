package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;

public class IntakeCommand extends InstantCommand {

    public IntakeCommand(){
        super (() -> {
            new SequentialCommandGroup(
                    new IntakeStartCommand(),
                    new ElevatorDownCommand()
            ).schedule();
        });

    }
}
