package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class ElevatorMiddleForIntakeCommand extends InstantCommand {
    public ElevatorMiddleForIntakeCommand(){
        super(() -> {
            Robot.getInstance().elevator.setMiddleForIntake();
        });
    }
}
