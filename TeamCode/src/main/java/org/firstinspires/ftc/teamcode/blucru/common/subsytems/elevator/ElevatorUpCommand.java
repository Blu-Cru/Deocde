package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class ElevatorUpCommand extends InstantCommand {
    public ElevatorUpCommand(){
        super(()->{
            Robot.getInstance().elevator.setUp();
        });
        addRequirements(Robot.getInstance().elevator);
    }
}
