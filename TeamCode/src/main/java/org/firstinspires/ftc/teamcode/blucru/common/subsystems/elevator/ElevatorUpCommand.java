package org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ElevatorUpCommand extends InstantCommand {
    public ElevatorUpCommand(){
        super(()->{
            Robot.getInstance().elevator.setUp();
        });
        addRequirements(Robot.getInstance().elevator);
    }
}
