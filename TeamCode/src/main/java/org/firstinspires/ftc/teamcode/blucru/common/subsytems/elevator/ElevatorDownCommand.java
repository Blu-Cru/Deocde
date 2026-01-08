package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class ElevatorDownCommand extends InstantCommand {
    public ElevatorDownCommand(){
        super(()->{
            Robot.getInstance().elevator.setDown();
        });
        addRequirements(Robot.getInstance().elevator);
    }
}
