package org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class ElevatorMiddleCommand extends InstantCommand {
    public ElevatorMiddleCommand(){
        super(() -> {
            Robot.getInstance().elevator.setMiddle();
        });
    }
}
