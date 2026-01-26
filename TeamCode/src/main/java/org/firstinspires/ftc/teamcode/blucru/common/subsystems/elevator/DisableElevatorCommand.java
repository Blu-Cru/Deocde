package org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class DisableElevatorCommand extends InstantCommand {

    public DisableElevatorCommand(){
        super(() -> {
            Robot.getInstance().elevator.turnOffElevatorServo();
        });
    }
}
