package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class IntakeStartCommand extends InstantCommand {
    public IntakeStartCommand(){
        super(()->{
            Robot.getInstance().intake.setIn();
        });
        addRequirements(Robot.getInstance().intake);
    }
}
