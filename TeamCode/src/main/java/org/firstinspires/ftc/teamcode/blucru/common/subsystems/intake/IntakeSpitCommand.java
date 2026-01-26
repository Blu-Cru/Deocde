package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class IntakeSpitCommand extends InstantCommand {
    public IntakeSpitCommand(){
        super(()->{
            Robot.getInstance().intake.setOut();
        });
        addRequirements(Robot.getInstance().intake);
    }
}
