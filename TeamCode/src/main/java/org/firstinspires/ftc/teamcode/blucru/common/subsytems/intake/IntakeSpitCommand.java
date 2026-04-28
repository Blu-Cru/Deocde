package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class IntakeSpitCommand extends InstantCommand {
    public IntakeSpitCommand(){
        super(()->{
            Robot.getInstance().intake.setOut();
        });
        addRequirements(Robot.getInstance().intake);
    }
}
