package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class IntakeStartCommand extends InstantCommand {
    public IntakeStartCommand(){
        super(()->{
            Robot.getInstance().intake.setIn();
        });
        addRequirements(Robot.getInstance().intake);
    }
}
