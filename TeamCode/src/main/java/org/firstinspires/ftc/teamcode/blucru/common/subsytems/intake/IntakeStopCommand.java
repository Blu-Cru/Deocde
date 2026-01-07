package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class IntakeStopCommand extends InstantCommand {
    public IntakeStopCommand(){
        super(()->{
            Robot.getInstance().intake.stop();
        });
        addRequirements(Robot.getInstance().intake);
    }
}
