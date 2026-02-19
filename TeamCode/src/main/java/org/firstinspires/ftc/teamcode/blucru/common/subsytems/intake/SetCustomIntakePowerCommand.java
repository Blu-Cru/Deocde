package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class SetCustomIntakePowerCommand extends InstantCommand {

    public SetCustomIntakePowerCommand(double power){
        super( () -> Robot.getInstance().intake.setPower(power));

        addRequirements(Robot.getInstance().intake);
    }

}
