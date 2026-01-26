package org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class SetCustomIntakePowerCommand extends InstantCommand {

    public SetCustomIntakePowerCommand(double power){
        super( () -> Robot.getInstance().intake.setPower(power));

        addRequirements(Robot.getInstance().intake);
    }

}
