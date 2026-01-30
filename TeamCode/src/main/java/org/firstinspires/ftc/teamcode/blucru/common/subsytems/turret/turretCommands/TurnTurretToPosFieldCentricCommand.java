package org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TurnTurretToPosFieldCentricCommand extends InstantCommand {

    public TurnTurretToPosFieldCentricCommand(double angle, double desiredRobotHeading){
        super(() -> {Robot.getInstance().turret.setFieldCentricPosition(angle, Math.toDegrees(Robot.getInstance().sixWheelDrivetrain.getHeading()), desiredRobotHeading,true);}
        );

        addRequirements(Robot.getInstance().turret);
        addRequirements(Robot.getInstance().sixWheelDrivetrain);

    }


}
