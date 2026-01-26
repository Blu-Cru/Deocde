package org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class TurnTurretToPosCommand extends InstantCommand {

    public TurnTurretToPosCommand(double angle){
        super(() -> {
            new RunCommand(
                    () -> Robot.getInstance().turret.setAngle(angle),
                    Robot.getInstance().turret
            ).schedule();
        });
    }


}
