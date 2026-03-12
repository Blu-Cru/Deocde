package org.firstinspires.ftc.teamcode.blucru.common.subsytems.tilt.tiltCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class ResetTiltCommand extends InstantCommand {

    public ResetTiltCommand(){
        super( () -> {new SequentialCommandGroup(
                new InstantCommand(() -> {Robot.getInstance().tilt.goUp();}),
                new WaitCommand(2000),
                new InstantCommand(() -> {Robot.getInstance().tilt.setPower(0);})

        ).schedule();});
    }
}
