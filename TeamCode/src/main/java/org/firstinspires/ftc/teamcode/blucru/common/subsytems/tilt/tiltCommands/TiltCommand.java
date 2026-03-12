package org.firstinspires.ftc.teamcode.blucru.common.subsytems.tilt.tiltCommands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class TiltCommand extends InstantCommand {

    public TiltCommand(){
        super( () -> {new SequentialCommandGroup(
                new InstantCommand(() -> {Robot.getInstance().tilt.goDown();}),
                new WaitCommand(1200),
                new InstantCommand(() -> {Robot.getInstance().tilt.setPower(0);})

        ).schedule();});
    }
}
