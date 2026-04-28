package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class ElevatorTransferWithJiggleCommand extends InstantCommand {

    public ElevatorTransferWithJiggleCommand(){
        super(
                () ->{
                    new SequentialCommandGroup(
                            new ElevatorUpCommand(),
                            new WaitCommand(500),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(500),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100),
                            new ElevatorDownCommand(),
                            new WaitCommand(100),
                            new ElevatorUpCommand(),
                            new WaitCommand(100)
                    ).schedule();
                }
        );
    }

}
