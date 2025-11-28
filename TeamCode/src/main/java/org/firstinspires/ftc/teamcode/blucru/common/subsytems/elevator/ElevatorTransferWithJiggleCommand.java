package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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
