package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorTransferWithJiggleCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class intakeTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addIntake();
        addElevator();
    }

    public void periodic(){
        if (driver1.pressedA()){
            new SequentialCommandGroup(
                    new IntakeStartCommand(),
                    new ElevatorDownCommand()
            ).schedule();
        }

        if (driver1.pressedY()){
            new SequentialCommandGroup(
                new IntakeStopCommand(),
                new ElevatorTransferWithJiggleCommand()
            ).schedule();
        }
    }

}
