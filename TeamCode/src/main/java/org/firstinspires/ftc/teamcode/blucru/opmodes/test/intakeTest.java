package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commands.SetIntakeToBeParallelCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorTransferWithJiggleCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class intakeTest extends BluLinearOpMode {

    public enum State{
        PARALLEING,
        IDLE
    }

    State state;

    public void initialize(){
        robot.clear();
        addIntake();
        addElevator();
        elevator.setUp();
        elevator.write();
        elevator.setDown();
        elevator.write();
        state = State.IDLE;
    }

    public void periodic(){
        if (driver1.pressedA()){
            new SequentialCommandGroup(
                    new IntakeStartCommand(),
                    new ElevatorDownCommand()
            ).schedule();
            state = State.IDLE;
        }

        if (driver1.pressedB()){
            state = State.PARALLEING;
        }

        if (driver1.pressedY()){
            new SequentialCommandGroup(
                    new IntakeStopCommand(),
                    new ElevatorUpCommand()
            ).schedule();
            state = State.IDLE;
        }

        switch (state){
            case IDLE:
                break;
            case PARALLEING:
                if (!intake.armsParallel() && !intake.getParallelingArms()){
                    new SetIntakeToBeParallelCommand().schedule();
                }
                break;
        }

        telemetry.addData("Parallel", intake.armsParallel());
    }

}
