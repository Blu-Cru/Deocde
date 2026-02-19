package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelArmsBooleanSupplier;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class intakeTest extends BluLinearOpMode {

    public enum State{
        PARALLEING,
        IDLE
    }

    State state;
    DigitalChannel channel;

    public void initialize(){
        robot.clear();
        addIntake();
        /*addElevator();
        elevator.setUp();
        elevator.write();
        elevator.setDown();
        elevator.write();
        state = State.IDLE;
        channel = hardwareMap.digitalChannel.get("aligner");*/
    }

    public void periodic(){
        if (driver1.pressedA()){
            new SequentialCommandGroup(
                    new IntakeStartCommand()
            ).schedule();
            state = State.IDLE;
        }

        if (gamepad1.b){
            telemetry.addLine("here");
            new ParallelizeIntakeCommand().schedule();
        }

        if (driver1.pressedY()){
            new SequentialCommandGroup(
                    new IntakeStopCommand()
            ).schedule();
            state = State.IDLE;
        }

    }

}
