package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;

@TeleOp (group = "a")

public class Tele extends BluLinearOpMode{

    StateMachine sm;

    public enum State{
        IDLE,
        INTAKING,
        DRIVING_TO_SHOOT
    }

    @Override
    public void initialize(){
        robot.clear();
        addSixWheel();
        addIntake();
        addElevator();
        addTransfer();
        addShooter();
        //addLLTagDetector();
        //addTurret();
        sm = new StateMachineBuilder()
                .state(State.IDLE)
                .transition(() -> driver1.pressedLeftTrigger(), State.INTAKING, () ->{
                    telemetry.addLine("here");
                    new IntakeCommand().schedule();
                })
                .state(State.INTAKING)
                .transition(() -> driver1.pressedOptions(), State.IDLE, () -> {
                    robot.idleRobot();
                })
                .transition(() -> driver1.pressedLeftBumper(), State.DRIVING_TO_SHOOT, () -> {
                    new TransferCommand().schedule();
                })
                .state(State.DRIVING_TO_SHOOT)
                .transition(() -> driver1.pressedOptions(), State.IDLE, () -> {
                    robot.idleRobot();
                })
                .transition(() -> driver1.pressedRightTrigger(), State.INTAKING, () -> {
                    new SequentialCommandGroup(
                            new ShootBallsCommand()
                    ).schedule();
                })
                .build();

        sm.setState(State.IDLE);

        elevator.setUp();
        elevator.write();
        elevator.setDown();
        elevator.write();
        transfer.setAllDown();
        transfer.write();

        sm.start();
    }

    public void onStart(){
         new ElevatorDownCommand().schedule();
    }

    public void periodic(){
        sm.update();
        sixWheel.teleDrive(gamepad1, 0.0001);
        if (driver2.pressedB()){
            sixWheel.setPosition(llTagDetector.getLLBotPose());
        }
        if (driver1.pressedLeftBumper()){
            if (sixWheel.getDrivePower() == 0.5){
                sixWheel.setDrivePower(1);
            } else {
                sixWheel.setDrivePower(0.5);
            }
        }
        telemetry.addData("State", sm.getState());

    }

}
