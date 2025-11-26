package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;

public class TeleOp extends BluLinearOpMode{

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
        addShooter();
        addLLTagDetector();
        addTurret();
        sm = new StateMachineBuilder()
                .state(State.IDLE)
                .transition(() -> driver1.pressedRightBumper(), State.INTAKING, () ->{
                    new IntakeCommand().schedule();
                })
                .state(State.INTAKING)
                .transition(() -> driver1.pressedOptions(), State.IDLE, () -> {
                    robot.idleRobot();
                })
                .transition(() -> driver2.pressedA(), State.DRIVING_TO_SHOOT, () -> {
                    new TransferCommand().schedule();
                })
                .state(State.DRIVING_TO_SHOOT)
                .transition(() -> driver1.pressedOptions(), State.IDLE, () -> {
                    robot.idleRobot();
                })
                .transition(() -> driver1.pressedRightBumper(), State.INTAKING, () -> {
                    new SequentialCommandGroup(
                            new ShootBallsCommand(),
                            new WaitCommand(100),
                            new IntakeCommand()
                    ).schedule();
                })
                .build();

        sm.setState(State.IDLE);
        sm.start();
    }

    public void periodic(){
        sm.update();
        sixWheel.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x);
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
    }

}
