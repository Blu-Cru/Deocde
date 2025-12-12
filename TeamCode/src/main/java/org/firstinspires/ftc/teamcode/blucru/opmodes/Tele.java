package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import java.util.function.BooleanSupplier;

import org.firstinspires.ftc.teamcode.blucru.common.commands.IdleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.UnshootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.UntransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

@TeleOp (group = "a")

public class Tele extends BluLinearOpMode{

    StateMachine sm;
    public boolean turreting = false;

    public enum State{
        IDLE,
        INTAKING,
        OUTTAKING,
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
        addLLTagDetector();
        addTurret();

        sm = new StateMachineBuilder()

                .state(State.IDLE)
                .transition(() -> driver1.pressedLeftBumper(), State.INTAKING, () ->{
                    new IntakeCommand().schedule();
                })
                .transition(() -> driver1.pressedRightBumper(), State.DRIVING_TO_SHOOT, () ->{
                    new UnshootCommand().schedule();
                })

                .state(State.INTAKING)
                .transition(() -> driver1.pressedLeftTrigger(), State.OUTTAKING, () -> {
                    new IntakeSpitCommand().schedule();
                })
                .transition(() -> driver1.pressedRightBumper(), State.IDLE, () -> {
                    robot.idleRobot();
                    new IdleCommand().schedule();
                })
                .transition(() -> driver1.pressedLeftBumper(), State.DRIVING_TO_SHOOT, () -> {
                    new TransferCommand(turreting).schedule();
                })

                .state(State.OUTTAKING)
                .transition(() -> driver1.pressedLeftTrigger(), State.INTAKING, () -> {
                    new IntakeCommand().schedule();
                })
                .transition(() -> driver1.pressedRightBumper(), State.IDLE, () -> {
                    robot.idleRobot();
                    new IdleCommand().schedule();
                })
                .transition(() -> driver1.pressedLeftBumper(), State.DRIVING_TO_SHOOT, () -> {
                    new TransferCommand(turreting).schedule();
                })

                .state(State.DRIVING_TO_SHOOT)
                .transition(() -> driver1.pressedRightBumper(), State.OUTTAKING, () -> {
                    new UntransferCommand().schedule();
                })
                .transition(() -> driver1.pressedLeftBumper(), State.IDLE, () -> {
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

        //Drivetrain
        sixWheel.teleDrive(gamepad1, 0.0001);
        if (driver2.pressedB() && !driver2.pressedShare()){
            sixWheel.setPosition(llTagDetector.getLLBotPose());
        }

        if (driver2.pressedA() && driver2.pressedX()){
            sixWheel.setPosition(new Pose2d(0, 0, 0));
        }

        if (driver1.pressedRightTrigger()){
            if (sixWheel.getDrivePower() == 0.5){
                sixWheel.setDrivePower(1);
            } else {
                sixWheel.setDrivePower(0.5);
            }
        }

        //Turret

        //auto-aim
        if (driver2.pressedLeftBumper() && turreting) {
            turreting = false;
        } else if (driver2.pressedLeftBumper() && !turreting) {
            turreting = true;
        }

        //manual
        if (driver2.pressedOptions()){
            turret.resetEncoder();
            turret.toggleManual();
        }

        if (turret.isManual()){
            turret.setPower(gamepad2.right_stick_x*0.5);
        }

        telemetry.addData("State", sm.getState());
    }

}
