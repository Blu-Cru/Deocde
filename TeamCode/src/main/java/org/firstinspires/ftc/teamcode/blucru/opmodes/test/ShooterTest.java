package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands.TurnOffShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands.TurnOnShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

import java.net.IDN;

@TeleOp(group = "test")
public class ShooterTest extends BluLinearOpMode {
    enum State{
        IDLE,
        SHOOTING
    }
    StateMachine sm;
    double hoodAngle;
    public void initialize(){
        sm = new StateMachineBuilder()
                .state(State.IDLE)
                .loop(
                        () -> {
                        new SetHoodAngleCommand(gamepad1.left_stick_y * 2 + Robot.getInstance().shooter.getHoodAngle()).schedule();
                        telemetry.addLine("here");
                        }
                )
                .transition(() -> driver1.pressedLeftBumper(), State.SHOOTING, () -> {
                    new TurnOnShooterCommand(1).schedule();
                })
                .state(State.SHOOTING)
                .loop(() ->{
                    telemetry.addLine("here1");
                })
                .transition(() -> driver1.pressedLeftBumper(), State.IDLE, () -> {
                    new TurnOffShooterCommand().schedule();
                    telemetry.addLine("here2");
                })
                .build();

        sm.setState(State.IDLE);
        sm.start();
        robot.clear();
        addShooter();
        shooter.setHoodAngle(0);
    }

    public void periodic(){
        sm.update();
        if (gamepad1.a){
            shooter.setHoodAngle(0);
        }
        if (gamepad1.b){
            new TurnOnShooterCommand(1).schedule();
        }
    }

    public void telemetry(){
        telemetry.addData("Shooter angle", shooter.getHoodAngle());
        telemetry.addData("G1LSTY", gamepad1.left_stick_y);
        telemetry.addData("State", sm.getState());
    }

}
