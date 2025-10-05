package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands.TurnOffShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooterCommands.TurnOnShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
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
                        }
                )
                .transition(() -> gamepad1.left_bumper, State.SHOOTING, () -> {
                    new TurnOnShooterCommand(1).schedule();
                })
                .transition(() -> gamepad1.right_bumper, State.IDLE, () -> {
                    new TurnOffShooterCommand().schedule();
                })
                .build();

        robot.clear();
        addShooter();
        shooter.setHoodAngle(0);
    }

    public void periodic(){
        sm.update();
    }

}
