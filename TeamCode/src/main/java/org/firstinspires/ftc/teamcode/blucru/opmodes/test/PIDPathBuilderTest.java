package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
@TeleOp(group = "tuner")
public class PIDPathBuilderTest extends BluLinearOpMode {
    enum States
    {
        IDLE,
        RUNNING_PATH
    }
    Path currentPath;
    StateMachine sm;
    @Override
    public void initialize(){
        robot.clear();
        enableDash();
        addDrivetrain();

        Globals.setAlliance(Alliance.RED);
        drivetrain.setCurrentPose(new Pose2d(0,0,Math.PI/2));

        sm = new StateMachineBuilder()
                .state(States.IDLE)
                .transition(() -> driver1.pressedA(), States.RUNNING_PATH, () -> {
                  currentPath = new TestPath().build().start();
                })
                .state(States.RUNNING_PATH)
                .transition(() -> currentPath.isDone(), States.IDLE, () -> {
                    drivetrain.switchToIdle();
                    currentPath.endMecanum();
                })
                .loop(() -> currentPath.run())
                .build();

        sm.setState(States.IDLE);
        sm.start();
    }

    public void periodic(){
        sm.update();
        drivetrain.drawPose();
    }

    public void onStart(){
        telemetry.addLine("onStart");
        telemetry.addData("Pose here", "x: "+ drivetrain.getCurrPose().getX() + "y: " + drivetrain.getCurrPose().getY() + "h: " + drivetrain.getCurrPose().getH());
        telemetry.update();
        sleep(5000);
    }

    public void telemetry(){
        telemetry.addData("Pose here", "x: "+ drivetrain.getCurrPose().getX() + "y: " + drivetrain.getCurrPose().getY() + "h: " + drivetrain.getCurrPose().getH());
    }
}
