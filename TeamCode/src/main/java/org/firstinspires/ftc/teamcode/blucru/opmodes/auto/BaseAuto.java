package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public abstract class BaseAuto extends BluLinearOpMode {

    public StateMachine sm;
    public Pose2d startPose;

    /**
     * Override this to define the auto's starting pose.
     */
    public abstract Pose2d getStartPose();

    /**
     * Override this to build the state machine for the auto.
     */
    public abstract StateMachine buildStateMachine();

    @Override
    public void initialize() {
        startPose = getStartPose();
        sm = buildStateMachine();
    }
}
