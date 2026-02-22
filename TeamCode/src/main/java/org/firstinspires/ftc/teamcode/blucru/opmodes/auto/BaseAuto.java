package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public abstract class BaseAuto extends BluLinearOpMode {

    public StateMachine sm;
    public Pose2d startPose;

    public abstract Pose2d getStartPose();

    public abstract StateMachine buildStateMachine();

    @Override
    public void initialize() {
        sm = buildStateMachine();
        startPose = getStartPose();
    }
}
