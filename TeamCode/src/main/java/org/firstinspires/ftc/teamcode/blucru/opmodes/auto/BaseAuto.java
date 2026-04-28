package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public abstract class BaseAuto extends BluLinearOpMode {

    public StateMachine sm;
    public Pose2d startPose;

    public abstract Pose2d getStartPose();

    public abstract StateMachine buildStateMachine();
    public void periodic() {}

    /**
     * Subclasses override this to add auto-specific telemetry. Called from the
     * AUTO section of the dispatcher's structured layout. Don't print state or
     * match time here — the dispatcher already does.
     */
    public void autoTelemetry(Telemetry telemetry) {}

    @Override
    public void initialize() {
        sm = buildStateMachine();
        startPose = getStartPose();
    }
}
