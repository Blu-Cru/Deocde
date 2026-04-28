package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import java.util.function.BooleanSupplier;

public class WaitUntilSegment implements PathSegment {
    final PathSegment previous;
    private final BooleanSupplier condition;
    private final double timeoutMs;
    private double startTime;

    public WaitUntilSegment(PathSegment previous, BooleanSupplier condition, double timeoutMs) {
        this.previous = previous;
        this.condition = condition;
        this.timeoutMs = timeoutMs;
    }

    @Override
    public boolean isDone() {
        // Done if the condition returns TRUE OR the safety timeout expires
        return condition.getAsBoolean() || (System.currentTimeMillis() - startTime >= timeoutMs);
    }

    @Override
    public void startSegment() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean failed() {
        return false;
    }

    @Override
    public Pose2d getPose() {
        return previous.getPose();
    }

    @Override
    public void runSegment() {
        previous.runSegment();
    }
}