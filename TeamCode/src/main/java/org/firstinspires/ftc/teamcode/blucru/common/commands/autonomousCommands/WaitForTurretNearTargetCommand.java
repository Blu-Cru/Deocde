package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class WaitForTurretNearTargetCommand extends CommandBase {
    public static double acceptableTurretErrorDeg = 10.0;
    public static long timeoutMs = 1200;

    private final double targetErrorDeg;
    private final long maxWaitMs;
    private long startTimeMs;

    public WaitForTurretNearTargetCommand() {
        this(acceptableTurretErrorDeg, timeoutMs);
    }

    public WaitForTurretNearTargetCommand(double targetErrorDeg, long maxWaitMs) {
        this.targetErrorDeg = targetErrorDeg;
        this.maxWaitMs = maxWaitMs;
    }

    @Override
    public void initialize() {
        startTimeMs = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        if (Robot.getInstance().turret == null) {
            return true;
        }

        double turretError = Math.abs(
                Robot.getInstance().turret.getTargetPosition()
                        - Robot.getInstance().turret.getAngle()
        );
        return turretError <= targetErrorDeg
                || System.currentTimeMillis() - startTimeMs >= maxWaitMs;
    }
}
