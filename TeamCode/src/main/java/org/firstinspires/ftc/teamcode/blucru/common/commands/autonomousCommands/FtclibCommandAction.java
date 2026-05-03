package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class FtclibCommandAction implements Action {
    private final Command command;
    private final boolean blockUntilFinished;
    private boolean started = false;

    public FtclibCommandAction(Command command) {
        this(command, true);
    }

    /**
     * @param command the FTCLib command to schedule
     * @param blockUntilFinished if true, the Action will return true while the command
     *                           is scheduled (blocking RR). If false, the Action will
     *                           schedule the command and immediately return false so
     *                           RR may continue.
     */
    public FtclibCommandAction(Command command, boolean blockUntilFinished) {
        this.command = command;
        this.blockUntilFinished = blockUntilFinished;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // Schedule exactly once.
        if (!started) {
            CommandScheduler.getInstance().schedule(command); // prefer scheduler.schedule()
            started = true;
            // telemetry marker for debugging
            try {
                packet.put("FtclibCommandAction", "Started: " + command.getClass().getSimpleName());
            } catch (Exception ignored) {
            }
        }

        // DO NOT call CommandScheduler.run() here.
        // Your OpMode loop should call it once per iteration.


        // Block the RR action until the command finishes (IF the command actually takes time).
        return false;
    }
}
