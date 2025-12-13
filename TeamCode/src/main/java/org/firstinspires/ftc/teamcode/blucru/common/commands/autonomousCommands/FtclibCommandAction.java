package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

public class FtclibCommandAction implements Action {
    private final Command command;
    private boolean started = false;

    public FtclibCommandAction(Command command) {
        this.command = command;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // Schedule exactly once.
        if (!started) {
            CommandScheduler.getInstance().schedule(command); // prefer scheduler.schedule()
            started = true;
        }

        // DO NOT call CommandScheduler.run() here.
        // Your OpMode loop should call it once per iteration.

        // Block the RR action until the command finishes (IF the command actually takes time).
        return !command.isFinished();
    }
}
