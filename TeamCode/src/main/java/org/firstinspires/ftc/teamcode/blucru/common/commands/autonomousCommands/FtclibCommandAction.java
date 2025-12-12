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
        // First time we’re called, schedule the command
        if (!started) {
            command.schedule();
            started = true;
        }

        // Run the FTCLib scheduler once per RR loop
        CommandScheduler.getInstance().run();

        // Keep running this action while the command is not finished
        return !command.isFinished();
    }

}
