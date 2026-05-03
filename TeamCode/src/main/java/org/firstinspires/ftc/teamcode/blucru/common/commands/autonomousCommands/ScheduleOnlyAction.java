package org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;

public class ScheduleOnlyAction implements Action {
    private final Command cmd;
    private boolean scheduled = false;

    public ScheduleOnlyAction(Command cmd) { this.cmd = cmd; }

    @Override
    public boolean run(TelemetryPacket packet) {
        if (!scheduled) {
            cmd.schedule();
            scheduled = true;
        }
        return false; // never blocks RR
    }
}

