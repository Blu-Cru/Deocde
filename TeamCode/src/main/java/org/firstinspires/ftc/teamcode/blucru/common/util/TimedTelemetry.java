package org.firstinspires.ftc.teamcode.blucru.common.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * A utility to display telemetry messages for a specific duration of time.
 * This ensures that fast-looping OpModes don't instantly wipe important messages.
 */
public class TimedTelemetry {
    private static class TimedMessage {
        String message;
        ElapsedTime timer;
        double durationMs;

        public TimedMessage(String message, double durationMs) {
            this.message = message;
            this.durationMs = durationMs;
            this.timer = new ElapsedTime();
        }

        public boolean isExpired() {
            return timer.milliseconds() > durationMs;
        }
    }

    private List<TimedMessage> messages;
    private Telemetry telemetry;

    public TimedTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.messages = new ArrayList<>();
    }

    /**
     * Adds a message to be displayed for the given duration in milliseconds.
     */
    public void addLine(String message, double durationMs) {
        messages.add(new TimedMessage(message, durationMs));
    }

    /**
     * Call this inside your loop right before telemetry.update().
     * It will push all unexpired messages to the standard telemetry.
     */
    public void draw() {
        Iterator<TimedMessage> iterator = messages.iterator();
        while (iterator.hasNext()) {
            TimedMessage tm = iterator.next();
            if (tm.isExpired()) {
                iterator.remove();
            } else {
                telemetry.addLine(tm.message);
            }
        }
    }
}
