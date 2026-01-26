package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

import java.util.ArrayList;
import java.util.HashMap;

public class PIDPath implements Path {

    ArrayList<PathSegment> segments;
    ArrayList<Callback> callbacks;
    HashMap<Integer, ArrayList<Command>> commands; // each segment has a list of commands
    int segmentIndex;
    boolean pathDone;

    public PIDPath(ArrayList<PathSegment> segments, HashMap<Integer, ArrayList<Command>> commands) {
        this(segments, commands, new ArrayList<>(segments.size()));
    }

    public PIDPath(ArrayList<PathSegment> segments, HashMap<Integer, ArrayList<Command>> commands,
            ArrayList<Callback> callbacks) {
        this.segments = segments;
        this.callbacks = callbacks;
        this.commands = commands;
        segmentIndex = 0;
        pathDone = false;
    }

    @Override
    public Path start() {
        pathDone = false;

        // start first segment
        segments.get(0).startSegment();
        segmentIndex = 0;

        try {
            // schedule all commands
            for (Command c : commands.get(segmentIndex)) {
                c.schedule();
            }
        } catch (NullPointerException e) {
            // comes here if the command is null
            Log.e("PID Path", "error scheduling command, null pointer");
        }

        try {
            // run callbacks
            callbacks.get(segmentIndex).run();
        } catch (NullPointerException e) {
            Log.e("PID Path", "error running callback, was a null pointer");
        }

        return this;
    }

    @Override
    public void run() {
        if (isDone()) {
            // it should exit
            return;
        }
        PathSegment currSegment = segments.get(segmentIndex);
        currSegment.runSegment();

        if (currSegment.isDone()) {
            // increase segment index
            segmentIndex++;
            if (isDone()) {
                // exit if done
                endSixWheel();
                return;
            }

            try {
                // schedule all commands
                for (Command c : commands.get(segmentIndex)) {
                    c.schedule();
                }
            } catch (Exception e) {
                // comes here if the command is null
                Log.e("PID Path", "error scheduling command, " + e.getMessage());
            }

            try {
                // run callbacks
                callbacks.get(segmentIndex).run();
            } catch (Exception e) {
                Log.e("PID Path", "error running callback, " + e.getMessage());
            }

            try {
                // start path
                segments.get(segmentIndex).startSegment();
            } catch (Exception e) {
                Log.e("PID Path", "error running next segment, " + e.getMessage());
            }

        }
    }

    public boolean failed() {
        return segments.get(segmentIndex).failed();
    }

    @Override
    public boolean isDone() {
        return segmentIndex >= segments.size() || pathDone;
    }

    @Override
    public void endMecanum() {
        Robot.getInstance().mecanumDrivetrain.switchToIdle();
        pathDone = true;
    }

    @Override
    public void endSixWheel() {
        Robot.getInstance().sixWheelDrivetrain.switchToIdle();
    }

    public void telemetry() {
        Globals.telemetry.addData("Path done: ", isDone());
        Globals.telemetry.addData("Path failed: ", failed());
        Globals.telemetry.addData("Path index", segmentIndex);
        Globals.telemetry.addData("Path length", segments.size());
    }
}
