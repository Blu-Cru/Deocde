package org.firstinspires.ftc.teamcode.blucru.common.pathing;


import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;

import java.util.ArrayList;
import java.util.HashMap;

public class SixWheelPIDPathBuilder {

    ArrayList<PathSegment> segments;
    HashMap<Integer, ArrayList<Command>> commands;
    ArrayList<Callback> callbacks;

    public SixWheelPIDPathBuilder() {
        segments = new ArrayList<>();
        commands = new HashMap<>();
        callbacks = new ArrayList<>();
    }

    public SixWheelPIDPathBuilder addPurePursuitPath(Point2d[] path, double maxTime){
        segments.add(new PurePursuitSegment(path,maxTime));
        return this;
    }

    public SixWheelPIDPathBuilder addMappedPurePursuitPath(Point2d[] path, double maxTime){
        for (int i = 0; i<path.length; i++){
            path[i] = Globals.mapPoint(path[i]);
        }

        return addPurePursuitPath(path, maxTime);
    }

    public SixWheelPIDPathBuilder addTurnTo(double heading, double maxTime){
        return addSegment(new TurnToSegment(heading, maxTime));
    }

    public SixWheelPIDPathBuilder addMappedTurnTo(double heading, double maxTime){
        return addSegment(new TurnToSegment(Globals.mapAngle(heading), maxTime));
    }

    public SixWheelPIDPathBuilder addSegment(PathSegment segment){
        segments.add(segment);
        return this;
    }

    public SixWheelPIDPathBuilder schedule(Command command){
        commands.computeIfAbsent(segments.size(), k -> new ArrayList<Command>());
        commands.get(segments.size()).add(command);
        return this;
    }

    public SixWheelPIDPathBuilder setPower(double power){
        return schedule(new MecanumDrivetrainSetPowerCommand(power));
    }

    public SixWheelPIDPathBuilder callback(Callback callback){
        // Fill callbacks up to current segment
        while(callbacks.size() < segments.size()){
            callbacks.add(null);
        }
        // Set callback for the segment that was just added
        callbacks.set(segments.size() - 1, callback);
        return this;
    }

    /**
     *units: milliseconds
     */
    public SixWheelPIDPathBuilder waitMilliseconds(double milliseconds){
        segments.add(new WaitSegment(milliseconds));
        return this;
    }
    public PIDPath build(){
        while(callbacks.size() <= segments.size()){
            //fill with null callbacks
            callbacks.add(null);
        }
        return new PIDPath(segments, commands, callbacks);
    }

    public Path start(){return this.build().start();}
}
