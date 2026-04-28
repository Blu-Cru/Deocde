package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

import java.util.ArrayList;
import java.util.HashMap;

public class PIDPathBuilder{

    ArrayList<PathSegment> segments;
    HashMap<Integer, ArrayList<Command>> commands;
    ArrayList<Callback> callbacks;

    public PIDPathBuilder() {
        segments = new ArrayList<>();
        commands = new HashMap<>();
        callbacks = new ArrayList<>();
    }

    public PIDPathBuilder addPoint(GoToPointSegment segment){
        segments.add(segment);
        return this;
    }

    public PIDPathBuilder addPoint(Pose2d pose){
        return addPoint(new GoToPointSegment(pose));
    }
    public PIDPathBuilder addPoint(Pose2d pose, double xyTol){
        return addPoint(new GoToPointSegment(pose, xyTol));
    }
    public PIDPathBuilder addPoint(Pose2d pose, boolean stopRequiredAtEnd){
        segments.add(new GoToPointSegment(pose, stopRequiredAtEnd));
        return this;
    }
    public PIDPathBuilder addPoint(Pose2d pose, double xyTol, boolean stopRequiredAtEnd){
        segments.add(new GoToPointSegment(pose, xyTol, stopRequiredAtEnd));
        return this;
    }

    public PIDPathBuilder addMappedPoint(Pose2d pose){
        return addPoint(Globals.mapPose(pose));
    }
    public PIDPathBuilder addMappedPoint(Pose2d pose, double xyTol){
        return addPoint(Globals.mapPose(pose), xyTol);
    }
    public PIDPathBuilder addMappedPoint(double x, double y, double h, double xyTol){
        return addMappedPoint(new Pose2d(x,y,h), xyTol);
    }
    public PIDPathBuilder addMappedPoint(double x, double y, double h){
        return addMappedPoint(new Pose2d(x,y,h));
    }
    public PIDPathBuilder addMappedPoint(double x, double y, double h, boolean stopRequired){
        return addMappedPoint(new Pose2d(x,y,h), stopRequired);
    }
    public PIDPathBuilder addMappedPoint(Pose2d pose, boolean stopRequired){
        segments.add(new GoToPointSegment(pose, stopRequired));
        return this;
    }

    public PIDPathBuilder addTurnToPoint(Vector2d drivePoint, Vector2d turnToPoint, double tolerance){
        segments.add(new TurnToPointSegment(drivePoint, turnToPoint, tolerance));
        return this;
    }
    public PIDPathBuilder addTurnToPoint(Vector2d drivePoint, Vector2d turnPoint){
        segments.add(new TurnToPointSegment(drivePoint, turnPoint));
        return this;
    }
    public PIDPathBuilder addMappedTurnToPoint(Vector2d drivePoint, Vector2d turnPoint, double tolerance){
        return addTurnToPoint(Globals.mapPoint(drivePoint), Globals.mapPoint(turnPoint), tolerance);
    }
    public PIDPathBuilder addMappedTurnToPoint(Vector2d drivePoint, Vector2d turnPoint){
        return addTurnToPoint(Globals.mapPoint(drivePoint), Globals.mapPoint(turnPoint));
    }

    public PIDPathBuilder addSegment(PathSegment segment){
        segments.add(segment);
        return this;
    }

    public PIDPathBuilder schedule(Command command){
        commands.computeIfAbsent(segments.size(), k -> new ArrayList<Command>());
        commands.get(segments.size()).add(command);
        return this;
    }

    public PIDPathBuilder setPower(double power){
        return schedule(new MecanumDrivetrainSetPowerCommand(power));
    }

    public PIDPathBuilder callback(Callback callback){
        while(callbacks.size() <= segments.size()){
            //add a dead callback
            callbacks.add(null);
        }
        callbacks.add(segments.size(), callback);
        return this;
    }

    /**
     *units: milliseconds
     */
    public PIDPathBuilder waitMilliseconds(double milliseconds){
        //run a wait segment with the last position
        segments.add(new WaitSegment(segments.get(segments.size() - 1), milliseconds));
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
