package org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

import java.util.LinkedList;

public class PoseHistory {

    private static double STORAGE_NANOSECONDS = 2 * Math.pow(10,9);

    private LinkedList<PoseMarker> poseList;


    public PoseHistory(){
        poseList = new LinkedList<>();
    }

    public void add(Pose2d pose, Pose2d vel){
        poseList.addFirst(new PoseMarker(pose, vel));

        //remove old poses
        long currentTime = (long) (System.currentTimeMillis() * Math.pow(10, 6));
        while (!poseList.isEmpty() && currentTime - poseList.getLast().nanoTime > STORAGE_NANOSECONDS){
            poseList.removeLast();
        }
    }

    public PoseMarker getPoseAtTime(long nanoTime){
        PoseMarker poseMarkerAfterTime = poseList.get(0);
        PoseMarker poseMarkerBeforeTime = poseList.get(0);
        Globals.telemetry.addData("list size", poseList.size());
        Globals.telemetry.addData("nanoTime", nanoTime);
        Globals.telemetry.addData("oth index time", poseList.get(0).nanoTime);
        Globals.telemetry.addData("last index time", poseList.getLast().nanoTime);
        for(PoseMarker poseMarker: poseList){
            if (poseMarker.nanoTime < nanoTime){
                poseMarkerBeforeTime = poseMarker;
                break;
            } else {
                poseMarkerAfterTime = poseMarker;
            }
        }

        Pose2d poseBefore = poseMarkerBeforeTime.getPose();
        Pose2d poseAfter = poseMarkerAfterTime.getPose();

        long timeBefore = nanoTime - poseMarkerBeforeTime.nanoTime;
        long timeAfter = poseMarkerAfterTime.nanoTime - nanoTime;

        long total = timeBefore + timeAfter;

        double beforeMultiplier = (double) timeBefore / total;
        double afterMultiplier = (double) timeAfter / total;

        Pose2d poseBeforeForInterpolation = new Pose2d(poseBefore.getX() * beforeMultiplier, poseBefore.getY() * beforeMultiplier, poseBefore.getH() * beforeMultiplier);
        Pose2d poseAfterForInterpolation = new Pose2d(poseAfter.getX() * afterMultiplier, poseAfter.getY() * afterMultiplier, poseAfter.getH() * afterMultiplier);

        Pose2d interpolatedPose = new Pose2d(poseBeforeForInterpolation.vec().addNotInPlace(poseAfterForInterpolation.vec()), poseBeforeForInterpolation.getH() + poseAfterForInterpolation.getH());

        return new PoseMarker(interpolatedPose, poseMarkerBeforeTime.getVel());
    }

    public void offset(Pose2d poseDelta){
        for (PoseMarker marker: poseList){
            marker.setPose(new Pose2d(marker.getPose().vec().addNotInPlace(poseDelta.vec()), marker.getPose().getH()));
        }
    }

    public String toString() {
        return poseList.toString();
    }
}
