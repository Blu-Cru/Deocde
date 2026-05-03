package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization;

import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

import java.util.LinkedList;

public class PoseHistory {

    private static final long STORAGE_NANOSECONDS = 1_000_000_000L; // 1 second

    private final LinkedList<PoseMarker> poseList;

    public PoseHistory(){
        poseList = new LinkedList<>();
    }

    public void add(Pose2d pose, Pose2d vel){
        poseList.addFirst(new PoseMarker(pose, vel));

        // PoseMarker timestamps with System.nanoTime() — eviction must use the same clock.
        long now = System.nanoTime();
        while (!poseList.isEmpty() && now - poseList.getLast().nanoTime > STORAGE_NANOSECONDS){
            poseList.removeLast();
        }
    }

    public PoseMarker getPoseAtTime(long nanoTime){
        if (poseList.isEmpty()) return null;

        PoseMarker newest = poseList.getFirst();
        PoseMarker oldest = poseList.getLast();
        if (nanoTime >= newest.nanoTime) return newest;
        if (nanoTime <= oldest.nanoTime) return oldest;

        PoseMarker before = newest;
        PoseMarker after = newest;
        for (PoseMarker poseMarker : poseList){
            if (poseMarker.nanoTime < nanoTime){
                before = poseMarker;
                break;
            } else {
                after = poseMarker;
            }
        }

        Pose2d poseBefore = before.getPose();
        Pose2d poseAfter = after.getPose();

        long timeBefore = nanoTime - before.nanoTime;
        long timeAfter = after.nanoTime - nanoTime;
        long total = timeBefore + timeAfter;

        if (total <= 0) return before;

        double afterMultiplier = (double) timeBefore / total;
        double beforeMultiplier = (double) timeAfter / total;

        Pose2d poseBeforeForInterpolation = new Pose2d(
                poseBefore.getX() * beforeMultiplier,
                poseBefore.getY() * beforeMultiplier,
                poseBefore.getH() * beforeMultiplier);
        Pose2d poseAfterForInterpolation = new Pose2d(
                poseAfter.getX() * afterMultiplier,
                poseAfter.getY() * afterMultiplier,
                poseAfter.getH() * afterMultiplier);

        Pose2d interpolatedPose = new Pose2d(
                poseBeforeForInterpolation.vec().addNotInPlace(poseAfterForInterpolation.vec()),
                poseBeforeForInterpolation.getH() + poseAfterForInterpolation.getH());

        return new PoseMarker(interpolatedPose, before.getVel());
    }

    public void offset(Pose2d poseDelta){
        for (PoseMarker marker: poseList){
            marker.setPose(new Pose2d(marker.getPose().vec().addNotInPlace(poseDelta.vec()), marker.getPose().getH()));
        }
    }

    public String toString() {
        return poseList.toString();
    }
    public double size(){
        return poseList.size();
    }
}
