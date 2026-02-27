package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive;

import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

public class PosLockPID {

    private PDController linController;
    private PDController rotController;
    public static double pLin = 0, dLin = 0;
    public static double pRot = 0, dRot = 0;

    public PosLockPID(){
        linController = new PDController(pLin, dLin);
        rotController = new PDController(pRot, dRot);
    }

    public double getLinPower(Pose2d currPose, Pose2d target, Pose2d vel){
        //transform st currPose is center
        Vector2d targetVec = target.vec().subtractNotInPlace(currPose.vec());
        Vector2d velVec = vel.vec();
        //rotate st par is on the x-axis
        targetVec.rotateInPlace(currPose.getH());
        velVec.rotateInPlace(currPose.getH());
        Globals.telemetry.addData("Target Vec", targetVec);
        Globals.telemetry.addData("Vel Vec", velVec);
        //usable stuff are x vals
        double usableDist = targetVec.getX();
        double usableVel = velVec.getX();

        //pid now
        return linController.calculate(usableDist, -usableVel);

    }

    public double getRotPower(Pose2d currPose, Pose2d target, Pose2d vel){
        //just pid
        return linController.calculate(target.getH() - currPose.getH(), vel.getH());
    }

}
