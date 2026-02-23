package org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain;

import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.SixWheelDriveBase;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

@Config
public class Limelight extends SixWheelDriveBase implements BluSubsystem, Subsystem {
    //TODO: Add Logic to prioritize other artifacts
    private Limelight3A limelight;
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public double getDistance() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double targetOffsetAngle_Vertical = result.getTy();
            double limelightMountAngleDegrees = -15;  // TODO: Tune this, how many degrees back is your limelight rotated from perfectly vertical?
            double limelightLensHeightInches = 20.0; // TODO: Tune this, distance from the center of the Limelight lens to the floor
            double goalHeightInches = 2.5; // distance from the target to the floor
            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        }
        return 0; // Backup in case no result
    }

    public double getXOffset() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double angleRadians = Math.toRadians(result.getTx());
            return getDistance() * Math.tan(angleRadians);
        }
        return 0;
    }

    public Pose2d getPoseOfArtifact() {
        double distancetoartifact = getDistance();
        double xoffset = getXOffset();
        double forwarddistance = sqrt(Math.pow(distancetoartifact, 2) - Math.pow(xoffset, 2));
        double x = localizer.getX() + (xoffset * Math.cos(localizer.getHeading()) - forwarddistance * Math.sin(localizer.getHeading()));
        double y = localizer.getY() + (xoffset * Math.sin(localizer.getHeading()) + forwarddistance * Math.cos(localizer.getHeading()));
        double deltaX = x - localizer.getX();
        double deltaY = y - localizer.getY();
        double headingToArtifact = Math.atan2(deltaY, deltaX);
        return new Pose2d(x, y, headingToArtifact);
    }


}
