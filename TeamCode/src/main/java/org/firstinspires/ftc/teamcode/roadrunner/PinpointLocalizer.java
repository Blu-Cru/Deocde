package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

@Config
public final class PinpointLocalizer implements Localizer {

    public static class Params {
        public double parYMM = 138.5;
        public double perpXMM = 94.5; // old 94.05
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection;
    public final GoBildaPinpointDriver.EncoderDirection initialPerpDirection;

    private final Supplier<Double> headingRad;  // REV hub IMU heading (rad)
    private final Supplier<Double> angVelRad;   // REV hub IMU ang vel (rad/s)

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(
            HardwareMap hardwareMap,
            double inPerTick,
            Pose2d initialPose,
            Supplier<Double> headingRad,
            Supplier<Double> angVelRad
    ) {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        this.headingRad = headingRad;
        this.angVelRad = angVelRad;

        double ticksPerMm = 19.894;
        double mmPerTick = 1.0 / ticksPerMm;

        driver.setEncoderResolution(ticksPerMm, DistanceUnit.MM);
        driver.setOffsets(
                PARAMS.parYMM,
                PARAMS.perpXMM,
                DistanceUnit.MM
        );

        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        // Keep if you want Pinpoint X/Y reset at init. Heading comes from REV IMU now.
        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();

        // Heading from REV hub IMU
        double heading = headingRad.get();

        // X/Y from Pinpoint, heading from REV IMU
        txPinpointRobot = new Pose2d(
                driver.getPosX(DistanceUnit.INCH),
                driver.getPosY(DistanceUnit.INCH),
                heading
        );

        Vector2d worldVelocity = new Vector2d(
                driver.getVelX(DistanceUnit.INCH),
                driver.getVelY(DistanceUnit.INCH)
        );

        // Convert world -> robot frame (RR convention)
        Vector2d robotVelocity = Rotation2d.fromDouble(-heading).times(worldVelocity);

        // Angular velocity from REV hub IMU
        double angVel = angVelRad.get();

        return new PoseVelocity2d(robotVelocity, angVel);
    }
}
