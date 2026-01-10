package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {

    public static class Params {
        public double parYTicks = 138.5;
        public double perpXTicks = 940.5; //old 94.05
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection;
    public final GoBildaPinpointDriver.EncoderDirection initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double ticksPerMm = 19.894;
        double mmPerTick = 1.0 / ticksPerMm;

        driver.setEncoderResolution(ticksPerMm, DistanceUnit.MM);
        driver.setOffsets(
                mmPerTick * PARAMS.parYTicks,
                mmPerTick * PARAMS.perpXTicks,
                DistanceUnit.MM
        );

        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        try {
            return txWorldPinpoint.times(txPinpointRobot);
        } catch (Exception e){
            throw new RuntimeException("Get Pose not working, txWorldPinpoint: " + txWorldPinpoint+ " ,Error: " + e.getMessage() );
        }
    }

    // In org/firstinspires/ftc/teamcode/roadrunner/PinpointLocalizer.java

    @Override
    public PoseVelocity2d update() {
        driver.update();

        if (Objects.requireNonNull(driver.getDeviceStatus())
                == GoBildaPinpointDriver.DeviceStatus.READY) {

            // FIX: Remove the negative sign here
            double heading = driver.getHeading(UnnormalizedAngleUnit.RADIANS);

            txPinpointRobot = new Pose2d(
                    driver.getPosX(DistanceUnit.INCH),
                    driver.getPosY(DistanceUnit.INCH),
                    heading
            );

            Vector2d worldVelocity = new Vector2d(
                    driver.getVelX(DistanceUnit.INCH),
                    driver.getVelY(DistanceUnit.INCH)
            );

            // This logic remains the same (Rotation2d handles the math)
            Vector2d robotVelocity =
                    Rotation2d.fromDouble(txPinpointRobot.heading.log())
                            .times(worldVelocity);

            // FIX: Remove the negative sign here as well
            double angVel = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

            return new PoseVelocity2d(robotVelocity, angVel);
        }

        // Small typo fix in your original code: 'asd' -> 'PoseVelocity2d'
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
