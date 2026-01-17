package org.firstinspires.ftc.teamcode.purePursuit.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.purePursuit.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.purePursuit.Math.Geometry.Rotation2d;

import static org.firstinspires.ftc.teamcode.purePursuit.Utils.ExtraUtils.drawRobot;

public class Odometry extends Subsystem {
	GoBildaPinpointDriver pinpoint;

	double velocityX = 0;
	double velocityY = 0;
	double velocityTheta = 0;

	Vector position = new Vector(3);

	ElapsedTime timer = new ElapsedTime();
	@Override
	public void initAuto(HardwareMap hwMap) {
		pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

		// Configure the Pinpoint driver
		// Set encoder resolution - adjust these values based on your specific pods
		pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

		// Set encoder directions - adjust if needed based on your robot
		pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
									  GoBildaPinpointDriver.EncoderDirection.FORWARD);

		// Reset position and calibrate IMU
		// Robot must be stationary during initialization
		pinpoint.resetPosAndIMU();
	}

	@Override
	public void periodic() {
		// Update the Pinpoint to get latest sensor data
		pinpoint.update();

		// Get position from Pinpoint (convert from mm to inches)
		double x = pinpoint.getPosX(DistanceUnit.INCH);
		double y = pinpoint.getPosY(DistanceUnit.INCH);
		double theta = pinpoint.getHeading(AngleUnit.RADIANS);

		// Get velocities from Pinpoint (convert from mm/sec to inches/sec)
		velocityX = pinpoint.getVelX(DistanceUnit.INCH);
		velocityY = pinpoint.getVelY(DistanceUnit.INCH);
		velocityTheta = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

		// Update position vector
		position.set(x, 0);
		position.set(y, 1);
		position.set(theta, 2);

		// Dashboard telemetry
		Dashboard.packet.put("pinpoint x", x);
		Dashboard.packet.put("pinpoint y", y);
		Dashboard.packet.put("pinpoint heading", theta);
		Dashboard.packet.put("pinpoint vx", velocityX);
		Dashboard.packet.put("pinpoint vy", velocityY);
		Dashboard.packet.put("pinpoint vtheta", velocityTheta);
		Dashboard.packet.put("pinpoint status", pinpoint.getDeviceStatus().toString());
		Dashboard.packet.put("pinpoint frequency", pinpoint.getFrequency());

		timer.reset();

		drawRobot(position, Dashboard.packet);
	}

	@Override
	public void shutdown() {

	}

	public void setEstimate(Vector estimate) {
		// Set the position on the Pinpoint driver
		org.firstinspires.ftc.robotcore.external.navigation.Pose2D newPose =
				new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
						DistanceUnit.INCH,
						estimate.get(0),
						estimate.get(1),
						AngleUnit.RADIANS,
						estimate.get(2)
				);
		pinpoint.setPosition(newPose);
		position = estimate;
	}

	public Vector getPosition() {
		return position;
	}

	public Pose2d getPose() {
		return new Pose2d(
				position.get(0),
				position.get(1),
				new Rotation2d(position.get(2))
		);
	}

	public Vector getVelocity() {
		return new Vector(new double[] {
				velocityX,
				velocityY,
				velocityTheta
		});
	}

	// Legacy methods for compatibility - these may not be meaningful with Pinpoint
	// but are kept in case they're called elsewhere
	public double getLeftVelocity() {
		// Return forward velocity as approximation
		return velocityX;
	}

	public double getRightVelocity() {
		// Return forward velocity as approximation
		return velocityX;
	}

	public double getLeftDelta() {
		// Return forward delta as approximation
		return velocityX * timer.seconds();
	}

	public double getRightDelta() {
		// Return forward delta as approximation
		return velocityX * timer.seconds();
	}
}
