package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

import java.util.Arrays;
import java.util.function.Supplier;

@Config
public class Shooter implements BluSubsystem, Subsystem {

    public static double p = 0.01, i = 0, d = 0, f = 0.00058;
    public static double limit = 20;
    public static final double ZERO_ANGLE = 26;
    public static final double TOP_ANGLE = 50;
    public static final double SERVO_ROT_TO_HOOD_ROT = 260 / 28;
    public static final double SERVO_ANGLE_DELTA = TOP_ANGLE - ZERO_ANGLE;
    public static final double SERVO_POS = SERVO_ROT_TO_HOOD_ROT * SERVO_ANGLE_DELTA / 255.0;
    public static double idleSpeed = 0.4;

    private BluMotorWithEncoder shooter1;
    private BluMotorWithEncoder shooter2;
    public HoodLeft hoodLeft;
    public HoodMiddle hoodMiddle;
    public HoodRight hoodRight;

    /**
     * Optional pose source for RoadRunner autos (Pinpoint/localizer).
     * If provided, AUTO_AIM will use this instead of sixWheelDrivetrain.
     *
     * IMPORTANT: Do NOT import com.acmerobotics.roadrunner.Pose2d here if you also have your own Pose2d class.
     * We use the fully-qualified name in this file.
     */
    private Supplier<com.acmerobotics.roadrunner.Pose2d> rrPoseSupplier = null;

    /**
     * Call this from your RR auto after you create the TankDrive:
     * Robot.getInstance().shooter.setRRPoseSupplier(() -> drive.localizer.getPose());
     */
    public void setRRPoseSupplier(Supplier<com.acmerobotics.roadrunner.Pose2d> supplier) {
        this.rrPoseSupplier = supplier;
    }

    enum State {
        IDLE,
        VELOCITY,
        AUTO_AIM
    }

    private State state;
    double targetVel = 0;
    ShooterVelocityPID pid;

    public Shooter() {
        shooter1 = new BluMotorWithEncoder("shooter1", DcMotorSimple.Direction.REVERSE);
        shooter2 = new BluMotorWithEncoder("shooter2", DcMotorSimple.Direction.REVERSE);
        hoodLeft = new HoodLeft();
        hoodMiddle = new HoodMiddle();
        hoodRight = new HoodRight();
        state = State.IDLE;
        pid = new ShooterVelocityPID(p, i, d, f);
    }

    public double shooterAngleToPos(double angle) {
        return (SERVO_POS) / SERVO_ANGLE_DELTA * angle - (ZERO_ANGLE * SERVO_POS) / SERVO_ANGLE_DELTA;
    }

    @Override
    public void init() {
        shooter1.init();
        shooter2.init();
    }

    @Override
    public void read() {
        shooter1.read();
        shooter2.read();
    }

    @Override
    public void write() {
        switch (state) {
            case IDLE:
                targetVel = 0;
                break;

            case VELOCITY:
                shooter1.setPower(pid.calculateDeltaPower(shooter1.getVel(), targetVel));
                shooter2.setPower(shooter1.getPower());
                Globals.telemetry.addData("delta", pid.calculateDeltaPower(shooter1.getVel(), targetVel));
                break;

            case AUTO_AIM:
                // --- AUTO_AIM must never crash if drivetrain pose isn't available ---
                // We need:
                //  1) goal pose (Globals.shootingGoalLPose)
                //  2) robot pose (either RR localizer or sixWheelDrivetrain)
                if (Globals.shootingGoalLPose == null) {
                    Globals.telemetry.addLine("[Shooter] AUTO_AIM: shootingGoalLPose is null -> skipping");
                    break;
                }

                // Get robot position (x,y) in the same field coordinate system as shootingGoalLPose.
                Vector2d robotPosVec = getRobotFieldPositionVec();
                if (robotPosVec == null) {
                    Globals.telemetry.addLine("[Shooter] AUTO_AIM: no robot pose source -> skipping");
                    break;
                }

                // Compute distance to goal
                double dist = Math.sqrt(
                        Globals.shootingGoalLPose
                                .subtractNotInPlace(robotPosVec)
                                .getDist()
                );

                Globals.telemetry.addData("distance", dist);

                // Lookup hood angles + velocity from interpolation table
                double[] interpolations = ShooterAutoAimInterpolation.interpolate(dist);

                Globals.telemetry.addData("Interpolations", Arrays.toString(interpolations));

                double leftHoodAngle = interpolations[0];
                double middleHoodAngle = interpolations[1];
                double rightHoodAngle = interpolations[2];
                double vel = interpolations[3];

                // Apply hood + velocity control
                hoodLeft.setShooterAngle(leftHoodAngle);
                hoodMiddle.setShooterAngle(middleHoodAngle);
                hoodRight.setShooterAngle(rightHoodAngle);

                shooter1.setPower(pid.calculateDeltaPower(shooter1.getVel(), vel));
                shooter2.setPower(shooter1.getPower());
                break;
        }

        shooter1.write();
        shooter2.write();
        hoodLeft.write();
        hoodMiddle.write();
        hoodRight.write();
    }

    /**
     * Returns robot position as a Vector2d (x,y) for AUTO_AIM.
     * Priority:
     *  1) RR pose supplier (Pinpoint/localizer) if set (RR auto)
     *  2) sixWheelDrivetrain pose if present (teleop / non-RR)
     *  3) null if neither exists (AUTO_AIM will safely skip)
     */
    private Vector2d getRobotFieldPositionVec() {
        try {
            // 1) RoadRunner pose (RR auto)
            if (rrPoseSupplier != null) {
                com.acmerobotics.roadrunner.Pose2d p = rrPoseSupplier.get();
                if (p != null) {
                    return new Vector2d(p.position.x, p.position.y);
                }
            }

            // 2) sixWheel pose (teleop / non-RR)
            if (Robot.getInstance() != null &&
                    Robot.getInstance().sixWheelDrivetrain != null &&
                    Robot.getInstance().sixWheelDrivetrain.getPos() != null) {

                // getPos().vec() returns your team Vector2d
                return Robot.getInstance().sixWheelDrivetrain.getPos().vec();
            }
        } catch (Exception e) {
            Globals.telemetry.addData("[Shooter] AUTO_AIM pose error", e.getMessage());
        }

        return null;
    }

    public void shoot(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
        state = State.IDLE;
    }

    public void idle() {
        shooter1.setPower(idleSpeed);
        shooter2.setPower(idleSpeed);
        state = State.IDLE;
    }

    public void shootWithVelocity(double vel) {
        targetVel = vel;
        state = State.VELOCITY;
    }

    public double getVel() {
        return shooter1.getVel();
    }

    public double getPower() {
        return shooter1.getPower();
    }

    public void autoAim() {
        this.state = State.AUTO_AIM;
    }

    public void rampDownShooter() {
        this.state = State.IDLE;
        shooter1.setPower(0);
        shooter2.setPower(0);
    }

    public void setHoodAngle(double angle) {
        hoodLeft.setShooterAngle(angle);
        hoodMiddle.setShooterAngle(angle);
        hoodRight.setShooterAngle(angle);
    }

    public void setHoodAngleIndependent(double langle, double mangle, double rangle) {
        hoodLeft.setShooterAngle(langle);
        hoodMiddle.setShooterAngle(mangle);
        hoodRight.setShooterAngle(rangle);
    }

    public void setLeftHoodAngle(double angle) {
        hoodLeft.setShooterAngle(angle);
    }

    public void setMiddleHoodAngle(double angle) {
        hoodMiddle.setShooterAngle(angle);
    }

    public void setRightHoodAngle(double angle) {
        hoodRight.setShooterAngle(angle);
    }

    public double getHoodAngle() {
        return hoodLeft.getHoodAngle();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Shooter power", shooter1.getPower());
        hoodLeft.telemetry();
        hoodMiddle.telemetry();
        hoodRight.telemetry();
    }

    @Override
    public void reset() {
        shooter1.reset();
        shooter2.reset();
        hoodLeft.reset();
        hoodMiddle.reset();
        hoodRight.reset();
    }

    public void updatePID() {
        pid.setPIDF(p, i, d, f);
    }
}
