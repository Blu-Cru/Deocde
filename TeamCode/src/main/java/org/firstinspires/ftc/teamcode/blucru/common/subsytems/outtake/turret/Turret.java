package org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class Turret implements BluSubsystem, Subsystem {

    private TurretServos servos;
    private BluEncoder encoder;
    private PIDController controller;
    private PIDController controllerClose;  // PID controller for close-range (small errors)
    Vector2d target;
    AprilTagProcessor tags;
    public double headingOffset = 0;
    private double position;
    private Double lastSetpoint = null;
    private static final double TAG_CAMERA_FOCAL_LENGTH_PX = 563.115;

    private final double TICKS_PER_REV = 4000 * 212.0 / 35;
    // Far PID (large errors)
    public static double kP = 0.023;
    public static double kI = 0.02;
        public static double kD =   0.0018;
    // Close PID (small errors) - tune these to reduce oscillation near target
    public static double kPClose = 0.012;
    public static double kIClose = 0.008;
    public static double kDClose = 0.0009;
    
    public static double tagAngleGain = 1;
    public static double tagAngleDeadband = 0.35;
    public static double tagMaxCorrectionAngle = 12;
    public static int tagOffsetSaveStableFrames = 3;

    public static double acceptableError = 0.5;
    public static double powerClip = 1;
    public static double errorThreshold = 50;  // Switch to close PID when error is below this (degrees)

    // Tune these in Dashboard to offset the autoaim!
    // Positive offset = aim more right
    public static double locAutoAimAngleOffset = 0; // degrees
    public static double tagAutoAimPixelOffset = 0; // pixels
    public static boolean useShotLineOffset = true;
    public static double shotLineOffsetDeadbandIn = 0.0;
    public static double shotLineBlueGainDegPerIn = 0.33;
    public static double shotLineRedGainDegPerIn = -0.195;
    public static double shotLineBlueMaxOffsetDeg = 3.0;
    public static double shotLineRedMaxOffsetDeg = 3.0;

    // Hysteresis: number of consecutive "no tag" frames required before falling back to LOC
    public static int TAG_DROPOUT_THRESHOLD = 20;
    private int tagDropoutCounter = 0;

    public static double MAX_ANGLE = 270;
    public static double MIN_ANGLE = -270;

    public static double distFromCenter = 72.35 / 25.4;

    private int centeredTagFrames = 0;
    private enum LastAutoAimMode {
        TAG,
        LOC
    }

    private enum State {
        MANUAL,
        PID,
        LOCK_ON_GOAL
    }

    private State state;
    private LastAutoAimMode lastAutoAimMode;


    public Turret(BluCRServo servoLeft, BluCRServo servoRight, BluCRServo servoCenter,BluEncoder encoder) {
        servos = new TurretServos(servoLeft, servoRight,servoCenter);
        this.encoder = encoder;
        controller = new PIDController(kP, kI, kD);
        controllerClose = new PIDController(kPClose, kIClose, kDClose);
        state = State.MANUAL;
        lastAutoAimMode = LastAutoAimMode.LOC;
        //dealing with camera

    }

    @Override
    public void init() {
        servos.init();
        encoder.init();
    }

    @Override
    public void read() {
        servos.read();
        encoder.read();
    }

    @Override
    public void write() {
        switch (state) {
            case MANUAL:
                break;

            case LOCK_ON_GOAL:
                boolean tagAvailable = Robot.getInstance().turretCam.getDetection() != null
                        && (Robot.getInstance().turretCam.detectedThisLoop()
                            || Math.abs(System.nanoTime() - Robot.getInstance().turretCam.getDetection().frameAcquisitionNanoTime) < 55000000);

                if (tagAvailable) {
                    if (lastAutoAimMode == LastAutoAimMode.LOC || tagDropoutCounter > 0) {
                        controller.reset();
                        controllerClose.reset();
                        centeredTagFrames = 0;
                    }

                    if (lastAutoAimMode == LastAutoAimMode.LOC) {
                        lastAutoAimMode = LastAutoAimMode.TAG;
                    }

                    // Tag is visible — reset dropout counter and use camera-based aiming
                    tagDropoutCounter = 0;
                    tagBasedAutoAim(Robot.getInstance().turretCam.getDetection());
                } else if (lastAutoAimMode == LastAutoAimMode.TAG) {
                    // Tag was active but just dropped — use hysteresis before switching back
                    tagDropoutCounter++;

                    if (tagDropoutCounter < TAG_DROPOUT_THRESHOLD) {
                        // Brief camera dropouts are common; freeze the turret instead of
                        // coasting on the last tag power command.
                        holdTagDropout();
                    } else {
                        // Tag has been gone long enough — genuinely fall back to localization
                        lastAutoAimMode = LastAutoAimMode.LOC;
                        tagDropoutCounter = 0;
                        centeredTagFrames = 0;
                        controller.reset();
                        controllerClose.reset();
                        localizationBasedAutoAim();
                    }
                } else {
                    // Never had tag, or already fell back — use localization
                    localizationBasedAutoAim();
                }
                break;
            case PID:
                updateControlLoop();
                break;
        }

        servos.write();
        encoder.write();
    }


    public void setAngle(double angle) {
        angle = -angle;
        if (lastSetpoint == null || Math.abs(angle - lastSetpoint) > 1e-6) {
            controller.reset();
            controllerClose.reset();
            lastSetpoint = angle;
        }
        position = angle;
        state = State.PID;
    }

    public void setAngle(double angle, boolean switchState) {
        position = angle;
        if (switchState) {
            state = State.PID;
        }
    }

    public void setPower(double power) {
        servos.setPower(power);
        state = State.MANUAL;
    }

    public void setFieldCentricPositionAutoAim(double targetHeading, double robotHeading, boolean switchState) {
        position = resolveTargetAngle(getTurretAngleForFieldHeading(targetHeading, robotHeading), getAngle());
        if (switchState) {
            state = State.PID;
        }
    }

    public void lockOnGoal() {
        if (state != State.LOCK_ON_GOAL) {
            controller.reset();
            controllerClose.reset();
            centeredTagFrames = 0;
            tagDropoutCounter = 0;
            lastAutoAimMode = LastAutoAimMode.LOC;
        }

        state = State.LOCK_ON_GOAL;
    }

    public void toggleManual() {
        if (state != State.MANUAL) {
            state = State.MANUAL;
        } else {
            position = 0;
            state = State.PID;
        }
    }

    public boolean isTurretatGoal() {
        if(Math.abs(getTargetPosition() - getAngle()) <= 2) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isManual() {
        return state == State.MANUAL;
    }

    public void updatePID() {
        controller.setPID(kP, kI, kD);
        controllerClose.setPID(kPClose, kIClose, kDClose);
    }

    public void updateControlLoop() {
        updatePID();

        position = Range.clip(position, MIN_ANGLE, MAX_ANGLE);

        double currentAngle = getAngle();
        double error = position - currentAngle;
        //Globals.telemetry.addData("Error", error);
        
        // Select appropriate PID controller based on error magnitude
        PIDController activePID;
        if (Math.abs(error) < errorThreshold) {
            activePID = controllerClose;
        } else {
            activePID = controller;
        }
        
        double power = activePID.calculate(currentAngle, position);
        power = Range.clip(power, -powerClip, powerClip);

        // software safety limits
        if (currentAngle > MAX_ANGLE + 3 && power > 0) power = 0;
        if (currentAngle < MIN_ANGLE - 3 && power < 0) power = 0;

//        Globals.telemetry.addData("Turret Angle", currentAngle);
//        Globals.telemetry.addData("Turret Target", position);
//        Globals.telemetry.addData("PID Error", error);
//        Globals.telemetry.addData("PID Output", power);

        if (Math.abs(error) > acceptableError) {
            servos.setPower(power);
        } else {
            servos.setPower(0);
//            Globals.telemetry.addLine("Turret At Setpoint");
        }
    }

    public void updateControlLoopNoWrapping() {
        updateControlLoop();
    }

    public double getAngle() {
        return -encoder.getCurrentPos() * (360.0 / TICKS_PER_REV);
    }

    public double getEncoderPos() {
        return encoder.getCurrentPos();
    }

    public double getPower() {
        return servos.getPower();
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public double getFieldCentricTargetGoalAngle(Pose2d robotPose) {
        if(Globals.alliance == Alliance.RED){
//            target = Globals.turretTargetRedPose;
            target = new Vector2d(Globals.turretTargetRedX, Globals.turretTargetRedY);
        }else{
//            target = Globals.turretTargetBluePose;
            target = new Vector2d(Globals.turretTargetBlueX, Globals.turretTargetBlueY);
        }

        Vector2d robotVec = robotPose.vec();
        double robotHeadingDeg = Math.toDegrees(robotPose.getH());

        double turretCenterX =
                robotVec.getX() - distFromCenter * Math.cos(Math.toRadians(robotHeadingDeg));
        double turretCenterY =
                robotVec.getY() - distFromCenter * Math.sin(Math.toRadians(robotHeadingDeg));

        double dx = target.getX() - turretCenterX;
        double dy = target.getY() - turretCenterY;

        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public double getTurretAngleForFieldHeading(double targetHeading, double robotHeading) {
        return normalizeDegrees(targetHeading - robotHeading - 180);
    }

    public void  localizationBasedAutoAim(){
        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        double theoreticalTurretAngle = getTheoreticalTurretAngle(robotPose);
        double modeledTurretOffset = getShotLineTurretOffset(robotPose);
        double correctedTurretAngle = applyTurretOffset(theoreticalTurretAngle) - locAutoAimAngleOffset;

        position = resolveTargetAngle(correctedTurretAngle + modeledTurretOffset, getAngle());

        updateControlLoop();
    }

    public void tagBasedAutoAim(AprilTagDetection detection) {
        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        double totalPixelOffset = tagAutoAimPixelOffset + getShotLinePixelOffset(robotPose);
        double rawXDelta = detection.center.x - (320 - totalPixelOffset);
        double angleError = Math.toDegrees(Math.atan2(rawXDelta, TAG_CAMERA_FOCAL_LENGTH_PX)) * tagAngleGain;
        angleError = Range.clip(angleError, -tagMaxCorrectionAngle, tagMaxCorrectionAngle);

        if (Math.abs(angleError) < tagAngleDeadband) {
            angleError = 0;
        }

        position = getAngle() + angleError;
        updateControlLoop();

        if (angleError == 0) {
            centeredTagFrames++;
        } else {
            centeredTagFrames = 0;
        }

        if (centeredTagFrames >= tagOffsetSaveStableFrames) {
            saveTurretOffset(getAngle());
        }
    }

    private void holdTagDropout() {
        position = getAngle();
        controller.reset();
        controllerClose.reset();
        centeredTagFrames = 0;
        servos.setPower(0);
    }

    public void saveTurretOffset(double detectedAngle) {
        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        double theoreticalTurretAngle = getTheoreticalTurretAngle(robotPose);
        double modeledTurretOffset = getShotLineTurretOffset(robotPose);

        // Store the learned correction in turret space so localization can keep tracking
        // the goal after the tag disappears.
        headingOffset = normalizeDegrees(detectedAngle - (theoreticalTurretAngle + modeledTurretOffset));
    }

    public double applyTurretOffset(double turretAngle) {
        return turretAngle + headingOffset;
    }

    private double resolveTargetAngle(double desiredAngle, double referenceAngle) {
        double bestAngle = Double.NaN;
        double bestError = Double.POSITIVE_INFINITY;

        for (int turns = -2; turns <= 2; turns++) {
            double candidate = desiredAngle + 360.0 * turns;
            if (candidate < MIN_ANGLE || candidate > MAX_ANGLE) {
                continue;
            }

            double error = Math.abs(candidate - referenceAngle);
            if (error < bestError) {
                bestError = error;
                bestAngle = candidate;
            }
        }

        if (!Double.isNaN(bestAngle)) {
            return bestAngle;
        }

        return Range.clip(desiredAngle, MIN_ANGLE, MAX_ANGLE);
    }

    private double getTheoreticalTurretAngle(Pose2d robotPose) {
        double targetFieldHeading = getFieldCentricTargetGoalAngle(robotPose);
        double robotHeading = Math.toDegrees(robotPose.getH());
        return getTurretAngleForFieldHeading(targetFieldHeading, robotHeading);
    }

    private Vector2d getTurretCenterPosition(Pose2d robotPose) {
        Vector2d turretOffset = new Vector2d(-distFromCenter, 0).rotate(robotPose.getH());
        return robotPose.vec().addNotInPlace(turretOffset);
    }

    private double getShotLineTurretOffset(Pose2d robotPose) {
        if (!useShotLineOffset || robotPose == null) {
            return 0;
        }

        boolean isBlue = Globals.alliance == Alliance.BLUE;
        Vector2d turretCenter = getTurretCenterPosition(robotPose);
        double rawDeviation = isBlue
                ? (turretCenter.getX() - turretCenter.getY()) * Math.sqrt(2) / 2.0
                : (turretCenter.getX() + turretCenter.getY()) * Math.sqrt(2) / 2.0;

        double oneSidedDeviation = Math.max(0, rawDeviation - shotLineOffsetDeadbandIn);
        if (oneSidedDeviation <= 0) {
            return 0;
        }

        double gain = isBlue ? shotLineBlueGainDegPerIn : shotLineRedGainDegPerIn;
        double maxOffset = isBlue ? shotLineBlueMaxOffsetDeg : shotLineRedMaxOffsetDeg;
        double offsetMagnitude = Range.clip(
                oneSidedDeviation * gain,
                0,
                maxOffset
        );

        // Field testing on far blue showed the original sign assumption was
        // inverted, so keep blue/red opposite but flipped from the first pass.
        return isBlue ? -offsetMagnitude : offsetMagnitude;
    }

    private double getShotLinePixelOffset(Pose2d robotPose) {
        double turretOffsetDeg = getShotLineTurretOffset(robotPose);

        // Match the same sign convention used by the live pixel target.
        return -TAG_CAMERA_FOCAL_LENGTH_PX * Math.tan(Math.toRadians(turretOffsetDeg));
    }

    private double normalizeDegrees(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle <= -180) {
            angle += 360;
        }
        return angle;
    }


    @Override
    public void telemetry(Telemetry telemetry) {
        servos.telemetry();
        encoder.telemetry();
        telemetry.addData("Target Pos", position);
        telemetry.addData("Turret State", state);
    }

    @Override
    public void reset() {
        // encoder.reset();
    }
    public double getTargetPosition(){
        return position;
    }

    public boolean atTarget(){
        //1 deg tolerance
        /*Globals.telemetry.addData("Target Position", position);
        Globals.telemetry.addData("Angle", getAngle());*/
        return Math.abs(position - getAngle()) < 2;
    }
}
