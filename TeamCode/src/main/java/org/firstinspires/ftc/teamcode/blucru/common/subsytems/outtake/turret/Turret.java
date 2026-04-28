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
    public double headingOffset = 0;
    private double position;
    private Double lastSetpoint = null;
    private static final double TAG_CAMERA_FOCAL_LENGTH_PX = 563.115;
    private double lastControlError = 0;
    private double lastControlPower = 0;
    private double lastRobotHeadingVelDegPerSec = 0;
    private double lastRobotTurnFeedForwardPower = 0;
    private double lastGoalTrackingRateDegPerSec = 0;
    private double lastGoalDistanceIn = 0;
    private boolean lastUsingClosePid = false;

    private final double TICKS_PER_REV = 4000 * 212.0 / 35;
    // Far PID (large errors)
    public static double kP = 0.023;
    public static double kI = 0.02;
        public static double kD =   0.0027;
    // Close PID (small errors) - tune these to reduce oscillation near target
    public static double kPClose = 0.012;
    public static double kIClose = 0.008;
    public static double kDClose = 0.0009;
    
    public static double tagAngleGain = 1;
    public static double tagAngleDeadband = 0.35;
    public static double tagMaxCorrectionAngle = 12;
    public static double tagHandoffMaxTurretError = 30;
    public static int tagOffsetSaveStableFrames = 3;

    public static double acceptableError = 0.5;
    public static double powerClip = 1;
    public static double errorThreshold = 30;  // Switch to close PID when error is below this (degrees)

    // Tune these in Dashboard to offset the autoaim!
    // Positive offset = aim more right
    public static double locAutoAimAngleOffset = 0; // degrees
    public static double tagAutoAimPixelOffset = 0; // pixels
    public static double leftShotSweepAngleOffsetDeg = -6.0;
    public static double middleShotSweepAngleOffsetDeg = 0.0;
    public static double rightShotSweepAngleOffsetDeg = 6.0;
    public static double leftShotSweepTagOffsetPx = 18.0;
    public static double middleShotSweepTagOffsetPx = 0.0;
    public static double rightShotSweepTagOffsetPx = -18.0;
    public static int goalSweepReadyFrames = 2;
    public static boolean useShotLineOffset = true;
    public static double shotLineOffsetDeadbandIn = 5.0;
    public static double shotLineBlueGainDegPerIn = 0.33;
    public static double shotLineRedGainDegPerIn = -0.32;
    public static double shotLineBlueMaxOffsetDeg = 3.0;
    public static double shotLineRedMaxOffsetDeg = 3.0;
    public static boolean useRobotTurnFeedForward = true;
    public static double robotTurnFeedForwardPowerPerDegPerSec = 0.00135;
    public static double robotTurnFeedForwardMaxPower = 0.45;
    public static boolean useLocalizationDuringTagDropout = true;

    // Hysteresis: number of consecutive "no tag" frames required before falling back to LOC
    public static int TAG_DROPOUT_THRESHOLD = 20;
    private int tagDropoutCounter = 0;

    public static double MAX_ANGLE = 300;
    public static double MIN_ANGLE = -300;

    public static double distFromCenter = 72.35 / 25.4;

    private int centeredTagFrames = 0;
    private boolean goalSweepEnabled = false;
    private GoalSweepStage goalSweepStage = GoalSweepStage.MIDDLE_SHOT;
    private Double goalSweepBaseAngle = null;

    public enum GoalSweepStage {
        LEFT_SHOT,
        MIDDLE_SHOT,
        RIGHT_SHOT
    }

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
                if (Robot.getInstance().turretCam == null) {
                    localizationBasedAutoAim();
                    break;
                }

                boolean tagAvailable = Robot.getInstance().turretCam.getDetection() != null
                        && (Robot.getInstance().turretCam.detectedThisLoop()
                            || Math.abs(System.nanoTime() - Robot.getInstance().turretCam.getDetection().frameAcquisitionNanoTime) < 55000000);

                if (tagAvailable && isCloseEnoughForTagAim()) {
                    if (lastAutoAimMode == LastAutoAimMode.LOC || tagDropoutCounter > 0) {
                        resetControllers(getAngle());
                        centeredTagFrames = 0;
                    }

                    if (lastAutoAimMode == LastAutoAimMode.LOC) {
                        lastAutoAimMode = LastAutoAimMode.TAG;
                    }

                    // Tag is visible — reset dropout counter and use camera-based aiming
                    tagDropoutCounter = 0;
                    tagBasedAutoAim(Robot.getInstance().turretCam.getDetection());
                } else if (tagAvailable) {
                    if (lastAutoAimMode == LastAutoAimMode.TAG) {
                        resetControllers(getAngle());
                    }

                    lastAutoAimMode = LastAutoAimMode.LOC;
                    tagDropoutCounter = 0;
                    centeredTagFrames = 0;
                    localizationBasedAutoAim();
                } else if (lastAutoAimMode == LastAutoAimMode.TAG) {
                    // Tag was active but just dropped — use hysteresis before switching back
                    tagDropoutCounter++;

                    if (tagDropoutCounter < TAG_DROPOUT_THRESHOLD) {
                        if (tagDropoutCounter == 1) {
                            resetControllers(getAngle());
                        }

                        centeredTagFrames = 0;

                        if (useLocalizationDuringTagDropout) {
                            // Bridge brief camera dropouts with localization so feedforward
                            // can keep tracking while the robot is turning.
                            localizationBasedAutoAim();
                        } else {
                            holdTagDropout();
                        }
                    } else {
                        // Tag has been gone long enough — genuinely fall back to localization
                        lastAutoAimMode = LastAutoAimMode.LOC;
                        tagDropoutCounter = 0;
                        centeredTagFrames = 0;
                        resetControllers(getAngle());
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
        setResolvedAngle(-angle, true);
    }

    public void setAngle(double angle, boolean switchState) {
        setResolvedAngle(-angle, switchState);
    }

    // Resets the PID controllers and primes them with one calculate() call so
    // their internal lastTimeStamp/period are seeded before the next real PID
    // step. Without priming, the first calculate() after reset() goes through
    // setSetPoint() which divides by stale period and can leave the output
    // saturated for the first loop tick after a setpoint change.
    private void resetControllers(double targetAngle) {
        controller.reset();
        controllerClose.reset();
        double currentAngle = getAngle();
        controller.calculate(currentAngle, targetAngle);
        controllerClose.calculate(currentAngle, targetAngle);
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
        disableGoalSweep();
        enterLockOnGoal();
    }

    public void lockOnGoalWithSweep() {
        enableGoalSweep();
        setGoalSweepStage(GoalSweepStage.LEFT_SHOT);
        enterLockOnGoal();
    }

    private void enterLockOnGoal() {
        if (state != State.LOCK_ON_GOAL) {
            resetControllers(getAngle());
            centeredTagFrames = 0;
            tagDropoutCounter = 0;
            lastAutoAimMode = LastAutoAimMode.LOC;
        }

        state = State.LOCK_ON_GOAL;
    }

    public void enableGoalSweep() {
        if (!goalSweepEnabled) {
            goalSweepEnabled = true;
            handleGoalSweepConfigChange();
        }
    }

    public void disableGoalSweep() {
        if (goalSweepEnabled || goalSweepStage != GoalSweepStage.MIDDLE_SHOT) {
            goalSweepEnabled = false;
            goalSweepStage = GoalSweepStage.MIDDLE_SHOT;
            goalSweepBaseAngle = null;
            handleGoalSweepConfigChange();
        }
    }

    public void setGoalSweepStage(GoalSweepStage stage) {
        if (goalSweepStage != stage) {
            goalSweepStage = stage;
            handleGoalSweepConfigChange();
        }
    }

    public boolean isGoalSweepLockedOnTag() {
        return goalSweepEnabled
                && state == State.LOCK_ON_GOAL
                && lastAutoAimMode == LastAutoAimMode.TAG
                && centeredTagFrames >= goalSweepReadyFrames;
    }

    public void beginGoalSweep() {
        enableGoalSweep();
        goalSweepBaseAngle = getAngle();
    }

    public void aimGoalSweepStage(GoalSweepStage stage) {
        setGoalSweepStage(stage);
        enableGoalSweep();

        if (goalSweepBaseAngle == null) {
            goalSweepBaseAngle = getAngle();
        }

        double desiredAngle = goalSweepBaseAngle + getGoalSweepAngleOffsetDeg(stage);
        setResolvedAngle(desiredAngle, true);
    }

    public boolean isGoalSweepStageAtTarget() {
        return state == State.PID && atTarget();
    }

    public double getAngularVelocityDegPerSec() {
        return -encoder.getVel() * (360.0 / TICKS_PER_REV);
    }

    public double getGoalSweepStageAngle(GoalSweepStage stage) {
        double base = goalSweepBaseAngle != null ? goalSweepBaseAngle : getAngle();
        return base + getGoalSweepAngleOffsetDeg(stage);
    }

    private void handleGoalSweepConfigChange() {
        centeredTagFrames = 0;
        if (state == State.LOCK_ON_GOAL) {
            resetControllers(getAngle());
        }
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

        if (position > MAX_ANGLE || position < MIN_ANGLE){
            position = normalizeDegrees(position);
        }
        position = Range.clip(position, MIN_ANGLE, MAX_ANGLE);

        double currentAngle = getAngle();
        double error = position - currentAngle;
        //Globals.telemetry.addData("Error", error);
        lastControlError = error;
        
        // Select appropriate PID controller based on error magnitude
        PIDController activePID;
        if (Math.abs(error) < errorThreshold) {
            activePID = controllerClose;
            lastUsingClosePid = true;
        } else {
            activePID = controller;
            lastUsingClosePid = false;
        }
        
        double power = activePID.calculate(currentAngle, position);
        power += getRobotTurnFeedForwardPower();
        power = Range.clip(power, -powerClip, powerClip);

        // software safety limits
        if (currentAngle > MAX_ANGLE + 3 && power > 0) power = 0;
        if (currentAngle < MIN_ANGLE - 3 && power < 0) power = 0;

//        Globals.telemetry.addData("Turret Angle", currentAngle);
//        Globals.telemetry.addData("Turret Target", position);
//        Globals.telemetry.addData("PID Error", error);
//        Globals.telemetry.addData("PID Output", power);

        if (Math.abs(error) > acceptableError) {
            lastControlPower = power;
            servos.setPower(power);
        } else {
            lastControlPower = 0;
            servos.setPower(0);
//            Globals.telemetry.addLine("Turret At Setpoint");
        }
    }

    public void updateControlLoopNoWrapping() {
        updateControlLoop();
    }

    private double getRobotTurnFeedForwardPower() {
        if (Robot.getInstance().sixWheelDrivetrain == null) {
            lastRobotHeadingVelDegPerSec = 0;
            lastRobotTurnFeedForwardPower = 0;
            lastGoalTrackingRateDegPerSec = 0;
            lastGoalDistanceIn = 0;
            return 0;
        }

        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        Pose2d robotVel = Robot.getInstance().sixWheelDrivetrain.getVel();
        lastRobotHeadingVelDegPerSec = Math.toDegrees(robotVel.getH());
        lastGoalTrackingRateDegPerSec = getGoalTrackingRateDegPerSec(robotPose, robotVel);

        if (state != State.LOCK_ON_GOAL || !useRobotTurnFeedForward) {
            lastRobotTurnFeedForwardPower = 0;
            return 0;
        }

        // Feed forward the rate that the turret target should move while tracking the field goal.
        lastRobotTurnFeedForwardPower = Range.clip(
                lastGoalTrackingRateDegPerSec
                        * robotTurnFeedForwardPowerPerDegPerSec,
                -robotTurnFeedForwardMaxPower,
                robotTurnFeedForwardMaxPower
        );
        return lastRobotTurnFeedForwardPower;
    }

    private double getGoalTrackingRateDegPerSec(Pose2d robotPose, Pose2d robotVel) {
        Vector2d turretCenter = getTurretCenterPosition(robotPose);
        Vector2d goal = getGoalTarget();
        double dx = goal.getX() - turretCenter.getX();
        double dy = goal.getY() - turretCenter.getY();
        double distSq = dx * dx + dy * dy;
        lastGoalDistanceIn = Math.sqrt(distSq);

        if (distSq < 1e-6) {
            return -lastRobotHeadingVelDegPerSec;
        }

        double robotHeading = robotPose.getH();
        double robotHeadingVel = robotVel.getH();
        double turretVelX = robotVel.getX() + distFromCenter * Math.sin(robotHeading) * robotHeadingVel;
        double turretVelY = robotVel.getY() - distFromCenter * Math.cos(robotHeading) * robotHeadingVel;
        double goalBearingVel = (dy * turretVelX - dx * turretVelY) / distSq;
        double turretTargetVel = goalBearingVel - robotHeadingVel;

        return Math.toDegrees(turretTargetVel);
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
        target = getGoalTarget();

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

    private Vector2d getGoalTarget() {
        if(Globals.alliance == Alliance.RED){
//            return Globals.turretTargetRedPose;
            return new Vector2d(Globals.turretTargetRedX, Globals.turretTargetRedY);
        }else{
//            return Globals.turretTargetBluePose;
            return new Vector2d(Globals.turretTargetBlueX, Globals.turretTargetBlueY);
        }
    }

    public double getTurretAngleForFieldHeading(double targetHeading, double robotHeading) {
        return normalizeDegrees(targetHeading - robotHeading - 180);
    }

    public void  localizationBasedAutoAim(){
        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        position = getLocalizationAutoAimTarget(robotPose);

        updateControlLoop();
    }

    public void tagBasedAutoAim(AprilTagDetection detection) {
        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        double totalPixelOffset = tagAutoAimPixelOffset
                + getShotLinePixelOffset(robotPose)
                + getGoalSweepPixelOffset();
        double angleError = getTagAngleError(detection, totalPixelOffset);

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
        resetControllers(position);
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

    private boolean isCloseEnoughForTagAim() {
        Pose2d robotPose = Robot.getInstance().sixWheelDrivetrain.getPos();
        double targetAngle = getLocalizationAutoAimTarget(robotPose);
        return Math.abs(targetAngle - getAngle()) <= tagHandoffMaxTurretError;
    }

    private double getLocalizationAutoAimTarget(Pose2d robotPose) {
        double theoreticalTurretAngle = getTheoreticalTurretAngle(robotPose);
        double modeledTurretOffset = getShotLineTurretOffset(robotPose);
        double correctedTurretAngle = applyTurretOffset(theoreticalTurretAngle) - locAutoAimAngleOffset;

        return resolveTargetAngle(correctedTurretAngle + modeledTurretOffset, getAngle());
    }

    private double getTagAngleError(AprilTagDetection detection, double totalPixelOffset) {
        double rawXDelta = detection.center.x - (320 - totalPixelOffset);
        double angleError = Math.toDegrees(Math.atan2(rawXDelta, TAG_CAMERA_FOCAL_LENGTH_PX)) * tagAngleGain;
        angleError = Range.clip(angleError, -tagMaxCorrectionAngle, tagMaxCorrectionAngle);

        if (Math.abs(angleError) < tagAngleDeadband) {
            angleError = 0;
        }

        return angleError;
    }

    private double resolveTargetAngle(double desiredAngle, double referenceAngle) {
        double bestAngle = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (int k = -2; k <= 2; k++) {
            double candidate = desiredAngle + 360.0 * k;
            if (candidate < MIN_ANGLE || candidate > MAX_ANGLE) {
                continue;
            }

            double distance = Math.abs(candidate - referenceAngle);
            if (distance < bestDistance) {
                bestDistance = distance;
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
        // Respect the sign of the gain to allow independent left/right tuning per alliance
        return Range.clip(
                oneSidedDeviation * gain,
                -maxOffset,
                maxOffset
        );
    }

    private double getShotLinePixelOffset(Pose2d robotPose) {
        double turretOffsetDeg = getShotLineTurretOffset(robotPose);

        // Match the same sign convention used by the live pixel target.
        // A positive turretOffsetDeg should result in a positive pixel offset
        // so that Localization and Tag aim correct in the same direction.
        return TAG_CAMERA_FOCAL_LENGTH_PX * Math.tan(Math.toRadians(turretOffsetDeg));
    }

    private double getGoalSweepPixelOffset() {
        if (!goalSweepEnabled) {
            return 0;
        }

        return getGoalSweepPixelOffsetForStage(goalSweepStage);
    }

    private double getGoalSweepPixelOffsetForStage(GoalSweepStage stage) {
        switch (stage) {
            case LEFT_SHOT:
                return leftShotSweepTagOffsetPx;
            case RIGHT_SHOT:
                return rightShotSweepTagOffsetPx;
            case MIDDLE_SHOT:
            default:
                return middleShotSweepTagOffsetPx;
        }
    }

    private double getGoalSweepAngleOffsetDeg(GoalSweepStage stage) {
        switch (stage) {
            case LEFT_SHOT:
                return leftShotSweepAngleOffsetDeg;
            case RIGHT_SHOT:
                return rightShotSweepAngleOffsetDeg;
            case MIDDLE_SHOT:
            default:
                return middleShotSweepAngleOffsetDeg;
        }
    }

    private void setResolvedAngle(double desiredAngle, boolean switchState) {
        double resolvedAngle = resolveTargetAngle(desiredAngle, getAngle());
        if (lastSetpoint == null || Math.abs(resolvedAngle - lastSetpoint) > 1e-6) {
            resetControllers(resolvedAngle);
            lastSetpoint = resolvedAngle;
        }
        position = resolvedAngle;
        if (switchState) {
            state = State.PID;
        }
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
        telemetry.addData("Goal Sweep", goalSweepEnabled ? goalSweepStage : "OFF");
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
