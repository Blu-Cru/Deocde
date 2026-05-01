package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootFlipTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferThenLockOnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class FarBluePreload extends BaseAuto {
    // Turret angle to be set while the robot is driving to shooting position
    double turretAnglePreaim = -116;

    // Turret angle to be set to once the bot reaches the shooting position
    double shootVeloLeft = 1420;
    double shootVeloMiddle = 1450;
    double shootVeloRight = 1430;
    Point2d shootingPoint = new Point2d(47, -9);

    double hood = 50;

    double pickupWallY = -62;
    double pickupWallX = 61; // default for hp
    private static final double CYCLE_HP_PATH_MIN_X = 53.0;

    enum State {
        PRELOAD,

        PARK,
        IDLE
    }

    Path currentPath;
    ElapsedTime matchTimer;
    // Time threshold to start a new cycle (30s match - ~5s per cycle)
    final double CYCLE_TIME_THRESHOLD = 26.0;
    boolean shouldReadColorSensors = false;

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(62, -7, Math.toRadians(-90));
    }

    @Override
    public void initialize() {
        Globals.setAlliance(Alliance.BLUE);
        addAutoSubsystems(true);


        //TODO: SWAP THE Y OFFSET BASED ON ALLIANCE. POSITIVE Y = TO THE LEFT. NEGATIVE Y = TO THE RIGHT.
        ballDetector.setCameraParameters(11.3, -6.5, 13.0, 15.0);
        ballDetector.activate();

        shooter.setHoodAngle(hood);
        shooter.write();
        elevator.setMiddle();
        elevator.write();
        transfer.setAllMiddle();
        transfer.write();
        turret.resetEncoder();
        turret.write();
        sixWheel.reset();
        sixWheel.write();
        intake.resetEncoder();

        matchTimer = new ElapsedTime();
        super.initialize();
    }

    @Override
    public StateMachine buildStateMachine() {
        return new StateMachineBuilder()
                .state(State.PRELOAD)
                .transition(() -> currentPath != null && currentPath.isDone(), State.PARK, () -> {
                    //shouldReadColorSensors = true;
                    shouldReadColorSensors = false;
                    startPath(buildParkPath());
                })



                .state(State.PARK)
                .transition(() -> currentPath != null && currentPath.isDone(), State.IDLE, () -> {
                    currentPath = null;
                })

                .state(State.IDLE)

                .build();
    }

    @Override
    public void initializePeriodic() {
//        turret.read();
//        if (driver1.pressedA()) {
//            turret.setAngle(turretAnglePreaim);
//        }
//        turret.write();
//        telemetry.addLine("--- INIT ---");
//        telemetry.addLine("Press A to set turret to Pre-Aim (-116)");
    }

    @Override
    public void onStart() {
        matchTimer.reset();
        Globals.setAlliance(Alliance.BLUE);
        shooter.shootWithVelocityIndependent(1460, 1520, 1500);
        sixWheel.setPosition(startPose);

        startPath(buildPreloadPath());
        sm.setState(State.PRELOAD);
        sm.start();
    }

    public void periodic() {
        if (currentPath != null) {
            currentPath.run();
        }
        sm.update();
    }

    @Override
    public void autoTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("LL Clump X", ballDetector.getClumpFieldX());
        telemetry.addData("Intake X", pickupWallX);
    }

    private void startPath(Path path) {
        currentPath = path;
        currentPath.start();
    }

    private void stopIntakeAndPath() {
        if (currentPath != null) {
            currentPath.endSixWheel();
        }
        scheduleVelocityTransferThenLockOn(0, shootVeloLeft, shootVeloMiddle, shootVeloRight, hood);
    }

    private boolean updateIntakeXPosition() {
        if (ballDetector.hasValidClump()) {
            double fieldX = ballDetector.getClumpFieldX();
            double minX = 23; // x value the closest we would ever want to intake towards the gate
            double maxX = 61; // max x value we would want to intake towards the wall
            pickupWallX = Range.clip(fieldX, minX, maxX);  //limits the x value from which we intake to a set range
            return true;
        }

        return false;
    }

    private boolean isTransferFull() {
        // Only read color sensors during intake states to avoid I2C overhead
        if (!shouldReadColorSensors)
            return false;
        return elevator.isFull();
    }

    private Path buildPreloadPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(62, -7),
                        new Point2d(62, -8)
                }, 50)
                .callback(()->{
                    new TurnTurretToPosCommand(turretAnglePreaim).schedule();
                })
                .waitMilliseconds(1700)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand()).schedule();
                })
                .waitMilliseconds(500)
                .build();
    }

    private Path buildParkPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(62, -7),
                        new Point2d(60,pickupWallY)
                },100)
                .waitMilliseconds(1000)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(60, pickupWallY),
                        new Point2d(57, -40)
                }, 5000)
                .build();
    }

    private void scheduleVelocityTransferThenLockOn(int delayBeforeTransferMs,
                                                    double leftVel,
                                                    double middleVel,
                                                    double rightVel,
                                                    Double hoodAngle) {
        new SequentialCommandGroup(
                new SetShooterVelocityIndependentCommand(leftVel, middleVel, rightVel),
                new WaitCommand(delayBeforeTransferMs),
                hoodAngle == null
                        ? new AutonomousTransferThenLockOnCommand()
                        : new AutonomousTransferThenLockOnCommand(hoodAngle)
        ).schedule();
    }
    private void scheduleVelocityTransferThenLockOn(int delayBeforeTransferMs) {
        new SequentialCommandGroup(
                new AutoAimCommand(),
                new WaitCommand(delayBeforeTransferMs),
                new AutonomousTransferThenLockOnCommand()

        ).schedule();
    }
}
