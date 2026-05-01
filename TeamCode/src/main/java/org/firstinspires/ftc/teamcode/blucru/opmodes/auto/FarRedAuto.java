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
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FarAutoTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class FarRedAuto extends BaseAuto {
    // Turret angle to be set while the robot is driving to shooting position
    double turretAnglePreaim = 116; // TODO: Change for Red

    // Turret angle to be set to once the bot reaches the shooting position
    double shootVeloLeft = 1420;
    double shootVeloMiddle = 1450;
    double shootVeloRight = 1430;
    Point2d shootingPoint = new Point2d(47, 8);

    double hood = 50;

    double pickupWallY = 63; // TODO: Change for Red
    double pickupWallX = 60; // TODO: Change for Red
    private static final double CYCLE_HP_PATH_MIN_X = 53.0; // TODO: Change for Red

    enum State {
        PRELOAD,
        INTAKE_SPIKE,
        SHOOT_SPIKE,
        INTAKE_HP,
        SHOOT_HP,
        INTAKE_CYCLE,
        SHOOT_CYCLE,
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
        return new Pose2d(62, 6, Math.PI / 2);
    }

    @Override
    public void initialize() {
        Globals.setAlliance(Alliance.RED);
        addAutoSubsystems(true);


        //TODO: SWAP THE Y OFFSET BASED ON ALLIANCE. POSITIVE Y = TO THE LEFT. NEGATIVE Y = TO THE RIGHT. already done for red
        ballDetector.setCameraParameters(11.3, 6.5, 13.0, 15.0);
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
                .transition(() -> currentPath != null && currentPath.isDone(), State.INTAKE_SPIKE, () -> {
                    //shouldReadColorSensors = true;
                    shouldReadColorSensors = false;
                    startPath(buildIntakeSpikePath());
                })

                .state(State.INTAKE_SPIKE)
                // Transition if path completes normally
                .transition(() -> currentPath != null && currentPath.isDone(), State.SHOOT_SPIKE, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildShootSpikePath());
                })
//                // Transition early if transfer is full
                .transition(() -> isTransferFull(), State.SHOOT_SPIKE, () -> {
                    shouldReadColorSensors = false;
                    stopIntakeAndPath();
                    startPath(buildShootSpikePath());
                })

                .state(State.SHOOT_SPIKE)
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() < CYCLE_TIME_THRESHOLD, State.INTAKE_HP, () -> {
                    //shouldReadColorSensors = true;
                    shouldReadColorSensors = false;
                    startPath(buildIntakeHPPath());
                })
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() >= CYCLE_TIME_THRESHOLD, State.PARK, () -> {
                    startPath(buildParkPath());
                })
                .state(State.INTAKE_HP)
                // Transition if path completes normally
                .transition(() -> currentPath != null && currentPath.isDone(), State.SHOOT_HP, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildShootHPPath());
                })
                // Transition early if transfer is full
                .transition(() -> isTransferFull(), State.SHOOT_SPIKE, () -> {
                    shouldReadColorSensors = false;
                    stopIntakeAndPath();
                    startPath(buildShootHPPath());
                })
                .state(State.SHOOT_HP)
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() < CYCLE_TIME_THRESHOLD, State.INTAKE_CYCLE, () -> {
                    //shouldReadColorSensors = true;
                    shouldReadColorSensors = true;
                    startCycleIntakePath();
                })
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() >= CYCLE_TIME_THRESHOLD, State.PARK, () -> {
                    startPath(buildParkPath());
                })
                .state(State.INTAKE_CYCLE)
                .transition(() -> currentPath != null && currentPath.isDone(), State.SHOOT_CYCLE, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildShootCyclePath());
                })
                .transition(() -> isTransferFull(), State.SHOOT_CYCLE, () -> {
                    shouldReadColorSensors = false;
                    stopIntakeAndPath();
                    startPath(buildShootCyclePath());
                })

                .state(State.SHOOT_CYCLE)
                // Cycle if time permits
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() < CYCLE_TIME_THRESHOLD, State.INTAKE_CYCLE, () -> {
                    startCycleIntakePath();
                })
                // Park if time is running out
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() >= CYCLE_TIME_THRESHOLD, State.PARK, () -> {
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
//        telemetry.addLine("Press A to set turret to Pre-Aim (116)");
    }

    @Override
    public void onStart() {
        matchTimer.reset();
        Globals.setAlliance(Alliance.RED);
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

    private void startCycleIntakePath() {
        boolean detectedPickupX = updateIntakeXPosition();
        if (detectedPickupX && pickupWallX >= CYCLE_HP_PATH_MIN_X) {
            startPath(buildIntakeHPPath());
        } else {
            startPath(buildIntakeCyclePath());
        }
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
                        new Point2d(62, 6),
                        new Point2d(62, 7)
                }, 50)
                .callback(()->{
                    new SequentialCommandGroup(
                            new LockOnGoalCommand()
                    ).schedule();
                })
                .waitMilliseconds(1700)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand()).schedule();
                })
                .waitMilliseconds(200)
                .build();
    }

    private Path buildIntakeSpikePath() {
        // for the far spike
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(62, 7),
                        new Point2d(46, 40),
                        // INTAKE FIRST SET
                        new Point2d(36, 49)
                }, 1700)
//                .waitMilliseconds(200)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(400, shootVeloLeft, shootVeloMiddle, shootVeloRight, hood);
                })
                .waitMilliseconds(0)
                .build();
    }

    private Path buildShootSpikePath() {
        // for the far spike
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(39, 39),
                        new Point2d(47, 24),
                        shootingPoint
                }, 2000)
                .waitMilliseconds(800)
                .callback(() -> {
                    new AutonomousShootCommand().schedule();
                })
                .waitMilliseconds(200)
                .build();
    }

    private Path buildIntakeHPPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(57, 40),
                        new Point2d(60, 54),
                        new Point2d(62, pickupWallY)
                }, 1600)
                .waitMilliseconds(0)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(400, shootVeloLeft, shootVeloMiddle, shootVeloRight, hood);
                })
                .waitMilliseconds(0)
                .build();
    }

    private Path buildShootHPPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(61, pickupWallY),
                        shootingPoint
                }, 3000, true)
                .addTurnTo(90, 250)
                .waitMilliseconds(600)
                .callback(() -> {
                    new AutonomousShootCommand().schedule();
                })
                .waitMilliseconds(200)
                .build();
    }

    private Path buildIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(pickupWallX+2, pickupWallY-9),
                        new Point2d(pickupWallX, pickupWallY+3)
                }, 1500)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(400, shootVeloLeft, shootVeloMiddle, shootVeloRight, hood);
                })
                .waitMilliseconds(0)
                .build();
    }

    private Path buildShootCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(pickupWallX, pickupWallY),
                        shootingPoint
                }, 3000, true)
                .addTurnTo(90, 250)
                .waitMilliseconds(600)
                .callback(() -> {
                    new AutonomousShootCommand().schedule();
                })
                .waitMilliseconds(200)
                .build();
    }

    private Path buildParkPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[]{
                        shootingPoint,
                        new Point2d(44,8)
                },100)
                .callback(()->{
                    new SequentialCommandGroup(
//                            new TurnTurretToPosCommand(-150),
//                            new WaitCommand(300),
//                            new CenterTurretCommand()
                    ).schedule();
                })
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(57, 49)
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
}
