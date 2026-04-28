package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootFlipTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootFlipTurretSweepCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferThenLockOnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FarAutoTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalSweepCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class

FarBlueAutoSweep extends BaseAuto {
    double turretAnglePreaim = -116;

    double shootVeloLeft = 1420;
    double shootVeloMiddle = 1450;
    double shootVeloRight = 1430;
    Point2d shootingPoint = new Point2d(48, -9);

    double hood = 50;

    double pickupWallY = -62;
    double pickupWallX = 61;
    private static final double CYCLE_HP_PATH_MIN_X = 54.0;

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
    final double CYCLE_TIME_THRESHOLD = 26.0;
    boolean shouldReadColorSensors = false;

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(63, -7, Math.toRadians(-90));
    }

    @Override
    public void initialize() {
        robot.clear();
        addSixWheel();
        addIntake();
        addElevator();
        addShooter();
        addTurret();
        robot.addTurretCam();
        addTransfer();
        addBallDetector();

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
                .transition(() -> currentPath != null && currentPath.isDone(), State.INTAKE_SPIKE, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildIntakeSpikePath());
                })

                .state(State.INTAKE_SPIKE)
                .transition(() -> currentPath != null && currentPath.isDone(), State.SHOOT_SPIKE, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildShootSpikePath());
                })
                .transition(() -> isTransferFull(), State.SHOOT_SPIKE, () -> {
                    shouldReadColorSensors = false;
                    stopIntakeAndPath();
                    startPath(buildShootSpikePath());
                })

                .state(State.SHOOT_SPIKE)
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() < CYCLE_TIME_THRESHOLD, State.INTAKE_HP, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildIntakeHPPath());
                })
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() >= CYCLE_TIME_THRESHOLD, State.PARK, () -> {
                    startPath(buildParkPath());
                })

                .state(State.INTAKE_HP)
                .transition(() -> currentPath != null && currentPath.isDone(), State.SHOOT_HP, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildShootHPPath());
                })
                .transition(() -> isTransferFull(), State.SHOOT_SPIKE, () -> {
                    shouldReadColorSensors = false;
                    stopIntakeAndPath();
                    startPath(buildShootHPPath());
                })

                .state(State.SHOOT_HP)
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() < CYCLE_TIME_THRESHOLD, State.INTAKE_CYCLE, () -> {
                    shouldReadColorSensors = false;
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
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() < CYCLE_TIME_THRESHOLD, State.INTAKE_CYCLE, this::startCycleIntakePath)
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
        turret.read();
        if (driver1.pressedA()) {
            turret.setAngle(turretAnglePreaim);
       }
       turret.write();
      telemetry.addLine("Press A to set turret to Pre-Aim (-116)");
    }

    @Override
    public void onStart() {
        matchTimer.reset();
        shooter.shootWithVelocityIndependent(1460, 1520, 1500);
        sixWheel.setPosition(startPose);
        Globals.setAlliance(Alliance.BLUE);

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
        new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight).schedule();
        new FarAutoTransferCommand(hood, turretAnglePreaim).schedule();
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
            double minX = 24;
            double maxX = 62;
            pickupWallX = Range.clip(fieldX, minX, maxX);
            return true;
        }

        return false;
    }

    private boolean isTransferFull() {
        if (!shouldReadColorSensors) {
            return false;
        }
        return elevator.isFull();
    }

    private Path buildPreloadPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(63, -7),
                        new Point2d(63, -8)
                }, 50)
                .callback(()->{
                    new TurnTurretToPosCommand(turretAnglePreaim).schedule();
                })
                .waitMilliseconds(1700)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootFlipTurretCommand()).schedule();
                })
                .waitMilliseconds(200)
                .build();
    }

    private Path buildIntakeSpikePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(63, -8),
                        new Point2d(47, -41),
                        new Point2d(37, -50)
                }, 1700)
                .waitMilliseconds(200)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(500,shootVeloLeft,shootVeloMiddle,shootVeloRight, hood);
                })
                .waitMilliseconds(0)
                .build();
    }

    private Path buildShootSpikePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(40, -40),
                        new Point2d(48, -25),
                        shootingPoint
                }, 2000)
                .waitMilliseconds(200)
                .callback(() -> {
                    new AutonomousShootFlipTurretSweepCommand().schedule();
                })
                .waitMilliseconds(2000)
                .build();
    }

    private Path buildIntakeHPPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(55, -45),
                        new Point2d(58, -55),
                        new Point2d(60, pickupWallY - 1)
                }, 1600)
                .waitMilliseconds(0)

                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(800,shootVeloLeft,shootVeloMiddle,shootVeloRight, hood);
                })
                .waitMilliseconds(0)
                .build();
    }

    private Path buildShootHPPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(62, pickupWallY),
                        shootingPoint
                }, 3000, true)
                .addTurnTo(-80, 500)
                .waitMilliseconds(200)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootFlipTurretSweepCommand()).schedule();
                })
                .waitMilliseconds(2000)
                .build();
    }

    private Path buildIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(pickupWallX, pickupWallY + 9),
                        new Point2d(pickupWallX, pickupWallY - 3)
                }, 1500)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(600,shootVeloLeft,shootVeloMiddle,shootVeloRight, hood);
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
                .addTurnTo(-80, 500)
                .waitMilliseconds(200)
                .callback(() -> {
                    new AutonomousShootFlipTurretSweepCommand().schedule();
                })
                .waitMilliseconds(2000)
                .build();
    }

    private Path buildParkPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(45, -9)
                }, 100)
                .callback(() -> {
                    new CenterTurretCommand().schedule();
                })
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(58, -50)
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
                new AutonomousTransferThenLockOnCommand(hoodAngle, true)
        ).schedule();
    }
}
