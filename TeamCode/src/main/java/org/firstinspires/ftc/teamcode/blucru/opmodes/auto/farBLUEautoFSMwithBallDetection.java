package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.FarAutoTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous
public class farBLUEautoFSMwithBallDetection extends BluLinearOpMode {
    // Turret angle to be set while the robot is driving to shooting position
    double turretAnglePreaim = -107;

    // Turret angle to be set to once the bot reaches the shooting position
    double turretAngleFinal = 160; // Field centric angle increase = towards obelisk decrease = towards gate
    double shootVeloLeft = 1440;
    double shootVeloMiddle = 1440;
    double shootVeloRight = 1430;
    Point2d shootingPoint = new Point2d(45, -9);

    double hood = 49;

    double pickupWallY = -62;
    double pickupWallX = 62; // default for hp

    enum State {
        PRELOAD,
        INTAKE_SPIKE,
        SHOOT_SPIKE,
        INTAKE_HP,
        DRIVE_TO_SHOOT_HP,     // drives to shooting position (no turret/shoot)
        INTAKE_CYCLE,
        DRIVE_TO_SHOOT_CYCLE,  // drives to shooting position (no turret/shoot)
        VERIFY_TRANSFER,       // checks if transfer succeeded
        RETRY_TRANSFER,        // retries the transfer sequence
        AIM_AND_SHOOT,         // aims turret and shoots (runs after transfer verified)
        PARK,
        IDLE
    }

    StateMachine sm;
    Path currentPath;
    ElapsedTime matchTimer;
    ElapsedTime retryTimer;       // tracks retry sequence duration
    ElapsedTime aimShootTimer;    // tracks aim + shoot timing

    // Time threshold to start a new cycle (30s match - ~7s per cycle)
    final double CYCLE_TIME_THRESHOLD = 25.0;
    // How long the retry sequence takes (spit + center + elevator down + wait + elevator up + middle)
    final double RETRY_DURATION_SEC = 3.0;
    // Time from aim to shoot
    final double AIM_DURATION_SEC = 0.7;
    // Time after shoot to let balls leave
    final double SHOOT_WAIT_SEC = 0.4;

    boolean shouldReadColorSensors = false;

    public void initialize() {
        robot.clear();
        addSixWheel();
        addIntake();
        addElevator();
        addShooter();
        addTurret();
        addTransfer();
        addBallDetector();

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
        retryTimer = new ElapsedTime();
        aimShootTimer = new ElapsedTime();

        sm = new StateMachineBuilder()
                // ========================
                // PRELOAD: shoot preloaded balls
                // ========================
                .state(State.PRELOAD)
                .transition(() -> currentPath != null && currentPath.isDone(), State.INTAKE_SPIKE, () -> {
                    shouldReadColorSensors = true;
                    startPath(buildIntakeSpikePath());
                })

                // ========================
                // INTAKE_SPIKE: drive to spike and intake
                // ========================
                .state(State.INTAKE_SPIKE)
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

                // ========================
                // SHOOT_SPIKE: drive to shoot position and shoot (has its own inline transfer)
                // ========================
                .state(State.SHOOT_SPIKE)
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() < CYCLE_TIME_THRESHOLD, State.INTAKE_HP, () -> {
                            shouldReadColorSensors = true;
                            startPath(buildIntakeHPPath());
                        })
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() >= CYCLE_TIME_THRESHOLD, State.PARK, () -> {
                            startPath(buildParkPath());
                        })

                // ========================
                // INTAKE_HP: drive to HP wall and intake
                // FarAutoTransferCommand fires at the end of the path callback
                // ========================
                .state(State.INTAKE_HP)
                .transition(() -> currentPath != null && currentPath.isDone(), State.DRIVE_TO_SHOOT_HP, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildDriveToShootPath());
                })
                .transition(() -> isTransferFull(), State.DRIVE_TO_SHOOT_HP, () -> {
                    shouldReadColorSensors = false;
                    stopIntakeAndPath();
                    startPath(buildDriveToShootPath());
                })

                // ========================
                // DRIVE_TO_SHOOT_HP: just drives to the shooting position, no turret commands
                // When it arrives, go to VERIFY_TRANSFER to check if balls transferred
                // ========================
                .state(State.DRIVE_TO_SHOOT_HP)
                .transition(() -> currentPath != null && currentPath.isDone(), State.VERIFY_TRANSFER, () -> {
                    // Read color sensors to check transfer status
                    shouldReadColorSensors = true;
                })

                // ========================
                // INTAKE_CYCLE: drive to detected ball position and intake
                // FarAutoTransferCommand fires at the end of the path callback
                // ========================
                .state(State.INTAKE_CYCLE)
                .transition(() -> currentPath != null && currentPath.isDone(), State.DRIVE_TO_SHOOT_CYCLE, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildDriveToShootCyclePath());
                })
                .transition(() -> isTransferFull(), State.DRIVE_TO_SHOOT_CYCLE, () -> {
                    shouldReadColorSensors = false;
                    stopIntakeAndPath();
                    startPath(buildDriveToShootCyclePath());
                })

                // ========================
                // DRIVE_TO_SHOOT_CYCLE: just drives to shooting position, no turret commands
                // ========================
                .state(State.DRIVE_TO_SHOOT_CYCLE)
                .transition(() -> currentPath != null && currentPath.isDone(), State.VERIFY_TRANSFER, () -> {
                    shouldReadColorSensors = true;
                })

                // ========================
                // VERIFY_TRANSFER: check if the balls made it into the turret
                // This state runs for ONE frame — immediately transitions
                // ========================
                .state(State.VERIFY_TRANSFER)
                // Transfer SUCCEEDED (no balls in elevator = they made it to turret)
                .transition(() -> !hasBallsInElevator(), State.AIM_AND_SHOOT, () -> {
                    // Transfer worked! Aim turret and shoot
                    new SequentialCommandGroup(
                            new ParallelizeIntakeCommand(),
                            new TurnTurretToPosFieldCentricCommand(turretAngleFinal),
                            new WaitCommand(600),
                            new AutonomousShootCommand()
                    ).schedule();
                    aimShootTimer.reset();
                })
                // Transfer FAILED (balls still stuck in elevator)
                .transition(() -> hasBallsInElevator(), State.RETRY_TRANSFER, () -> {
                    // Retry: spit extra balls, center turret, drop elevator, try again
                    new SequentialCommandGroup(
                            new IntakeSpitCommand(),
                            new CenterTurretCommand(),
                            new ElevatorDownCommand(),
                            new AllTransferDownCommand(),
                            new WaitCommand(1000),
                            new ParallelizeIntakeCommand(),
                            new ElevatorUpCommand(),
                            new WaitCommand(300),
                            new ElevatorMiddleCommand(),
                            new WaitCommand(150),
                            new AllTransferMiddleCommand(),
                            new SetHoodAngleCommand(hood)
                    ).schedule();
                    retryTimer.reset();
                })

                // ========================
                // RETRY_TRANSFER: wait for retry sequence to complete, then aim & shoot
                // ========================
                .state(State.RETRY_TRANSFER)
                .transition(() -> retryTimer.seconds() > RETRY_DURATION_SEC, State.AIM_AND_SHOOT, () -> {
                    // Retry done — now aim turret and shoot
                    new SequentialCommandGroup(
                            new ParallelizeIntakeCommand(),
                            new TurnTurretToPosFieldCentricCommand(turretAngleFinal),
                            new WaitCommand(600),
                            new AutonomousShootCommand()
                    ).schedule();
                    aimShootTimer.reset();
                })

                // ========================
                // AIM_AND_SHOOT: aim turret, shoot, then immediately start next path
                // The turret aim + shoot commands execute while the bot drives away
                // ========================
                .state(State.AIM_AND_SHOOT)
                .transition(() -> aimShootTimer.seconds() > 0.9
                        && matchTimer.seconds() < CYCLE_TIME_THRESHOLD, State.INTAKE_CYCLE, () -> {
                    shouldReadColorSensors = true;
                    updateIntakeXPosition();
                    startPath(buildIntakeCyclePath());
                })
                .transition(() -> aimShootTimer.seconds() > 0.9
                        && matchTimer.seconds() >= CYCLE_TIME_THRESHOLD, State.PARK, () -> {
                    startPath(buildParkPath());
                })

                // ========================
                // PARK
                // ========================
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
        telemetry.addLine("--- INIT ---");
        telemetry.addLine("Press A to set turret to Pre-Aim (-116)");
    }

    public void onStart() {
        matchTimer.reset();
        shooter.shootWithVelocityIndependent(1490, 1520, 1490);
        sixWheel.setPosition(new Pose2d(63, -7, Math.toRadians(-90)));
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
        telemetry.addData("State", sm.getState());
        telemetry.addData("Time", matchTimer.seconds());
        telemetry.addData("LL Clump X", ballDetector.getClumpFieldX());
        telemetry.addData("Intake X", pickupWallX);
    }

    // ===========================
    // HELPER METHODS
    // ===========================

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

    private void updateIntakeXPosition() {
        if (ballDetector.hasValidClump()) {
            double fieldX = ballDetector.getClumpFieldX();
            double minX = 20;
            double maxX = 62;
            pickupWallX = Range.clip(fieldX, minX, maxX);
        }
    }

    private boolean isTransferFull() {
        if (!shouldReadColorSensors)
            return false;

        elevator.updateLeftBallColor();
        elevator.updateMiddleBallColor();
        elevator.updateRightBallColor();

        return ShooterMotifCoordinator.getLeftColor() != BallColor.UNKNOWN &&
                ShooterMotifCoordinator.getMiddleColor() != BallColor.UNKNOWN &&
                ShooterMotifCoordinator.getRightColor() != BallColor.UNKNOWN;
    }

    /**
     * Checks if there are balls still stuck in the elevator (transfer failed).
     * Returns true if ANY color sensor detects a ball.
     */
    private boolean hasBallsInElevator() {
        elevator.updateLeftBallColor();
        elevator.updateMiddleBallColor();
        elevator.updateRightBallColor();

        return ShooterMotifCoordinator.getLeftColor() != BallColor.UNKNOWN ||
                ShooterMotifCoordinator.getMiddleColor() != BallColor.UNKNOWN ||
                ShooterMotifCoordinator.getRightColor() != BallColor.UNKNOWN;
    }

    // ===========================
    // PATH BUILDERS
    // ===========================

    private Path buildPreloadPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(63, -7),
                        new Point2d(63, -8)
                }, 100)
                .waitMilliseconds(1500)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand()).schedule();
                })
                .waitMilliseconds(1000)
                .build();
    }

    private Path buildIntakeSpikePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(63, -8),
                        new Point2d(37, -50)
                }, 2000)
                .waitMilliseconds(200)
                .callback(() -> {
                    new SequentialCommandGroup(
                        new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                        new IntakeSpitCommand(),
                        new WaitCommand(200),
                        new ElevatorUpCommand(),
                        new WaitCommand(200),
                        new ElevatorMiddleCommand(),
                        new WaitCommand(150),
                        new AllTransferMiddleCommand(),
                        new SetHoodAngleCommand(hood),
                        new IntakeStopCommand(),
                        new ParallelizeIntakeCommand(),
                        new WaitCommand(200),
                        new TurnTurretToPosCommand(-95)
                    ).schedule();
                })
                .waitMilliseconds(0)
                .build();
    }

    private Path buildShootSpikePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(40, -40),
                        new Point2d(45, -25),
                        shootingPoint
                }, 2000)
                .waitMilliseconds(300)
                .callback(() -> {
                    new TurnTurretToPosFieldCentricCommand(turretAngleFinal).schedule();
                })
                .waitMilliseconds(400)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand()).schedule();
                })
                .waitMilliseconds(300)
                .build();
    }

    private Path buildIntakeHPPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(61, -45),
                        new Point2d(62, -55),
                        new Point2d(62, pickupWallY)
                }, 1200)
                .waitMilliseconds(300)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                            new FarAutoTransferCommand(hood, turretAnglePreaim)).schedule();
                })
                .waitMilliseconds(0)
                .build();
    }

    /** Drive-only path to shooting position (used after HP intake). No turret/shoot callbacks. */
    private Path buildDriveToShootPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(62, pickupWallY),
                        shootingPoint
                }, 3000)
                .addTurnTo(-90, 500)
                .build();
    }

    private Path buildIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(pickupWallX, pickupWallY - 2)
                }, 1200)
                .waitMilliseconds(1000)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                            new FarAutoTransferCommand(hood, turretAnglePreaim)).schedule();
                })
                .waitMilliseconds(0)
                .build();
    }

    /** Drive-only path to shooting position (used after cycle intake). No turret/shoot callbacks. */
    private Path buildDriveToShootCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(pickupWallX, pickupWallY),
                        shootingPoint
                }, 3000)
                .addTurnTo(-90, 500)
                .build();
    }

    private Path buildParkPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(58, -50)
                }, 5000)
                .build();
    }
}
