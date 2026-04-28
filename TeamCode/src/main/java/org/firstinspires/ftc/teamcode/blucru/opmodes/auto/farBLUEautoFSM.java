package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutoLongSpitTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@Autonomous
public class farBLUEautoFSM extends BluLinearOpMode {

    double turretAngleRest = 156; // Field centric angle increase = towards obelisk decrease = towards gate
    double shootVeloLeft = 1440;
    double shootVeloMiddle = 1440;
    double shootVeloRight = 1430;
    Point2d shootingPoint = new Point2d(45, -9);

    double leftHood = 49;
    double middleHood = 45;
    double rightHood = 49;

    double pickupWallY = -62;

    enum State {
        PRELOAD,
        INTAKE_1,
        SHOOT_1,
        INTAKE_CYCLE,
        SHOOT_CYCLE,
        PARK,
        IDLE
    }

    StateMachine sm;
    Path currentPath;
    ElapsedTime matchTimer;
    // Time threshold to start a new cycle (30s match - ~7s per cycle)
    final double CYCLE_TIME_THRESHOLD = 23.0;
    boolean shouldReadColorSensors = false;

    public void initialize() {
        robot.clear();
        addSixWheel();
        addIntake();
        addElevator();
        addShooter();
        addTurret();
        addTransfer();
        addLLTagDetector();
        shooter.setHoodAngleIndependent(leftHood, middleHood, rightHood);
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

        sm = new StateMachineBuilder()
                .state(State.PRELOAD)
                .transition(() -> currentPath != null && currentPath.isDone(), State.INTAKE_1, () -> {
                    shouldReadColorSensors = true;
                    startPath(buildIntake1Path());
                })

                .state(State.INTAKE_1)
                // Transition if path completes normally
                .transition(() -> currentPath != null && currentPath.isDone(), State.SHOOT_1, () -> {
                    shouldReadColorSensors = false;
                    startPath(buildShoot1Path());
                })
                // Transition early if transfer is full
                .transition(() -> isTransferFull(), State.SHOOT_1, () -> {
                    shouldReadColorSensors = false;
                    stopIntakeAndPath();
                    startPath(buildShoot1Path());
                })

                .state(State.SHOOT_1)
                .transition(() -> currentPath != null && currentPath.isDone()
                        && matchTimer.seconds() < CYCLE_TIME_THRESHOLD, State.INTAKE_CYCLE, () -> {
                            shouldReadColorSensors = true;
                            startPath(buildIntakeCyclePath());
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
                            startPath(buildIntakeCyclePath());
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

    public void onStart() {
        matchTimer.reset();
        shooter.shootWithVelocityIndependent(1510, 1520, 1490);
        turret.setAngle(-116);
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
    }

    private void startPath(Path path) {
        currentPath = path;
        currentPath.start();
    }

    private void stopIntakeAndPath() {
        if (currentPath != null) {
            currentPath.endSixWheel();
        }
        new AutonomousTransferCommand(leftHood, middleHood, rightHood).schedule();
    }

    private boolean isTransferFull() {
        // Only read color sensors during intake states to avoid I2C overhead
        if (!shouldReadColorSensors)
            return false;

        elevator.updateLeftBallColor();
        elevator.updateMiddleBallColor();
        elevator.updateRightBallColor();

        return ShooterMotifCoordinator.getLeftColor() != BallColor.UNKNOWN &&
                ShooterMotifCoordinator.getMiddleColor() != BallColor.UNKNOWN &&
                ShooterMotifCoordinator.getRightColor() != BallColor.UNKNOWN;
    }

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
                .waitMilliseconds(300)
                .build();
    }

    private Path buildIntake1Path() {
        // for the far spike
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(63, -8),
                        // INTAKE FIRST SET
                        new Point2d(37, -50)
                }, 2000)
                .waitMilliseconds(200)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                            new IntakeStopCommand(),
                            new WaitCommand(100),
                            new IntakeSpitCommand(),
                            new WaitCommand(300),
                            new ElevatorUpCommand(),
                            new WaitCommand(300),
                            new ElevatorMiddleCommand(),
                            new WaitCommand(150),
                            new AllTransferMiddleCommand(),
                            new SetLeftHoodAngleCommand(leftHood),
                            new SetRightHoodAngleCommand(middleHood),
                            new SetMiddleHoodAngleCommand(rightHood),
                            // new WaitCommand(200), //TODO: TUNE WAIT
                            new IntakeStopCommand(),
                            new ParallelizeIntakeCommand()
                    // new WaitCommand(2000),
                    // new TurnTurretToPosCommand(102)
                    ).schedule();
                })
                .waitMilliseconds(1500)
                .build();
    }

    private Path buildShoot1Path() {
        // for the far spike
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(40, -40),
                        new Point2d(45, -25),
                        shootingPoint
                }, 2000)
                .waitMilliseconds(300)
                .callback(() -> {
                    new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                })
                .waitMilliseconds(1000)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand()).schedule();
                })
                .waitMilliseconds(300)
                .build();
    }

    private Path buildIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPoint,
                        new Point2d(61, -45),
                        new Point2d(62, -55),
                        new Point2d(62, pickupWallY)
                }, 1200)
                .waitMilliseconds(1000)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                            new AutonomousTransferCommand(leftHood, middleHood, rightHood)).schedule();
                })
                .waitMilliseconds(1500)
                .build();
    }

    private Path buildShootCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(62, pickupWallY),
                        shootingPoint
                }, 3000)
                .waitMilliseconds(500)
                .callback(() -> {
                    new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                })
                .waitMilliseconds(1100)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand()).schedule();
                })
                .waitMilliseconds(300)
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
