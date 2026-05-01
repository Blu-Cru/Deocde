package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootFlipTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferThenLockOnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.WaitForTurretNearTargetCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.TurnOffShooterCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretTo180DegreeTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;



public class CloseRedAuto extends BaseAuto {
    double velo = 1170;
    double veloMiddle = 1230;
    double hood = 43;
    double GATE_CYCLE_TIME_THRESHOLD = 21;
    private Point2d shootingPose = new Point2d(-4, 12);

    enum State {
        PRELOAD, MIDDLE_SPIKE, FIRST_GATE_CYCLE, GATE_CYCLE, CLOSE_SPIKE,
        FAR_SPIKE, PARK, IDLE
    }

    Path currentPath;

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-51, 55, -0.926);
    }

    @Override
    public StateMachine buildStateMachine() {
        return new StateMachineBuilder()
                .state(State.PRELOAD)
                .transition(() -> currentPath != null && currentPath.isDone(), State.MIDDLE_SPIKE,
                        () -> {
                            startPath(buildSpikeMiddlePath());
                        })

                .state(State.MIDDLE_SPIKE)
                // Transition if path completes normally
                .transition(() -> currentPath != null && currentPath.isDone(), State.FIRST_GATE_CYCLE, () -> {
                    startPath(buildFirstIntakeCyclePath());
                })

                .state(State.FIRST_GATE_CYCLE)
                .transition(() -> currentPath != null && currentPath.isDone()
                                && Globals.matchTime.seconds() < GATE_CYCLE_TIME_THRESHOLD,
                        State.GATE_CYCLE, () -> {
                            startPath(buildRestIntakeCyclePath());
                        })
                // Park if time is running out
                .transition(() -> currentPath != null && currentPath.isDone()
                                && Globals.matchTime.seconds() >= GATE_CYCLE_TIME_THRESHOLD,
                        State.CLOSE_SPIKE, () -> {
                            startPath(buildSpikeClosePath());
                        })
                .state(State.GATE_CYCLE)
                .transition(() -> currentPath != null && currentPath.isDone()
                        && Globals.matchTime.seconds() >= GATE_CYCLE_TIME_THRESHOLD, State.CLOSE_SPIKE, () -> {
                    startPath(buildSpikeClosePath());
                })
                .transition(() -> currentPath != null && currentPath.isDone()
                        && Globals.matchTime.seconds() < GATE_CYCLE_TIME_THRESHOLD, State.GATE_CYCLE, () -> {
                    startPath(buildRestIntakeCyclePath());
                })
                .state(State.CLOSE_SPIKE)
                .transition(() -> currentPath != null && currentPath.isDone(), State.PARK, () -> {
                    startPath(buildParkPath());
                })

                .state(State.PARK)
                // Auto Done, switch to IDLE
                .transition(() -> currentPath != null && currentPath.isDone(), State.IDLE, () -> {
                })

                .state(State.IDLE)
                .onEnter(this::stopIntakeShooterAndPath)

                .build();
    }

    @Override
    public void initialize() {
        Globals.setAlliance(Alliance.RED);
        addAutoSubsystems(false);
        shooter.setHoodAngle(26);
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
        intake.write();

        super.initialize();
    }

    public void onStart() {
        Globals.matchTime.reset();
        Globals.setAlliance(Alliance.RED);
        shooter.shootWithVelocityIndependent(925,950,950);
        turret.setAngle(2);
        sixWheel.setPosition(startPose);
        currentPath = buildPreloadPath();
        startPath(currentPath);
        sm.setState(State.PRELOAD);
        sm.start();
    }

    @Override
    public void periodic() {
        if (currentPath != null) {
            currentPath.run();
        }
        if (sm != null) {
            sm.update();
        }
    }

    private void startPath(Path path) {
        currentPath = path;
        currentPath.start();
    }

    private void stopIntakeShooterAndPath() {
        if (currentPath != null) {
            currentPath.endSixWheel();
        }
        new IntakeStopCommand().schedule();
        new TurnOffShooterCommand().schedule();
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
                new WaitCommand(delayBeforeTransferMs),
                new AutonomousTransferThenLockOnCommand(),
                new AutoAimCommand()
        ).schedule();
    }

    /**
     *
     * This path shoots the preloads
     *
     */
    private Path buildPreloadPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-51, 55),
                        new Point2d(-39.35, 41.68)
                }, 1000)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AllTransferUpCommand(),
                            new WaitCommand(200),
                            // Run shooter idle and turret flip simultaneously — removes 2 scheduler
                            // ticks of delay before the turret starts turning.
                            new ParallelCommandGroup(
                                    new IdleShooterCommand(),
                                    new TurnTurretToPosCommand(-90),
                                    new ElevatorDownCommand(),
                                    new AllTransferDownCommand()
                            ),
                            new WaitCommand(200),
                            new CenterTurretCommand(),
                            new WaitForTurretNearTargetCommand(),
                            new IntakeStartCommand()).schedule();
                })
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 200)
                .build();
    }

    private Path buildSpikeMiddlePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        //purposely off
                        new Point2d(-39.35, 41.68),
                        // small guide point for the turn
                        new Point2d(-27.01, 35.35),
                        new Point2d(-14.65, 30.02),
                        new Point2d(3.07, 19.55),
                        new Point2d(10.21,24.36),
                }, 2000)
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(10.21,24.36),
                        new Point2d(12.42, 32.30),
                        new Point2d(12.76, 45.30),
                        new Point2d(12, 56.43)
                },2000)
//                        .waitMilliseconds(500)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(100, velo, veloMiddle, velo, hood);
                })
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(12, 56.29),
                        new Point2d(15.92, 51.22),
                        new Point2d(12.34,29.30),
                        shootingPose
                }, 1500, true)
                .waitMilliseconds(500)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand()
                    ).schedule();
                })
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 200)
                .build();
    }

    private Path buildFirstIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose,
                        new Point2d(14.74-1, 44.24)
                }, 1000)
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(14.74-1, 44.24),
                        new Point2d(13.87-1, 49.27),
                        new Point2d(11.22-1, 62.34)}, 700)
                .waitUntil(() -> elevator.isFull(),1500)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(300, velo, veloMiddle, velo, hood);
                })
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(11.22-1, 62.37),
                        new Point2d(15.92, 51.22),
                        new Point2d(4,29.30),
                        shootingPose
                }, 2000, true)
                .waitMilliseconds(500)
                .callback(() -> new AutonomousShootCommand().schedule())
                .waitMilliseconds(200)
                .build();
    }

    private Path buildRestIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose,
                        new Point2d(14.74-1, 44.24)
                }, 1000)
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(14.74-1, 44.24),
                        new Point2d(13.87-1, 49.27),
                        new Point2d(11.22-1, 62.34)}, 700)
                .waitUntil(() -> elevator.isFull(),1500)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(300, velo, veloMiddle, velo, hood);
                })

//                .addTurnTo(-90,500)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(11.22-1, 62.37),
                        new Point2d(15.92, 51.22),
                        new Point2d(4,29.30),
                        shootingPose
                }, 2000, true)
                .waitMilliseconds(400)
                .callback(() -> new SequentialCommandGroup(
                        new AllTransferUpCommand(),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new IdleShooterCommand(),
                                new CenterTurretCommand(),
                                new ElevatorDownCommand(),
                                new AllTransferDownCommand()
                        ),
                        new WaitForTurretNearTargetCommand(),
                        new IntakeStartCommand()
                ).schedule())
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 200)
                .build();
    }


    /**
     *
     * This path grabs the close spikes
     *
     */
    private Path buildSpikeClosePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose,
                        new Point2d(-5.55, 33.78),
                        new Point2d(-9.91, 59.91)
                }, 1300)
                .waitMilliseconds(400)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(400);
                })

                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-9.91, 59.91),
//                        new Point2d(-6, -19),
                        new Point2d(-16.05,20),
                }, 2000)
//                                .addTurnTo(-30, 1000)
                .waitMilliseconds(700)
                .callback(() -> new AutonomousShootCommand().schedule())
                .waitMilliseconds(200)
                .build();
    }

    /**
     *
     * This path grabs and shoots the far spikes
     *
     */
    private Path buildSpikeFarPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-16.13, 12.06),
                        new Point2d(32.76, 44.76),
                        new Point2d(39.91, 50.58),

                }, 2000)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(400, velo - 40, veloMiddle, velo - 20, null);
                })
//                                .waitMilliseconds(100)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(32.81, 46.76),
                        new Point2d(-4.11, 12.74)
                }, 2000)
                .waitUntil(()-> Robot.getInstance().turret.atTarget(), 500)
                .callback(() -> new AutonomousShootCommand().schedule())
                .waitMilliseconds(200)
                .build();
    }

    private Path buildParkPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-16.05,20),
                        new Point2d(-16, 60)
                }, 5000)
                .build();
    }
}
