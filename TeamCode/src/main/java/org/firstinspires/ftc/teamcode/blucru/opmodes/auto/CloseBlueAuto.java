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



public class CloseBlueAuto extends BaseAuto {
    double turretAngle = 142;
    double preAimTurretAngle = -120;
    double gateCyclePreAimAngle = -120;
    double velo = 1170;
    double veloMiddle = 1230;
    double hood = 43;
    double GATE_CYCLE_TIME_THRESHOLD = 21;
    private Point2d shootingPose = new Point2d(-4, -12);

    enum State {
        PRELOAD, MIDDLE_SPIKE, FIRST_GATE_CYCLE, GATE_CYCLE, CLOSE_SPIKE,
        FAR_SPIKE, PARK, IDLE
    }

    Path currentPath;

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-53.66, -51.5, 0.869);
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
        shooter.shootWithVelocityIndependent(925,950,925);
        turret.setAngle(2);
        sixWheel.setPosition(startPose);
        currentPath = buildPreloadPath();
        startPath(currentPath);
        Globals.setAlliance(Alliance.BLUE);
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
                new AutoAimCommand(),
                new WaitCommand(delayBeforeTransferMs),
                new AutonomousTransferThenLockOnCommand()
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
                        new Point2d(-53.66, -51.5),
                        new Point2d(-41.27, -38.87)
                }, 1000)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootFlipTurretCommand()).schedule();
                })
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 200)
                .build();
    }

    private Path buildSpikeMiddlePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        //purposely off
                        new Point2d(-41.27, -38.87),
                        // small guide point for the turn
                        new Point2d(-28.59, -33.25),
                        new Point2d(-15.95, -28.63),
                        new Point2d(2.35, -19.18),
                        new Point2d(9.19,-24.39),
                }, 2000)
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(9.19,-24.39),
                        new Point2d(10.95, -32.45),
                        new Point2d(10.55, -45.45),
                        new Point2d(10, -56.29)
                },2000)
//                        .waitMilliseconds(500)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(200, velo, veloMiddle, velo, hood);
                })
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(10.28, -54.44),
                        new Point2d(13.37, -51.53),
                        new Point2d(11.04,-29.45),
                        shootingPose
                }, 1500, true)
                .waitMilliseconds(500)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootFlipTurretCommand()
                    ).schedule();
                })
                .waitMilliseconds(300)
//                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 500)
                .build();
    }

    private Path buildFirstIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose,
                        new Point2d(12.58, -44.51)
                }, 1000)
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(12.58, -44.51),
                        new Point2d(11.43, -49.47),
                        new Point2d(8.13, -59.38)}, 700)
                .waitUntil(() -> elevator.isFull(),1500)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(400, velo, veloMiddle, velo, hood);
                })
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(7.13, -59.35),
                        new Point2d(13.37, -51.53),
                        new Point2d(4,-29.45),
                        shootingPose
                }, 2000, true)
                .waitMilliseconds(400)
                .callback(() -> new AutonomousShootFlipTurretCommand().schedule())
                .waitMilliseconds(300)
                .build();
    }

    private Path buildRestIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose,
                        new Point2d(12.58, -44.51)
                }, 1000)
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(12.58, -44.51),
                        new Point2d(11.43, -49.47),
                        new Point2d(8.13, -59.38)}, 700)
                .waitUntil(() -> elevator.isFull(),1500)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(400, velo, veloMiddle, velo, hood);
                })

//                .addTurnTo(-90,500)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(7.13, -59.35),
                        new Point2d(13.37, -51.53),
                        new Point2d(4,-29.45),
                        shootingPose
                }, 2000, true)
                .waitMilliseconds(200)
                .callback(() -> new SequentialCommandGroup(
                        new AllTransferUpCommand(),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new IdleShooterCommand(),
                                new MoveTurretTo180DegreeTransferCommand(),
                                new ElevatorDownCommand(),
                                new AllTransferDownCommand()
                        ),
                        new WaitForTurretNearTargetCommand(),
                        new IntakeStartCommand()
                ).schedule())
                .waitMilliseconds(300)
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
                        new Point2d(-7.07, -32.91),
                        new Point2d(-12.80, -56.74)
                }, 1300)
                .waitMilliseconds(400)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(400);
                })

                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-16.71, -53.62),
//                        new Point2d(-6, -19),
                        new Point2d(-16.49,-13.61)
                }, 2000)
//                                .addTurnTo(-30, 1000)
                .waitMilliseconds(400)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AllTransferUpCommand(),
                            new WaitCommand(200),
                            new IdleShooterCommand(),
                            new TurnTurretToPosCommand(-90),
                            new WaitCommand(400),
                            new CenterTurretCommand(),
                            //new WaitCommand(200),
                            new ElevatorDownCommand(),
                            new AllTransferDownCommand()
                    ).schedule();
                })
                .waitMilliseconds(300)
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
                        new Point2d(-16.40, -10.61),
                        new Point2d(30.54, -46.05),
                        new Point2d(37.36, -52.26),

                }, 2000)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(400, velo - 40, veloMiddle, velo - 20, null);
                })
//                                .waitMilliseconds(100)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(30.48, -48.05),
                        new Point2d(-4.44, -11.98)
                }, 2000)
                .waitUntil(()-> Robot.getInstance().turret.atTarget(), 500)
                .callback(() -> new AutonomousShootCommand().schedule())
                .waitMilliseconds(200)
                .build();
    }

    private Path buildParkPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-16.49,-13.61),
                        new Point2d(-16.49, -45)
                }, 5000)
                .build();
    }
}
