package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelDrivetrainSetSimulatedBatteryVoltageCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.TurnOffShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

public class RootNegativeOneFSM extends BaseAuto {
    double turretAngle = 142;
    double preAimTurretAngle = -120;
    double gateCyclePreAimAngle = -120;
    double velo = 1230;
    double veloMiddle = 1230;
    double hood = 40;
    double intakeCycleSimulatedVoltage = 12.5;
    double GATE_CYCLE_TIME_THRESHOLD = 23;
    private Point2d shootingPose = new Point2d(5, -7);

    enum State {
        PRELOAD, MIDDLE_SPIKE, FIRST_GATE_CYCLE, GATE_CYCLE, LAST_GATE_CYCLE_PICKUP, LAST_GATE_CYCLE_SHOOT, CLOSE_SPIKE,
        FAR_SPIKE, PARK, IDLE
    }

    Path currentPath;

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-51, -54, Math.toRadians(51.529));
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
                .state(State.LAST_GATE_CYCLE_PICKUP)
                // Shoot if picked up
                .transition(() -> currentPath != null && currentPath.isDone(),
                        State.LAST_GATE_CYCLE_SHOOT, () -> {
                            startPath(buildShootMotifCyclePath());
                        })

                .state(State.LAST_GATE_CYCLE_SHOOT)
                // Get close spike once shot
                .transition(() -> currentPath != null && currentPath.isDone(), State.CLOSE_SPIKE,
                        () -> {
                            startPath(buildSpikeClosePath());
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
        robot.clear();
        addSixWheel();
        addIntake();
        addElevator();
        addShooter();
        addTurret();
        addTransfer();
        shooter.setHoodAngle(34);
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
        shooter.shootWithVelocityIndependent(1000,1050,1000);
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
            telemetry.addData("State", sm.getState());
        }
        telemetry.addData("Time", Globals.matchTime.seconds());
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

    /**
     *
     * This path shoots the preloads
     *
     */
    private Path buildPreloadPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-51, -54),
                        new Point2d(-30, -32)
                }, 1000)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand(false)).schedule();
                })
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 200)
                .build();
    }

    /**
     *
     * This path is to get the middle spike without opening the gate and then shoot
     * them
     *
     */
    private Path buildSpikeMiddlePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-30, -32),
                        // small guide point for the turn
                        new Point2d(-10, -30),
                        new Point2d(7, -37),
                        new Point2d(10, -50),
                        new Point2d(13, -65),
                }, 2300)
//                        .waitMilliseconds(500)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new SetShooterVelocityIndependentCommand(velo, veloMiddle,velo),
                            new AutonomousTransferCommand(hood),
                            new WaitCommand(700),
//                                                        new TurnTurretToPosCommand(-100),
//                                                        new WaitCommand(400),
                            new LockOnGoalCommand()
                    ).schedule();
                })
//                                .addTurnTo(-80, 300)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(13, -55),
//                                        new Point2d(5, -40),
                        shootingPose
                }, 1500, true)
                .waitMilliseconds(500)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand(false)
                    ).schedule();
                })
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 500)
                .build();
    }

    private Path buildFirstIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose,
                        new Point2d(16, -42),
                        new Point2d(14.5, -50),
                        new Point2d(10, -60)
                }, 2000)
                .waitUntil(() -> elevator.isFull(),1800)
                .callback(() -> {
                    new SequentialCommandGroup(
//                                                new WaitCommand(400),
                            new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                            new AutonomousTransferCommand(hood),
                            new WaitCommand(700),
                            new TurnTurretToPosCommand(-110),
                            new WaitCommand(400),
                            new LockOnGoalCommand()
                    ).schedule();
                })
//                                .addTurnTo(-90,500)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(10, -60),
                        new Point2d(16, -52),
                        shootingPose
                }, 2000, true)
                .waitMilliseconds(400)
                .callback(() -> new AutonomousShootCommand(false).schedule())
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 300)
                .build();
    }

    private Path buildRestIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose,
                        new Point2d(16, -42),
                        new Point2d(14.5, -50),
                        new Point2d(10, -60)
                }, 2000)
                .waitUntil(() -> elevator.isFull(),1800)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new WaitCommand(400),
                            new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                            new AutonomousTransferCommand(hood),
                            new WaitCommand(700),
                            new TurnTurretToPosCommand(-110),
                            new WaitCommand(400),
                            new LockOnGoalCommand()
                    ).schedule();
                })
//                .addTurnTo(-90,500)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(10, -60),
                        new Point2d(16, -52),
                        shootingPose
                }, 2000, true)
                .waitMilliseconds(400)
                .callback(() -> new AutonomousShootCommand(false).schedule())
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 300)
                .build();
    }


    private Path buildShootMotifCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(9, -56),
                        new Point2d(10, -44),
                        new Point2d(-12, -15)
                }, 2000)
                .waitMilliseconds(200)
                .callback(() -> new AutonomousShootCommand().schedule())
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 400)
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
                        new Point2d(-5, -34),
                        new Point2d(-10, -58)
                }, 2000)
//                                .waitMilliseconds(200)
                .callback(() -> {
                    new SequentialCommandGroup(
                            //new ReadBallColorsCommand(),
                            //new WaitCommand(100),
                            new WaitCommand(400),
                            new SetShooterVelocityIndependentCommand(velo-120, veloMiddle-120, velo-120),
                            new AutonomousTransferCommand(hood),
                            new WaitCommand(700),
//                                                        new WaitCommand(400),//wait for the addturnto
                            new LockOnGoalCommand()).schedule();
                })
//                                .waitMilliseconds(100)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-14, -55),
                        new Point2d(-6, -19),
                        new Point2d(-15,-12)
                }, 2000)
//                                .addTurnTo(-30, 1000)
                .waitMilliseconds(400)
                .callback(() -> new AutonomousShootCommand().schedule())
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 300)
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
                        new Point2d(-15, -12),
                        new Point2d(33, -46),
                        new Point2d(40, -52),

                }, 2000)
//                                .waitMilliseconds(300)
                .callback(() -> {
                    new SequentialCommandGroup(
                            //new ReadBallColorsCommand(),
                            //new WaitCommand(100),
                            new WaitCommand(400),
                            new SetShooterVelocityIndependentCommand(velo-40, veloMiddle, velo-20),
                            new AutonomousTransferCommand(hood),
                            new WaitCommand(700),
//                                                        new TurnTurretToPosCommand(-70),
//                                                        new WaitCommand(400),
                            new LockOnGoalCommand())
                            .schedule();
                })
//                                .waitMilliseconds(100)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(33, -48),
                        new Point2d(-3, -13)
                }, 2000)
                .waitUntil(()-> Robot.getInstance().turret.atTarget(), 500)
                .callback(() -> new AutonomousShootCommand().schedule())
                .waitMilliseconds(200)
                .build();
    }

    private Path buildParkPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-3,-13),
                        new Point2d(10, -19)
                }, 5000)
                .build();
    }
}
