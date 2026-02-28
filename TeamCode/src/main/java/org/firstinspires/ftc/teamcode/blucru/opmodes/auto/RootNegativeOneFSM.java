package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootWithMotifCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.ReadBallColorsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.TurnOffShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

@Autonomous
public class RootNegativeOneFSM extends BaseAuto {
    double turretAngle     = 142;
    double preAimTurretAngle = -90;
    double velo            = 1100;
    double veloMiddle      = 1100;
    double leftHood        = 32;
    double middleHood      = 32;
    double rightHood       = 32;

    double GATE_CYCLE_TIME_THRESHOLD = 20;

    enum State { PRELOAD, MIDDLE_SPIKE, GATE_CYCLE, LAST_GATE_CYCLE_PICKUP, LAST_GATE_CYCLE_SHOOT, CLOSE_SPIKE, FAR_SPIKE, PARK, IDLE}
    StateMachine autoStateMachine;

    /*public class AutoPath extends SixWheelPIDPathBuilder {
        public AutoPath() {
            super();
            this
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-51, -54),
                            new Point2d(-20, -25)
                    }, 2000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-20, -25),
                            //small guide point for the turn
                            new Point2d(-19, -15),
                            new Point2d(-5, -47),
                    }, 2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-5, -47),
                            new Point2d(-5, -19)
                    }, 2000)
                    .callback(() -> {
                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
                    })
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .callback(() -> {
                        new IntakeStartCommand().schedule();
                    })
                    .addTurnTo(-90, 1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, -19),
                            new Point2d(14, -58)
                    }, 2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(14, -58),
                            new Point2d(-16, -19)
                    }, 2000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .callback(() -> {
                        new IntakeStartCommand().schedule();
                    })
                    .addTurnTo(-90, 1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, -19),
                            new Point2d(14, -58)
                    }, 2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(14, -58),
                            new Point2d(-16, -19)
                    }, 2000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, -19),
                            new Point2d(-16, -37)
                    }, 2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, -37),
                            new Point2d(-16, -19)
                    }, 2000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    });
        }
    }*/

    Path currentPath;

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-51, -54, Math.toRadians(51.529));
    }

    @Override
    public StateMachine buildStateMachine() {
        return new StateMachineBuilder()
                .state(State.PRELOAD)
                    .transition(() -> currentPath != null && currentPath.isDone(), State.MIDDLE_SPIKE, () -> {
                    startPath(buildSpikeMiddlePath());
                })

                .state(State.MIDDLE_SPIKE)
                // Transition if path completes normally
                .transition(() -> currentPath != null && currentPath.isDone(), State.GATE_CYCLE, () -> {
                    startPath(buildIntakeCyclePath());
                })

                .state(State.GATE_CYCLE)
                .transition(() -> currentPath != null && currentPath.isDone(), State.GATE_CYCLE, () -> {
                    startPath(buildIntakeCyclePath());
                })
                // Park if time is running out
                .transition(() -> currentPath != null && currentPath.isDone()
                        && Globals.matchTime.seconds() >= GATE_CYCLE_TIME_THRESHOLD, State.LAST_GATE_CYCLE_PICKUP, () -> {
                    startPath(buildIntakeMotifCyclePath());
                })

                .state(State.LAST_GATE_CYCLE_PICKUP)
                //Shoot if picked up
                .transition(() -> currentPath != null && currentPath.isDone(), State.LAST_GATE_CYCLE_SHOOT, () -> {
                    startPath(buildShootMotifCyclePath());
                })

                .state(State.LAST_GATE_CYCLE_SHOOT)
                //Get close spike once shot
                .transition(() -> currentPath != null && currentPath.isDone(), State.CLOSE_SPIKE, () -> {
                    startPath(buildSpikeClosePath());
                })

                .state(State.CLOSE_SPIKE)
                //Get far spike once shot
                .transition(() -> currentPath != null && currentPath.isDone(), State.FAR_SPIKE, () -> {
                    startPath(buildSpikeFarPath());
                })

                .state(State.FAR_SPIKE)
                //Park once shot
                .transition(() -> currentPath != null && currentPath.isDone(), State.PARK, () -> {
                    startPath(buildParkPath());
                })

                .state(State.PARK)
                //Auto Done, switch to IDLE
                .transition(() -> currentPath != null && currentPath.isDone(), State.IDLE, () -> {
                })

                .state(State.IDLE)
                .onEnter(this::stopIntakeShooterAndPath)

                .build();
    }

    @Override
    public void initialize() {
        shooter.setHoodAngleIndependent(30, 30, 30);
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

    public void onStart(){
        Globals.matchTime.reset();
        shooter.shootWithVelocity(1150);
        turret.setAngle(5);
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
     * */
    private Path buildPreloadPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(-51, -54),
                        new Point2d(-20, -22)
                }, 2000)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootCommand()
                    ).schedule();
                })
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 400)
                .build();
    }

    /**
     *
     * This path is to get the middle spike without opening the gate and then shoot them
     *
     * */
    private Path buildSpikeMiddlePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(-20, -22),
                        //small guide point for the turn
                        new Point2d(-5, -15),
                        new Point2d(7, -47),
                }, 2000)

                .callback(() -> {new SequentialCommandGroup(
                        new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                        new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                        new WaitCommand(700),
                        new TurnTurretToPosCommand(preAimTurretAngle)).schedule();})

                .addPurePursuitPath(new Point2d[]{
                        new Point2d(7, -47),
                        new Point2d(7, -19)
                }, 2000)

                .callback(() -> {
                    new AutonomousShootCommand().schedule();
                })

                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 400)
                .build();
    }

    private Path buildIntakeCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(7, -19),
                        new Point2d(11, -44),
                        new Point2d(7, -56)
                }, 2000)
                .waitMilliseconds(1000)
                .callback(() -> {new SequentialCommandGroup(
                        new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                        new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                        new WaitCommand(700),
                        new TurnTurretToPosCommand(preAimTurretAngle)).schedule();})
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(7, -56),
                        new Point2d(7, -19)
                }, 2000)
                .waitMilliseconds(100)
                .callback(() ->
                        new AutonomousShootCommand().schedule())
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 400)
                .build();
    }

    private Path buildIntakeMotifCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(7, -19),
                        new Point2d(11, -44),
                        new Point2d(7, -56)
                }, 2000)
                .waitMilliseconds(1000)
                .callback(() -> {new SequentialCommandGroup(
                        new ReadBallColorsCommand(),
                        new WaitCommand(100),
                        new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                        new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                        new WaitCommand(700),
                        new TurnTurretToPosCommand(preAimTurretAngle)).schedule();})
                .waitMilliseconds(0)
                .build();
    }

    private Path buildShootMotifCyclePath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(7, -56),
                        new Point2d(11, -44),
                        new Point2d(-12, -15)
                }, 2000)
                .waitMilliseconds(100)
                .callback(() ->
                        new AutonomousShootWithMotifCommand().schedule())
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 400)
                .build();
    }
    /**
     *
     * This path grabs the close spikes
     *
     * */
    private Path buildSpikeClosePath(){
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-12, -15),
                        new Point2d(-16, -37)
                }, 2000)
                .waitMilliseconds(100)
                .callback(() -> {new SequentialCommandGroup(
                        new ReadBallColorsCommand(),
                        new WaitCommand(100),
                        new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                        new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                        new WaitCommand(700),
                        new TurnTurretToPosCommand(preAimTurretAngle)).schedule();})
                .waitMilliseconds(100)
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(-16, -37),
                        new Point2d(-16, -20)
                }, 2000)
                .addTurnTo(-20, 1000)
                .waitMilliseconds(100)
                .callback(() -> new AutonomousShootWithMotifCommand().schedule())
                .waitMilliseconds(0)
                .build();
    }

    /**
     *
     * This path grabs and shoots the far spikes
     *
     * */
    private Path buildSpikeFarPath(){
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-16, -37),
                        new Point2d(16, -58)
                }, 2000)
                .waitMilliseconds(100)
                .callback(() -> {new SequentialCommandGroup(
                        new ReadBallColorsCommand(),
                        new WaitCommand(100),
                        new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                        new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                        new WaitCommand(700),
                        new TurnTurretToPosCommand(preAimTurretAngle)).schedule();})
                .waitMilliseconds(100)
                .addPurePursuitPath(new Point2d[]{
                        new Point2d(16, -58),
                        new Point2d(-5, -10)
                }, 2000)
                .waitMilliseconds(100)
                .callback(() -> new AutonomousShootWithMotifCommand().schedule())
                .waitMilliseconds(0)
                .build();
    }

    private Path buildParkPath() {
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-5, -10),
                        new Point2d(5, -20)
                }, 5000)
                .build();
    }
}