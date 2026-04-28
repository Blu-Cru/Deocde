package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootFlipTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootWithMotifCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferThenLockOnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.ReadBallColorsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.TurnOffShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

/**
 * Close Blue Auto with Motif Scoring Support.
 */
public class CloseBlueAutoMotif extends BaseAuto {
    double velo = 1170;
    double veloMiddle = 1270;
    boolean alreadySignalledPattern;
    double hood = 40;

    enum State {
        PRELOAD,
        FIRST_SET,
        SECOND_SET,
        THIRD_SET,
        HP_SET
    }
    private Point2d shootingPose = new Point2d(4, -8);

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-51, -54, Math.toRadians(51.529));
    }

    @Override
    public StateMachine buildStateMachine() {
        return new StateMachineBuilder()
                .state(State.PRELOAD)
                .transition(() -> currentPath != null && currentPath.isDone(), State.FIRST_SET,
                        () -> {
                            startPath(firstSetPath());
                        })

                .state(State.FIRST_SET)
                // Transition if path completes normally
                .transition(() -> currentPath != null && currentPath.isDone(), State.SECOND_SET, () -> {
                    startPath(secondSetPath());
                })

                .state(State.SECOND_SET)
                .transition(() -> currentPath != null && currentPath.isDone(),
                        State.THIRD_SET, () -> {
                            startPath(thirdSetPath());
                        })
                .state(State.THIRD_SET)
                .transition(() -> currentPath != null && currentPath.isDone(),
                        State.HP_SET, () -> {
                    startPath(hpSetPath());
                })
                .state(State.HP_SET)
                .onEnter(() -> {
                })
                .build();
    }

    Path currentPath;

    @Override
    public void initialize() {
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
        alreadySignalledPattern = false;
        sixWheel.reset();

        ShooterMotifCoordinator.clear();
        super.initialize();
    }

    @Override
    public void onStart() {
        shooter.shootWithVelocityIndependent(925,950,925);
        turret.setAngle(2);
        llTagDetector.switchToMotif();
        sixWheel.setPosition(startPose);
        currentPath = preloadPath();
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

    @Override
    public void autoTelemetry(Telemetry telemetry){
        telemetry.addData("Motif", ShooterMotifCoordinator.getMotif());
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

    private Path preloadPath(){
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-52, -54),
                        new Point2d(-40, -41)
                }, 1000)
                .callback(() -> {
                    new AutonomousShootCommand(25).schedule();
                })
                .waitUntil(() -> Robot.getInstance().shooter.hasShot(3), 200)
                .addTurnTo(-25,5000)
                .build();
    }

    private Path firstSetPath(){
        return new SixWheelPIDPathBuilder()
                .callback(() -> {
                    new SequentialCommandGroup(
                            new WaitCommand(400),
                            new CenterTurretCommand()
                    ).schedule();
                })
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-40, -41),
                        new Point2d(-14, -47)
                }, 2000)
                .waitMilliseconds(500)
                // Transfer - Wait for stillness, read colors, then transfer
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(500,velo, veloMiddle,velo,hood);
                })
                .waitMilliseconds(50)
                .addTurnTo(-90, 5000)
                // HEAD BACK
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(-14, -45), // was (-10, 50)
                        new Point2d(-14, -35), // was (-10, 17)
                        shootingPose
                }, 2000)
                .addTurnTo(-90, 1000)
                // SHOOT FIRST SET - Use motif-aware shooting
                .waitMilliseconds(400)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootFlipTurretCommand()).schedule();
                })
                .waitUntil(() -> shooter.hasShot(3), 200)
                .build();
    }


    private Path secondSetPath(){
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose,
                        new Point2d(10,-25),
                        new Point2d(12, -33),
                        new Point2d(12, -46),
                        new Point2d(7, -59),
                }, 2000)
                .waitMilliseconds(50)

                // Transfer - Wait for stillness, read colors, then transfer
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(500,velo, veloMiddle,velo,hood);
                })

                // HEAD BACK
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(10, -60), // was (12.5, 46)
                        shootingPose
                }, 2000, true)
                .waitMilliseconds(400)

                // SHOOT SECOND SET - Use motif-aware shooting
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootWithMotifCommand()).schedule();
                })
                .waitUntil(() -> shooter.hasShot(3), 2000)
                .build();
    }

    private Path thirdSetPath(){
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose, // was (-10, 17)
                        new Point2d(10, -30),
                        new Point2d(32, -46),
                        new Point2d(39, -52),
                }, 1100)
                // Transfer - Wait for stillness, read colors, then transfer
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(500, velo,veloMiddle,velo,hood);
                })
                .waitMilliseconds(400)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(36, -45),
//                        new Point2d(10, -30),
                        shootingPose // was (-10, 17)
                }, 1200)
                //.waitMilliseconds(1000)
                .waitUntil(() -> turret.atTarget(),750)
                //small time for settling
                .waitMilliseconds(50)
                // SHOOT THIRD SET - Use motif-aware anti-jam shooting
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootWithMotifCommand(),
                            new WaitCommand(300),
                            new IntakeStopCommand()).schedule();
                })
                .waitUntil(() -> shooter.hasShot(3), 2000)
                .build();
    }

    private Path hpSetPath(){
        return new SixWheelPIDPathBuilder()
                .addPurePursuitPath(new Point2d[] {
                        shootingPose,
                        new Point2d(10, -20),
                        new Point2d(30, -45),
                        new Point2d(52, -55),
                        new Point2d(63,-60)
                }, 3000)
                .waitMilliseconds(500)
                .callback(() -> {
                    scheduleVelocityTransferThenLockOn(500, velo,veloMiddle,velo, hood);

                })
                .waitMilliseconds(400)
                .addPurePursuitPath(new Point2d[] {
                        new Point2d(63, -60),
                        new Point2d(59, -50),
                        new Point2d(30,-30),
                        shootingPose // was (-10, 17)
                }, 3000,true)
                //.waitMilliseconds(1000)
                .waitMilliseconds(200)
                // SHOOT THIRD SET - Use motif-aware anti-jam shooting
                .callback(() -> {
                    new SequentialCommandGroup(
                            new AutonomousShootWithMotifCommand(),
                            new WaitCommand(300),
                            new IntakeStopCommand()).schedule();
                })
                .waitUntil(() -> shooter.hasShot(3), 2000)
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
