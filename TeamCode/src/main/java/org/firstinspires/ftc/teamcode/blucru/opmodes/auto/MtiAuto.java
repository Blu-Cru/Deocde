package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

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
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotifPattern;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

/**
 * Close Blue Auto with Motif Scoring Support.
 */
// @Autonomous(name = "PP Close Blue Auto (Motif)")
public class MtiAuto extends BaseAuto {
    double turretAngle = 150; ////field centric, decrease = more towards gate, increase = towards obelisk
    double nonFieldCentricTurretAngle = -87;
    double velo = 1115;
    double veloMiddle = 1130;
    double farVeloLeft = 1500;
    double farVeloMiddle = 1500;
    double farVeloRight = 1500;
    double hoodFar = 49;
    double hood = 34;
    boolean alreadySignalledPattern;

    enum State {
        RUNNING
    }

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();

            this.addPurePursuitPath(new Point2d[] {
                    new Point2d(-51, -54), // was (-45, 52)
                    new Point2d(-16, -19) // was (-10, 17)
            }, 4000)
                    .waitMilliseconds(100)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()).schedule();
                    })
                    .waitUntil(() -> shooter.hasShot(3), 200)

                    //TURN TO CLOSE SET
                    .addTurnTo(-10,1000)

                    .waitMilliseconds(50) // TODO: Remove this check if it is turned to move on

                    // INTAKE CLOSE SET
                    .addTurnTo(-90, 5000)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, -19),
                            new Point2d(-14, -47)
                    }, 2000)

                    // Transfer - Wait for stillness, read colors, then transfer
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(hood),
                                new WaitCommand(700),
                                new LockOnGoalCommand()).schedule();
                        alreadySignalledPattern = true;
                    })
                    .waitMilliseconds(100)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-14, -47), // was (-10, 50)
                            new Point2d(-16, -19) // was (-10, 17)
                    }, 2000)
//                    .addTurnTo(-45, 1000)
//                    .callback(
//                            () -> {new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();}
//                    )
                    // SHOOT FIRST SET - Use motif-aware shooting
                    .waitMilliseconds(2000)
                    .waitUntil(() -> turret.atTarget(),750)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()).schedule();
                    })
                    .waitUntil(() -> shooter.hasShot(3), 2000)

                    // INTAKE MIDDLE SPIKE
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, -19),
                            new Point2d(-10, -24),
                            new Point2d(7, -28),
                            new Point2d(10, -50),
                            new Point2d(3, -63)
                    }, 2000)
                    .waitMilliseconds(100)
                    // Transfer - Wait for stillness, read colors, then transfer
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new ReadBallColorsCommand(), // Read all color sensors at once
                                new WaitCommand(300),
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(hood),
                                new WaitCommand(700),
                                new LockOnGoalCommand()).schedule();

                    })
                    .waitMilliseconds(300)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(3, -63), // was (12.5, 46)
                            new Point2d(-16, -19) // was (-10, 17)
                    }, 2000)
//                    .callback(() -> {
//                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
//                    })
                    .waitUntil(() -> turret.atTarget(),1000)
                    .waitMilliseconds(400)
                    //small time for settling
                    .waitMilliseconds(50)

                    // SHOOT SECOND SET - Use motif-aware shooting
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootWithMotifCommand()).schedule();
                    })
                    .waitUntil(() -> shooter.hasShot(3), 2000)

                    // PICKUP THIRD SET
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, -19), // was (-10, 17)
//                            new Point2d(10, -30),
                            new Point2d(38, -52) // was (37, 46)
                    }, 1100)
                    .addTurnTo(-31, 500)
                    .waitMilliseconds(500)
                    // Transfer - Wait for stillness, read colors, then transfer
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new ReadBallColorsCommand(), // Read all color sensors at once
                                new WaitCommand(100),
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(hood),
                                new WaitCommand(700),
                                new LockOnGoalCommand()).schedule();

                    })
                    .waitMilliseconds(400)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(38, -52),
                            new Point2d(10, -30),
                            new Point2d(-16, -19) // was (-10, 17)
                    }, 2000)
//                    .callback(() -> {
//                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
//                    })
                    .waitUntil(() -> turret.atTarget(),2000)
                    //small time for settling
                    .waitMilliseconds(50)
                    // SHOOT THIRD SET - Use motif-aware anti-jam shooting
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootWithMotifCommand(),
                                new WaitCommand(300)
//                                new IntakeStopCommand()
                        ).schedule();
                    })
                    .waitUntil(() -> shooter.hasShot(3), 2000)
                    //PICKUP FARTHEST SET
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, -19),
                            new Point2d(45, -30),
                            new Point2d(62, -62)
                    }, 2500)
                    .waitMilliseconds(100)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new ReadBallColorsCommand(),
                                new WaitCommand(100),
                                new SetShooterVelocityIndependentCommand(farVeloLeft, farVeloMiddle, farVeloRight),
                                new AutonomousTransferCommand(hoodFar),
                                new WaitCommand(700),
                                new LockOnGoalCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(100)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(62, -62),
                            new Point2d(45, -9)
                    }, 2000)
                    .waitMilliseconds(3000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootWithMotifCommand(),
                                new WaitCommand(300)
                        ).schedule();

                    })
                    .waitUntil(() -> shooter.hasShot(3), 400)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(45, -9),
                            new Point2d(45, -20)
                    }, 1500)
                    .build();
        }
    }

    Path currentPath;

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-51, -54, Math.toRadians(51.529));
    }

    @Override
    public StateMachine buildStateMachine() {
        return new StateMachineBuilder()
                .state(State.RUNNING)
                .loop(() -> {
                    if (currentPath != null) {
                        currentPath.run();
                    }
                    if (!alreadySignalledPattern) {
                        gamepad1.setLedColor(100, 255, 100, 1000);
                        alreadySignalledPattern = true;
                        telemetry.addData("Detected Motif", ShooterMotifCoordinator.getMotif());
                    }

                    telemetry.addData("Left Ball", ShooterMotifCoordinator.getLeftColor());
                    telemetry.addData("Middle Ball", ShooterMotifCoordinator.getMiddleColor());
                    telemetry.addData("Right Ball", ShooterMotifCoordinator.getRightColor());
                    telemetry.addData("Motif Pattern", ShooterMotifCoordinator.getMotif());
                })
                .build();
    }

    @Override
    public void initialize() {
        robot.addTurretCam();
        ShooterMotifCoordinator.setMotif(MotifPattern.PPG);
        shooter.setHoodAngle(32);
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
        shooter.shootWithVelocity(1120); // orig 850 before switching to triple shot
        turret.setAngle(7);
        sixWheel.setPosition(startPose);
        currentPath = new TestingPath().build().start();
        Globals.setAlliance(Alliance.BLUE);

        sm.setState(State.RUNNING);
        sm.start();
    }

    @Override
    public void periodic() {
        sm.update();
    }
}
