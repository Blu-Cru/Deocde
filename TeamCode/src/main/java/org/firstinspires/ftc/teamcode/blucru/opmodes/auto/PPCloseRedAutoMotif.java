package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootWithMotifCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.ReadBallColorsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

/**
 * Close Red Auto with Motif Scoring Support.
 */
// @Autonomous(name = "PP Close Red Auto (Motif)")
public class PPCloseRedAutoMotif extends BaseAuto {
    double turretAngle = 217; // field centric, decrease = towards obelisk increase = towards gate
    double velo = 1110;
    double veloMiddle = 1120;
    double hood = 34;
    boolean alreadySignalledPattern;

    enum State {
        RUNNING
    }

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();

            this.addPurePursuitPath(new Point2d[] {
                    new Point2d(-51, 54), // was (-45, 52)
                    new Point2d(-16, 19) // was (-10, 17)
            }, 4000)
                    .waitMilliseconds(500)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()).schedule();
                    })
                    .waitMilliseconds(300)
                    .addTurnTo(10,1000)

                    // INTAKE FIRST SET
                    .waitMilliseconds(400)
                    .addTurnTo(90, 5000)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, 19),
                            new Point2d(-14, 35),
                            new Point2d(-5, 47),
                    }, 2000)
                    .addTurnTo(90, 500)
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-5,47),
                            new Point2d(-5,60)
                    }, 1000)

                    // Transfer - Wait for stillness, read colors, then transfer
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new ReadBallColorsCommand(), // Read all color sensors at once
                                new WaitCommand(300),
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(hood)).schedule();
                        alreadySignalledPattern = true;
                        llTagDetector.switchToPosition();
                    })
                    .waitMilliseconds(500)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-5, 60), // was (-10, 50)
                            new Point2d(-16, 19) // was (-10, 17)
                    }, 2000)
                    .waitMilliseconds(400)
                    .callback(() -> {
                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
                    })
                    // SHOOT FIRST SET - Use motif-aware shooting
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootWithMotifCommand()).schedule();
                    })
                    .waitMilliseconds(1750)

                    // INTAKE SECOND SET
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, 19),
                            new Point2d(15, 49)
                    }, 2000)
                    .waitMilliseconds(300)
                    // Transfer - Wait for stillness, read colors, then transfer
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new ReadBallColorsCommand(), // Read all color sensors at once
                                new WaitCommand(300),
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(hood)).schedule();

                    })
                    .waitMilliseconds(700)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(13, 49), // was (12.5, 46)
                            new Point2d(-16, 19) // was (-10, 17)
                    }, 2000)
                    .waitMilliseconds(400)
                    .callback(() -> {
                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
                    })
                    .waitMilliseconds(1000)

                    // SHOOT SECOND SET - Use motif-aware shooting
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootWithMotifCommand()).schedule();
                    })
                    .waitMilliseconds(1750)

                    // PICKUP THIRD SET
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, 19),
                            new Point2d(36, 50) // was (37, 46)
                    }, 1100)
                    .addTurnTo(10, 500)
                    .waitMilliseconds(1000)
                    // Transfer - Wait for stillness, read colors, then transfer
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new ReadBallColorsCommand(), // Read all color sensors at once
                                new WaitCommand(300),
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(hood)).schedule();

                    })
                    .waitMilliseconds(700)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(36, 49),
                            new Point2d(-16, 19) // was (-10, 17)
                    }, 1500)
                    .addTurnTo(0, 1000)
                    .waitMilliseconds(700)
                    .callback(() -> {
                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
                    })
                    .waitMilliseconds(1000)
                    // SHOOT THIRD SET - Use motif-aware anti-jam shooting
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootWithMotifCommand(),
                                new WaitCommand(300),
                                new IntakeStopCommand()).schedule();
                    })
                    .waitMilliseconds(1760)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, 19),
                            new Point2d(10, 30)
                    }, 1300)
                    .build();
        }
    }

    Path currentPath;

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-51, 54, Math.toRadians(-51.529));
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
                        llTagDetector.read();
                        if (llTagDetector.detectedPattern()) {
                            gamepad1.setLedColor(100, 255, 100, 1000);
                            alreadySignalledPattern = true;
                            telemetry.addData("Detected Motif", ShooterMotifCoordinator.getMotif());
                        }
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
        turret.setAngle(-7);
        llTagDetector.switchToMotif();
        sixWheel.setPosition(startPose);
        currentPath = new TestingPath().build().start();
        Globals.setAlliance(Alliance.RED);

        sm.setState(State.RUNNING);
        sm.start();
    }

    @Override
    public void periodic() {
        sm.update();
    }
}
