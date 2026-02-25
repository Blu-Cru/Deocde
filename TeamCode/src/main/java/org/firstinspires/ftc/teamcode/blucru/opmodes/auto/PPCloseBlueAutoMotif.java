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
 * Close Blue Auto with Motif Scoring Support.
 */
// @Autonomous(name = "PP Close Blue Auto (Motif)")
public class PPCloseBlueAutoMotif extends BaseAuto {
    double turretAngle = 139; ////field centric, decrease = more towards gate, increase = towards obelisk
    double velo = 1115;
    double veloMiddle = 1130;
    double leftHood = 34;
    double middleHood = 34;
    double rightHood = 34;
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
                    .waitMilliseconds(200) // TODO: Remove this check if the velocity has shot and changed
                    .addTurnTo(-10,1000)

                    .waitMilliseconds(100) // TODO: Remove this check if it is turned to move on

                    // INTAKE FIRST SET
                    .addTurnTo(-90, 5000)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, -19),
                            new Point2d(-16, -37),
                            new Point2d(-5, -47),
                            new Point2d(-5,-60)
                    }, 2000)

                    // Transfer - Wait for stillness, read colors, then transfer
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new ReadBallColorsCommand(), // Read all color sensors at once
                                new WaitCommand(300),
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)).schedule();
                        alreadySignalledPattern = true;
                        llTagDetector.switchToPosition();
                    })
                    .waitMilliseconds(300)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-6, -57), // was (-10, 50)
                            new Point2d(-16, -19) // was (-10, 17)
                    }, 2000)
                    .addTurnTo(-45, 1000)
                    .callback(() -> {
                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
                    })
                    // SHOOT FIRST SET - Use motif-aware shooting
                    .waitMilliseconds(1200)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootWithMotifCommand()).schedule();
                    })
                    .waitMilliseconds(1750) // TODO: Remove this check if the velocity has shot and changed

                    // INTAKE SECOND SET
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, -19),
                            new Point2d(17, -51)
                    }, 2000)
                    .waitMilliseconds(100)
                    // Transfer - Wait for stillness, read colors, then transfer
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new ReadBallColorsCommand(), // Read all color sensors at once
                                new WaitCommand(300),
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)).schedule();

                    })
                    .waitMilliseconds(300)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(13, -49), // was (12.5, 46)
                            new Point2d(-16, -19) // was (-10, 17)
                    }, 2000)
                    .callback(() -> {
                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
                    })
                    .waitMilliseconds(1000)

                    // SHOOT SECOND SET - Use motif-aware shooting
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootWithMotifCommand()).schedule();
                    })
                    .waitMilliseconds(1850) // TODO: Remove this check if the velocity has shot and changed

                    // PICKUP THIRD SET
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, -19), // was (-10, 17)
                            new Point2d(10, -30),
                            new Point2d(36, -45) // was (37, 46)
                    }, 1100)
                    .addTurnTo(-31, 500)
                    .waitMilliseconds(500)
                    // Transfer - Wait for stillness, read colors, then transfer
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new ReadBallColorsCommand(), // Read all color sensors at once
                                new WaitCommand(300),
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(1000),
                                new TurnTurretToPosFieldCentricCommand(turretAngle+2)).schedule();

                    })
                    .waitMilliseconds(400)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(36, -45),
                            new Point2d(10, -30),
                            new Point2d(-16, -19) // was (-10, 17)
                    }, 1200)
                    .waitMilliseconds(1000)
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
                    .waitMilliseconds(1700) // TODO: Remove this check if the velocity has shot and changed
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, -19),
                            new Point2d(10, -30)
                    }, 1300)
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
        shooter.setHoodAngleIndependent(32, 32, 32);
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
        llTagDetector.switchToMotif();
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
