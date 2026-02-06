package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
//import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootWithMotifAntiJamCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootWithMotifCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.ReadBallColorsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

/**
 * Close Red Auto with Motif Scoring Support.
 * 
 * This autonomous uses the Limelight to detect the motif pattern from the
 * obelisk AprilTag.
 * It reads ball colors while stationary after intake paths to ensure accurate
 * readings without
 * impacting loop times during movement.
 */
@Autonomous(name = "PP Close Red Auto (Motif)")
public class PPCloseRedAutoMotif extends BluLinearOpMode {
    double turretAngle = 216; // field centric, decrease = towards obelisk increase = towards gate
    double velo = 1120;
    double veloMiddle = 1140;
    double leftHood=34;
    double middleHood=34;
    double rightHood=34;
    boolean alreadySignalledPattern;

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();

            // Shift applied: dx = -6, dy = +2 (old start -45,52 -> new start -51,54)

            this.addPurePursuitPath(new Point2d[] {
                    new Point2d(-51, 54), // was (-45, 52)
                    new Point2d(-16, 19) // was (-10, 17)
            }, 4000)
                    .waitMilliseconds(100)
                    .addTurnTo(10,1000)

                    .waitMilliseconds(300)
                    // SHOOT PRELOAD - preload doesn't need motif (no color sensing yet)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()).schedule();
                    })

                    // INTAKE FIRST SET
                    .waitMilliseconds(400)
                    .addTurnTo(90, 5000)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, 19),
                            new Point2d(-14, 35),
                            new Point2d(-6, 47),
//                            new Point2d(-6, 57)
                    }, 2000)
                    .addTurnTo(90, 500)
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-6,47),
                            new Point2d(-6,57)
                    }, 1000)

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
                    .waitMilliseconds(500) // Ensure enough time for the sequence above to complete

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-6, 57), // was (-10, 50)
                            new Point2d(-16, 19) // was (-10, 17)
                    }, 2000)
//                    .addTurnTo(45, 1000)
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
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)).schedule();

                    })
                    .waitMilliseconds(700)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(13, 49), // was (12.5, 46)
                            new Point2d(-16, 19) // was (-10, 17)
                    }, 2000)
//                    .addTurnTo(45, 1000)
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
//                    .addTurnTo(45, 500)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-16, 19), // was (-10, 17)
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
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)).schedule();

                    })
                    .waitMilliseconds(700)
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(36, 49),
                            new Point2d(-16, 19) // was (-10, 17)
                    }, 2000)
                    .addTurnTo(0, 1000)
                    .waitMilliseconds(400)
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

    public void initialize() {
        robot.clear();
        addSixWheel();
        addIntake();
        addElevator();
        addShooter();
        addTurret();
        addTransfer();
        addLLTagDetector();
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

        // Clear any previous ball color readings
        ShooterMotifCoordinator.clear();



    }

    public void onStart() {
        shooter.shootWithVelocity(1120); // orig 850 before switching to triple shot
        turret.setAngle(57);
        llTagDetector.switchToMotif();
        currentPath = new TestingPath().build().start();
        sixWheel.setPosition(new Pose2d(-51, 54, Math.toRadians(-51.529)));
        Globals.setAlliance(Alliance.RED);
    }

    public void periodic() {
        currentPath.run();

        // Read limelight to detect motif pattern from obelisk
        if (!alreadySignalledPattern) {
            llTagDetector.read();
            if (llTagDetector.detectedPattern()) {
                // Signal pattern detected with LED
                gamepad1.setLedColor(100, 255, 100, 1000);
                alreadySignalledPattern = true;

                // Add telemetry for the detected motif
                telemetry.addData("Detected Motif", ShooterMotifCoordinator.getMotif());
            }
        }

        // Display current ball colors for debugging
        telemetry.addData("Left Ball", ShooterMotifCoordinator.getLeftColor());
        telemetry.addData("Middle Ball", ShooterMotifCoordinator.getMiddleColor());
        telemetry.addData("Right Ball", ShooterMotifCoordinator.getRightColor());
        telemetry.addData("Motif Pattern", ShooterMotifCoordinator.getMotif());
    }
}
