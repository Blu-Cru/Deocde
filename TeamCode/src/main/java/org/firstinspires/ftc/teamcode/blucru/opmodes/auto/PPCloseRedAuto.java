package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootAntiJamCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous
public class PPCloseRedAuto extends BluLinearOpMode {
    double turretAngle =230; //field centric, robot centric is 85
    double velo =1180;
    double leftHood;
    double middleHood;
    double rightHood;
    boolean alreadySignalledPattern;

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();

            // Shift applied: dx = -6, dy = +2 (old start -45,52 -> new start -51,54)

            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(-51, 54),   // was (-45, 52)
                            new Point2d(-16, 19)    // was (-10, 17)
                    }, 5000)
                    .waitMilliseconds(500)
                    // SHOOT PRELOAD
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })

                    // INTAKE FIRST SET
                    .waitMilliseconds(200)
                    .addTurnTo(90, 5000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, 19),
                            new Point2d(-16,33),
                            new Point2d(-6,45),
                            new Point2d(-5, 57)
                    }, 2000)

                    .callback(() -> {
                        new SequentialCommandGroup(
                                new IntakeStopCommand(),
                                new WaitCommand(100),
                                new SetShooterVelocityIndependentCommand(velo, velo, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(2500),
                                new TurnTurretToPosCommand(85)
                        ).schedule();
                    })
                    .waitMilliseconds(500)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-5, 54),   // was (-10, 50)
                            new Point2d(-16, 19)    // was (-10, 17)
                    }, 2000)
                    .addTurnTo(45,1000)
//                    .callback(()->{
//                        new LockOnGoalCommand().schedule();
//                    })
                    // SHOOT FIRST SET
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    // INTAKE SECOND SET
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, 19),
//                            new Point2d(0,30), use for gate open
//                            new Point2d(10, 48) use for gate open
                            new Point2d(10,46)
                    }, 2000)
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new IntakeStopCommand(),
                                new WaitCommand(100),
                                new SetShooterVelocityIndependentCommand(velo, velo, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(2000),
                                new TurnTurretToPosCommand(85)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(6.5, 54),   // was (12.5, 46)
                            new Point2d(-16, 19)    // was (-10, 17)
                    }, 2000)
                    .addTurnTo(45,1000)
//                    .callback(()->{
//                        new LockOnGoalCommand().schedule();
//                    })
                    .waitMilliseconds(1000)

                    // SHOOT SECOND SET
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(200)
                    .addTurnTo(35,500)

                    // PICKUP THIRD SET
                    .addTurnTo(45, 1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, 19),   // was (-10, 17)
                            new Point2d(33, 48)     // was (37, 46)
                    }, 1100)
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new IntakeStopCommand(),
                                new WaitCommand(100),
                                new SetShooterVelocityIndependentCommand(velo, velo, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(2500),
                                new TurnTurretToPosCommand(85)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(36, 48),
//                            new Point2d(0,25),
                            new Point2d(-16, 19)    // was (-10, 17)
                    }, 2000)
//                    .waitMilliseconds(1000)
                    .addTurnTo(45,1000)
//                    .callback(()->{
//                        new LockOnGoalCommand().schedule();
//                    })
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootAntiJamCommand(),
                                new WaitCommand(300),
                                new IntakeSpitCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, 19),    // was (37, 46)
                            new Point2d(0, 30)    // was (-10, 17)
                    }, 1300)
                    // SHOOT THIRD SET
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
        shooter.setHoodAngleIndependent(30, 30, 30);
        shooter.write();
        elevator.setMiddle();
        elevator.write();
        transfer.setAllMiddle();
        transfer.write();
        turret.resetEncoder();
        turret.write();
        turret.setAngle(10);
        turret.write();
        sixWheel.reset();
        sixWheel.write();
        intake.resetEncoder();
        intake.write();
        alreadySignalledPattern = false;
    }

    public void onStart() {
        shooter.shootWithVelocity(1120); // orig 850 before switching to triple shot
        turret.setAngle(90);
        llTagDetector.switchToMotif();
        sixWheel.setPosition(new Pose2d(-51, 54, Math.toRadians(-51.529)));
        currentPath = new TestingPath().build().start();
    }

    public void periodic() {
        currentPath.run();
        if (!alreadySignalledPattern){
            llTagDetector.read();
            if (llTagDetector.detectedPattern()){
                gamepad1.setLedColor(100,255,100, 1000);
                alreadySignalledPattern = true;
            }
        }
        telemetry.addData("Turret Target Pos", turret.getTargetPosition());
    }
}
