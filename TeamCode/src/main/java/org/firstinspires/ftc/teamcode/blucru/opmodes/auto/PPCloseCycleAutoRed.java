package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous
public class PPCloseCycleAutoRed extends BluLinearOpMode {
    double turretAngle = 142; //field centric, decrease = more towards gate, increase = towards obelisk
    double velo =1120;
    double veloMiddle = 1140;
    double leftHood=34;
    double middleHood=34;
    double rightHood=34;
    boolean alreadySignalledPattern;

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();


            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(-49, 54),   // was (-45, 52)
                            new Point2d(-16, 19)    // was (-10, 17)
                    }, 5000)
                    .waitMilliseconds(500)
                    // SHOOT PRELOAD
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })

                    // INTAKE MIDDLE SET
                    .waitMilliseconds(200)
                    .addTurnTo(-90, 5000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, 19),
                            new Point2d(-16,25),
                            new Point2d(-4,35),
                            new Point2d(6, 57)
                    }, 2000)

                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(6, 57),   // was (-10, 50)
                            new Point2d(-16, 19)    // was (-10, 17)
                    }, 2000)
                    .addTurnTo(-45,1000)
                    .waitMilliseconds(500)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
                    })
                    // SHOOT FIRST SET
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    //GATE STEAL
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, 19),
                            new Point2d(8, 45),
                            new Point2d(5, 57)
                    }, 5000)
                    .waitMilliseconds(3000)
                    //HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(5, 57),
                            new Point2d(0, 50),
                            new Point2d(-16, 19)
                    }, 5000)
                    //SHOOT
                    .callback(() -> {
                        new AutonomousShootCommand().schedule();
                    })
                    //GATE STEAL
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, 19),
                            new Point2d(8, 45),
                            new Point2d(5, 57)
                    }, 5000)
                    .waitMilliseconds(3000)
                    //HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(5, 57),
                            new Point2d(0, 50),
                            new Point2d(-16, 19)
                    }, 5000)
                    //SHOOT
                    .callback(() -> {
                        new AutonomousShootCommand().schedule();
                    })
                    // INTAKE CLOSE SET
                    //.addTurnTo(70, 2000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, 19),
                            //new Point2d(0,30),
                            //new Point2d(6.5, 48)
                            new Point2d(13, 49)
                    }, 2000)
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(6.5, 54),   // was (12.5, 46)
                            new Point2d(-16, 19)    // was (-10, 17)
                    }, 2000)
                    .addTurnTo(-45,1000)
                    .waitMilliseconds(500)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
                    })
                    .waitMilliseconds(1000)

                    // SHOOT SECOND SET
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(200)
                    .addTurnTo(-35,500)

                    /*// PICKUP THIRD SET
                    .addTurnTo(45, 1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, -19),   // was (-10, 17)
                            new Point2d(39, -46)     // was (37, 46)
                    }, 1100)
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(velo, veloMiddle, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(36, -48),
//                            new Point2d(0,25),
                            new Point2d(-16, -19)    // was (-10, 17)
                    }, 2000)
//                    .waitMilliseconds(1000)
                    .addTurnTo(-45,1000)
                    .waitMilliseconds(500)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngle).schedule();
                    })
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootAntiJamCommand(),
                                new WaitCommand(300),
                                new IntakeSpitCommand()
                        ).schedule();
                    })*/
                    .waitMilliseconds(1000)
                    //LEAVE
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
        sixWheel.reset();
        sixWheel.write();
        intake.resetEncoder();
        intake.write();
        alreadySignalledPattern = false;
    }

    public void onStart() {
        shooter.shootWithVelocity(1120); // orig 850 before switching to triple shot
        turret.setAngle(5);
        llTagDetector.switchToMotif();
        sixWheel.setPosition(new Pose2d(-49, -54, Math.toRadians(51.529)));
        currentPath = new TestingPath().build().start();
        Globals.setAlliance(Alliance.BLUE);

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
    }
}
