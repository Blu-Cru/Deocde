package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootAntiJamCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

// @Autonomous
public class PPCloseBlueAuto extends BaseAuto {
    double turretAngle = 142; //field centric, decrease = more towards gate, increase = towards obelisk
    double velo = 1150;
    double leftHood;
    double middleHood ;
    double rightHood ;
    boolean alreadySignalledPattern;

    enum State {
        RUNNING
    }

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();

            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(-49, -54),   // was (-45, 52)
                            new Point2d(-16, -19)    // was (-10, 17)
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
                    .addTurnTo(-90, 5000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, -19),
                            new Point2d(-16,-33),
                            new Point2d(-6,-45),
                            new Point2d(-5, -57)
                    }, 2000)

                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(velo, velo, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-5, -54),   // was (-10, 50)
                            new Point2d(-16, -19)    // was (-10, 17)
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

                    // INTAKE SECOND SET
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, -19),
                            new Point2d(13, -49)
                    }, 2000)
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(velo, velo, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(6.5, -54),   // was (12.5, 46)
                            new Point2d(-16, -19)    // was (-10, 17)
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

                    // PICKUP THIRD SET
                    .addTurnTo(45, 1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, -19),   // was (-10, 17)
                            new Point2d(39, -46)     // was (37, 46)
                    }, 1100)
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(velo, velo, velo),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood)
                        ).schedule();
                    })
                    .waitMilliseconds(1000)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(36, -48),
                            new Point2d(-16, -19)    // was (-10, 17)
                    }, 2000)
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
                    })
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(-16, -19),    // was (37, 46)
                            new Point2d(0, -30)    // was (-10, 17)
                    }, 1300)
                    // SHOOT THIRD SET
                    .build();
        }
    }

    Path currentPath;

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-49, -54, Math.toRadians(51.529));
    }

    @Override
    public StateMachine buildStateMachine() {
        return new StateMachineBuilder()
                .state(State.RUNNING)
                .loop(() -> {
                    if (currentPath != null) {
                        currentPath.run();
                    }
                    if (!alreadySignalledPattern){
                        llTagDetector.read();
                        if (llTagDetector.detectedPattern()){
                            gamepad1.setLedColor(100,255,100, 1000);
                            alreadySignalledPattern = true;
                        }
                    }
                })
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
        alreadySignalledPattern = false;
        
        super.initialize();
    }

    @Override
    public void onStart() {
        shooter.shootWithVelocity(1120); // orig 850 before switching to triple shot
        turret.setAngle(5);
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
