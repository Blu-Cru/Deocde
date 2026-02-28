package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@Autonomous
public class RootNegativeOne extends BaseAuto {
    double turretAngle     = 142;
    double preAimTurretAngle = -90;
    double velo            = 1150;
    double veloMiddle      = 1150;
    double leftHood        = 30;
    double middleHood      = 30;
    double rightHood       = 30;

    enum State { RUNNING }

    public class AutoPath extends SixWheelPIDPathBuilder {
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
                    .callback(() -> {
                        new IntakeStartCommand().schedule();
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
        super.initialize();
    }

    @Override
    public void onStart() {
        shooter.shootWithVelocity(1150);
        turret.setAngle(5);
        sixWheel.setPosition(startPose);
        currentPath = new AutoPath().build().start();
        Globals.setAlliance(Alliance.BLUE);
        sm.setState(State.RUNNING);
        sm.start();
    }

    @Override
    public void periodic() {
        sm.update();
    }
}