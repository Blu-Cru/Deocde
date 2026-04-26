package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ResetForIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.RetransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ReturnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TimedWaitUntilCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.UnshootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.GoalSweepShootAllCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.ShootReverseWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretFrom180To0TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretTo180DegreeTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.tilt.tiltCommands.TiltCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

import java.util.LinkedList;

@TeleOp(name = "Sweep Shooting Test", group = "test")
public class SweepShootingTest extends BluLinearOpMode {

    StateMachine sm;
    public boolean turreting = true;

    public static boolean dashfield = true;
    public boolean autoTagUpdating = true;
    public int rumbleDur = 200;
    public int shot = 0;
    public boolean targetHit = false;

    public static double RIGHT_SHOT_TOLERANCE_DEG = 1.0;

    private static final int MAX_TRAIL_SIZE = 200;
    private final LinkedList<double[]> poseTrail = new LinkedList<>();

    public enum State {
        IDLE,
        INTAKING,
        INTAKING_ELEVATED,
        DRIVING_TO_SHOOT,
        INTAKING_FROM_ABOVE
    }

    TelemetryPacket packet = new TelemetryPacket();
    Canvas overlay = packet.fieldOverlay();

    @Override
    public void initialize() {
        reportTelemetry = true;
        robot.clear();
        addSixWheel();
        robot.addTurretCam();
        addIntake();
        addElevator();
        addTransfer();
        addShooter();
        addTurret();
        addTilt();
        CommandScheduler.getInstance().reset();
        enableDash();

        sm = new StateMachineBuilder()

                .state(State.IDLE)
                .transition(() -> driver1.pressedLeftTrigger(), State.INTAKING, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ResetForIntakeCommand().schedule();
                })
                .transition(() -> driver1.pressedRightBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    new UnshootCommand().schedule();
                })
                .transition(() -> driver1.pressedA(), State.INTAKING_FROM_ABOVE, () -> {
                    gamepad1.rumble(rumbleDur);
                    new SequentialCommandGroup(
                            new ElevatorMiddleCommand(),
                            new WaitCommand(200),
                            new AllTransferMiddleCommand(),
                            new SetHoodAngleCommand(26),
                            new ShootReverseWithVelocityCommand(350)
                    ).schedule();
                })
                .transition(() -> driver1.pressedLeftBumper() || driver2.pressedRightBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot = 0;
                    new TransferCommand(turreting).schedule();
                })
                .transition(() -> gamepad1.left_trigger > 0.2 && gamepad1.right_trigger > 0.2, State.INTAKING_ELEVATED, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ElevatorUpCommand().schedule();
                })

                .state(State.INTAKING)
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.2) {
                        intake.setIn();
                    } else if (gamepad1.right_trigger > 0.2) {
                        intake.setOut();
                    } else {
                        intake.setPID();
                    }
                })
                .transition(() -> gamepad1.left_trigger > 0.2 && gamepad1.right_trigger > 0.2, State.INTAKING_ELEVATED, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ElevatorUpCommand().schedule();
                })
                .transition(() -> driver1.pressedDpadDown(), State.INTAKING, () -> {
                    gamepad1.rumble(rumbleDur);
                    if (Math.abs(turret.getAngle()) < 5) {
                        new MoveTurretTo180DegreeTransferCommand().schedule();
                    } else {
                        new MoveTurretFrom180To0TransferCommand().schedule();
                    }
                })
                .transition(() -> driver1.pressedLeftBumper() || driver2.pressedRightBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot = 0;
                    new TransferCommand(turreting).schedule();
                })

                .state(State.INTAKING_ELEVATED)
                .loop(() -> intake.setIn())
                .transition(() -> gamepad1.right_trigger < 0.2, State.INTAKING, () -> {
                    new ElevatorDownCommand().schedule();
                })
                .transition(() -> driver1.pressedDpadDown(), State.INTAKING, () -> {
                    gamepad1.rumble(rumbleDur);
                    if (Math.abs(turret.getAngle()) < -5) {
                        new MoveTurretTo180DegreeTransferCommand().schedule();
                    } else {
                        new MoveTurretFrom180To0TransferCommand().schedule();
                    }
                })
                .transition(() -> driver1.pressedLeftBumper() || driver2.pressedRightBumper() || elevator.isFull(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot = 0;
                    new TransferCommand(turreting).schedule();
                })

                .state(State.DRIVING_TO_SHOOT)
                .transition(() -> driver1.pressedRightBumper(), State.INTAKING, () -> {
                    gamepad1.rumble(rumbleDur);
                    targetHit = false;
                    new ConditionalCommand(
                            buildSweepShootAllCommand(),
                            new ReturnCommand(),
                            () -> (shot == 0 && sixWheel.getPos().getX() > 0)
                    ).schedule();
                })
                .transition(() -> driver1.pressedDpadLeft(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ConditionalCommand(
                            buildSweepShootAllCommand(),
                            buildSingleSweepShotCommand(Turret.GoalSweepStage.LEFT_SHOT),
                            () -> (shot == 2)
                    ).schedule();
                    shot += 1;
                })
                .transition(() -> driver1.pressedDpadUp(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ConditionalCommand(
                            buildSweepShootAllCommand(),
                            buildSingleSweepShotCommand(Turret.GoalSweepStage.MIDDLE_SHOT),
                            () -> (shot == 2)
                    ).schedule();
                    shot += 1;
                })
                .transition(() -> driver1.pressedDpadRight(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ConditionalCommand(
                            buildSweepShootAllCommand(),
                            buildSingleSweepShotCommand(Turret.GoalSweepStage.RIGHT_SHOT),
                            () -> (shot == 2)
                    ).schedule();
                    shot += 1;
                })
                .transition(() -> shot >= 3, State.INTAKING, () -> {
                    shot = 0;
                    targetHit = false;
                })
                .transition(() -> driver1.pressedLeftBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    targetHit = false;
                    new RetransferCommand(turreting).schedule();
                })

                .state(State.INTAKING_FROM_ABOVE)
                .transition(() -> driver1.pressedLeftBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot = 0;
                    new TransferCommand(turreting).schedule();
                })
                .build();

        sm.setState(State.IDLE);

        elevator.setUp();
        elevator.write();
        elevator.setDown();
        elevator.write();
        transfer.setAllDown();
        transfer.write();
        turret.setAngle(0);
        turret.write();

        sm.start();
    }

    @Override
    public void initializePeriodic() {
        telemetry.addLine("TURN ON INTAKE: HOLD LEFT TRIGGER");
        telemetry.addLine("EJECT: HOLD RIGHT TRIGGER");
        telemetry.addLine("TRANSFER: LEFT BUMPER");
        telemetry.addLine("SWEEP SHOOT ALL: RIGHT BUMPER");
        telemetry.addLine("SWEEP SINGLE SHOTS: DPAD LEFT / UP / RIGHT");
        telemetry.addLine("INTAKE FROM ABOVE: X");
        telemetry.addLine("DRIVER 2 CONTROLS: TRANSFER: RIGHT BUMPER");
    }

    @Override
    public void onStart() {
        new ElevatorDownCommand().schedule();
    }

    @Override
    public void periodic() {
        sm.update();

        if (driver2.pressedTouchpad()) {
            gamepad2.rumble(350);
            shooter.redAlliance = true;
            Globals.setAlliance(Alliance.RED);
        } else if (driver2.pressedRightTrigger()) {
            gamepad2.rumble(350);
            shooter.redAlliance = false;
            Globals.setAlliance(Alliance.BLUE);
        }

        if (shooter.targetHit() && !targetHit) {
            gamepad1.rumble(500);
            targetHit = true;
        }

        sixWheel.teleDrive(gamepad1, 0.0001);

        if (driver2.pressedA() && !driver2.pressedOptions()) {
            gamepad2.rumble(1000);
            sixWheel.setPosition(new Pose2d(0, 0, 0));
        }

        if (driver2.pressedLeftBumper() && turreting) {
            gamepad2.rumble(1000);
            turreting = false;
        } else if (driver2.pressedLeftBumper() && !turreting) {
            gamepad2.rumble(1000);
            turreting = true;
        }

        if (driver2.pressedShare()) {
            gamepad2.rumble(500);
            turret.resetEncoder();
            turret.toggleManual();
        }

        if (turret.isManual()) {
            turret.setPower(gamepad2.right_stick_x * 0.3);
        }

        if (driver2.pressedDpadLeft()) {
            gamepad2.rumble(200);
            if (Globals.alliance == Alliance.RED) {
                Globals.turretTargetRedY -= 5;
            } else {
                Globals.turretTargetBlueY -= 5;
            }
        }
        if (driver2.pressedDpadRight()) {
            gamepad2.rumble(200);
            if (Globals.alliance == Alliance.RED) {
                Globals.turretTargetRedY += 5;
            } else {
                Globals.turretTargetBlueY += 5;
            }
        }

        if (driver2.pressedX()) {
            gamepad2.rumble(200);
            if (Globals.alliance == Alliance.RED) {
                Globals.shootingGoalRPose = new Vector2d(
                        Globals.shootingGoalRPose.getX() + 2,
                        Globals.shootingGoalRPose.getY() - 2
                );
            } else {
                Globals.shootingGoalLPose = new Vector2d(
                        Globals.shootingGoalLPose.getX() + 2,
                        Globals.shootingGoalLPose.getY() + 2
                );
            }
        }
        if (driver2.pressedB() && !gamepad2.options) {
            gamepad2.rumble(200);
            if (Globals.alliance == Alliance.RED) {
                Globals.shootingGoalRPose = new Vector2d(
                        Globals.shootingGoalRPose.getX() - 2,
                        Globals.shootingGoalRPose.getY() + 2
                );
            } else {
                Globals.shootingGoalLPose = new Vector2d(
                        Globals.shootingGoalLPose.getX() - 2,
                        Globals.shootingGoalLPose.getY() - 2
                );
            }
        }

        if (driver1.pressedY()) {
            gamepad2.rumble(200);
            autoTagUpdating = !autoTagUpdating;
        }

        if (driver1.pressedA()) {
            gamepad2.rumble(200);
            sixWheel.setHeading(0);
            if (Globals.alliance == Alliance.RED) {
                Globals.shootingGoalRPose = new Vector2d(
                        Globals.OGshootingGoalRPose.getX(),
                        Globals.OGshootingGoalRPose.getY()
                );
                Globals.turretTargetRedY = Globals.OGturretTargetRedY;
                Globals.turretTargetRedX = Globals.OGturretTargetRedX;
            } else {
                Globals.shootingGoalLPose = new Vector2d(
                        Globals.OGshootingGoalLPose.getX(),
                        Globals.OGshootingGoalLPose.getY()
                );
                Globals.turretTargetBlueY = Globals.OGturretTargetBlueY;
                Globals.turretTargetBlueX = Globals.OGturretTargetBlueX;
            }
        }

        if (Robot.getInstance().turretCam.computedBotposeThisLoop()) {
            Pose2d tagPose = Robot.getInstance().turretCam.getBotPosePoseHistory();
            if (tagPose != null && Robot.getInstance().turretCam.getBotpose() != null) {
                double dx = tagPose.getX() - sixWheel.getPos().getX();
                double dy = tagPose.getY() - sixWheel.getPos().getY();
                double relocDist = Math.sqrt(dx * dx + dy * dy);

                if (relocDist < 15) {
                    sixWheel.setPosition(tagPose);
                    gamepad2.rumble(200);
                    Globals.telemetry.addLine("Re-loc yay!");
                } else {
                    gamepad2.rumble(500);
                    Globals.telemetry.addLine("Bad re-loc :( ");
                }
            }
        }

        if (driver2.pressedLeftTrigger()) {
            Intake.offset += 5;
            Intake.offset = ((Intake.offset + 90) % 180 + 180) % 180 - 90;
        }

        if (driver2.pressedLeftStickButton() && driver2.pressedRightStickButton()) {
            new TiltCommand().schedule();
        }

        Pose2d currentPos = Robot.getInstance().sixWheelDrivetrain.getPos();
        poseTrail.addLast(new double[]{currentPos.getX(), currentPos.getY()});
        while (poseTrail.size() > MAX_TRAIL_SIZE) {
            poseTrail.removeFirst();
        }

        if (dashfield) {
            overlay.setStroke("gray");
            overlay.setStrokeWidth(1);
            for (double[] point : poseTrail) {
                overlay.strokeCircle(point[0], point[1], 1.5);
            }
            overlay.setStrokeWidth(2);
            overlay.setStroke("blue");
            overlay.strokeCircle(currentPos.getX(), currentPos.getY(), 9);
            double cos = Math.cos(currentPos.getH());
            double sin = Math.sin(currentPos.getH());
            overlay.strokeLine(
                    currentPos.getX() + cos * 4.5,
                    currentPos.getY() + sin * 4.5,
                    currentPos.getX() + cos * 9,
                    currentPos.getY() + sin * 9
            );
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
        telemetry.addData("Pose History Length", robot.positionHistory.size());
    }

    private Command buildSingleSweepShotCommand(Turret.GoalSweepStage stage) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> turret.beginGoalSweep()),
                new InstantCommand(() -> turret.aimGoalSweepStage(stage)),
                new TimedWaitUntilCommand(
                        400,
                        () -> turret.isGoalSweepStageAtTarget()
                ),
                new WaitCommand(40),
                getTransferUpCommand(stage)
        );
    }

    private Command buildSweepShootAllCommand() {
        return new GoalSweepShootAllCommand(
                new WaitCommand(150),
                new CenterTurretCommand(),
                new IdleShooterCommand(),
                new WaitCommand(200),
                new AllTransferDownCommand(),
                new ResetForIntakeCommand()
        );
    }

    private Command getTransferUpCommand(Turret.GoalSweepStage stage) {
        switch (stage) {
            case LEFT_SHOT:
                return new LeftTransferUpCommand();
            case RIGHT_SHOT:
                return new RightTransferUpCommand();
            case MIDDLE_SHOT:
            default:
                return new MiddleTransferUpCommand();
        }
    }
}
