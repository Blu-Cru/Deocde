package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousAntiJamTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootAntiJamCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous
public class PPFarRedAuto extends BluLinearOpMode {
    double turretAnglePreload = 102; //ROBOT CENTRIC: 102  FIELD CENTRIC: 168
    double turretAngleRest = 170; //Field centric angle
    double shootVeloLeft = 1480; //TODO:tune
    double shootVeloMiddle = 1460;
    double shootVeloRight = 1480;
    double leftHood = 49;
    double middleHood = 45;
    double rightHood = 49;

    // shifted +3 in Y to keep the same path relative to new start pose
    double pickupWallY = 60;

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(63, 25),
                            new Point2d(63, 26)
                    }, 100)
                    .waitMilliseconds(1500) //TODO: TUNE
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(63, 26),
                            new Point2d(52, 42),
                            // INTAKE FIRST SET
                            new Point2d(40, 46) // 35,36 when measured with loco test
                    }, 5000)
                    .waitMilliseconds(400)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutonomousTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(300),
                                new TurnTurretToPosFieldCentricCommand(turretAngleRest, 90)
                        ).schedule();
                    })
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(42, 41),
                            // SHOOT FIRST SET
                            new Point2d(60, 23)
                    }, 2000)
                    .addTurnTo(90, 1000)
                    .waitMilliseconds(400)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootAntiJamCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(700)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(60, 23),
                            // INTAKE SECOND SET
                            new Point2d(62, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(300)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutonomousAntiJamTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(400),
                                new TurnTurretToPosFieldCentricCommand(turretAngleRest,90)
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(62, pickupWallY),
                            // SHOOT SECOND SET
                            new Point2d(61, 23)
                    }, 3000)

                    .waitMilliseconds(3100)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootAntiJamCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(61, 23),
                            // INTAKE THIRD SET
                            new Point2d(62, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(300)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutonomousAntiJamTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(400),
                                new TurnTurretToPosFieldCentricCommand(turretAngleRest,90)
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(62, pickupWallY),
                            // SHOOT THIRD SET
                            new Point2d(61, 23)
                    }, 3000)

                    .waitMilliseconds(3100)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootAntiJamCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(200)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(61, 23),
                            // INTAKE FOURTH SET
                            new Point2d(62, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(300)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutonomousAntiJamTransferCommand(leftHood, middleHood, rightHood),
                                new WaitCommand(400),
                                new TurnTurretToPosFieldCentricCommand(turretAngleRest,90)
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(62, pickupWallY),
                            // SHOOT FOURTH SET
                            new Point2d(61, 23)
                    }, 5000)

                    .waitMilliseconds(3100)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootAntiJamCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(1500)
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
        shooter.setHoodAngleIndependent(leftHood, middleHood, rightHood);
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
    }

    public void onStart() {
        shooter.shootWithVelocityIndependent(1500, 1480, 1500);
        turret.setAngle(102);
//        turret.setFieldCentricPosition(turretAngle, Math.toDegrees(Robot.getInstance().sixWheelDrivetrain.getPos().getH()), true);
        sixWheel.setPosition(new Pose2d(63, 25, Math.toRadians(90)));
        currentPath = new TestingPath().build().start();
    }

    public void periodic() {
        currentPath.run();
    }
}
