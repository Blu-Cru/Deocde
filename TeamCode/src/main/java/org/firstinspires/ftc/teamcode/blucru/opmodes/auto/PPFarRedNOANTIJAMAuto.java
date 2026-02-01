package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutoLongSpitTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousAntiJamTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootAntiJamCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous
public class PPFarRedNOANTIJAMAuto extends BluLinearOpMode {
    double turretAnglePreload = 102; //ROBOT CENTRIC: 102  FIELD CENTRIC: 168
    double turretAngleRest = 172; //Field centric angle decrease = towards obelisk increase = towards gate
    double shootVeloLeft = 1480; //TODO:tune
    double shootVeloMiddle = 1460;
    double shootVeloRight = 1480;
    double leftHood = 49;
    double middleHood = 45;
    double rightHood = 49;

    // shifted +3 in Y to keep the same path relative to new start pose
    double pickupWallY = 59;

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(63, 24),
                            new Point2d(63, 25)
                    }, 100)
                    .waitMilliseconds(1500) //TODO: TUNE
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(63, 25),
                            new Point2d(52, 41),
                            // INTAKE FIRST SET
                            new Point2d(40, 45) // 35,36 when measured with loco test
                    }, 5000)
                    .waitMilliseconds(200)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new IntakeStopCommand(),
                                new WaitCommand(200),
                                new IntakeSpitCommand(),
                                new WaitCommand(500),
                                new ElevatorUpCommand(),
                                new WaitCommand(400),
                                new ElevatorMiddleCommand(),
                                new WaitCommand(150),
                                new AllTransferMiddleCommand(),
                                new SetLeftHoodAngleCommand(leftHood),
                                new SetRightHoodAngleCommand(middleHood),
                                new SetMiddleHoodAngleCommand(rightHood),
//                    new WaitCommand(200), //TODO: TUNE WAIT
                                new IntakeStopCommand(),
                                new ParallelizeIntakeCommand()
//                                new WaitCommand(2000),
//                                new TurnTurretToPosCommand(102)
                        ).schedule();
                    })
                    .waitMilliseconds(200)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(42, 40),
                            // SHOOT FIRST SET
                            new Point2d(58, 24)
                    }, 2000)
                    .addTurnTo(90, 1000)
                    .waitMilliseconds(300)

                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(700)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(58, 22),
                            // INTAKE SECOND SET
                            new Point2d(58, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(600)
                    .callback(() -> {
                        new SequentialCommandGroup(

                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutoLongSpitTransferCommand(leftHood, middleHood, rightHood)
//                                new WaitCommand(4000),
//                                new TurnTurretToPosCommand(102)
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(58, pickupWallY),
                            // SHOOT SECOND SET
                            new Point2d(58, 22)
                    }, 3000)
                    .waitMilliseconds(2000)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })

                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(58, 22),
                            // INTAKE THIRD SET
                            new Point2d(58, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(600)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new WaitCommand(300),

                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutoLongSpitTransferCommand(leftHood, middleHood, rightHood)
//                                new WaitCommand(4000),
//                                new TurnTurretToPosCommand(102)
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(58, pickupWallY),
                            // SHOOT THIRD SET
                            new Point2d(58, 22)
                    }, 3000)
                    .waitMilliseconds(2000)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(200)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(58, 22),
                            // INTAKE FOURTH SET
                            new Point2d(58, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(600)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutoLongSpitTransferCommand(leftHood, middleHood, rightHood)
//                                new WaitCommand(2000),
//                                new TurnTurretToPosCommand(102)
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(58, pickupWallY),
                            // SHOOT FOURTH SET
                            new Point2d(58, 22)
                    }, 5000)
                    .waitMilliseconds(2000)

//                    .callback(()->{
//                    .waitMilliseconds(1000)
//                    .callback(() -> {
//                        new SequentialCommandGroup(
//                                new AutonomousShootCommand()
//                        ).schedule();
//                    })
//                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(58,22),
                            new Point2d(58, 50)
                    },5000)
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
        sixWheel.setPosition(new Pose2d(63, 24, Math.toRadians(90)));
        currentPath = new TestingPath().build().start();
        Globals.setAlliance(Alliance.RED);
    }

    public void periodic() {
        llTagDetector.read();
        currentPath.run();
    }
}
