package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutoLongSpitTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.IdleShooterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetShooterVelocityIndependentCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.turretCommands.TurnTurretToPosFieldCentricCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous
public class farBLUEauto extends BluLinearOpMode {
    double turretAnglePreload = 102; //ROBOT CENTRIC: 102  FIELD CENTRIC: 168
    double turretAngleRest = 161.5; //Field centric angle increase = towards obelisk decrease = towards gate
    double shootVeloLeft = 1460; //TODO:tune
    double shootVeloMiddle = 1490;
    double shootVeloRight = 1470;
    double leftHood = 49;
    double middleHood = 45;
    double rightHood = 49;

    double pickupWallY = -59;

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(63, -24),
                            new Point2d(63, -25)
                    }, 100)
                    .waitMilliseconds(1500) //TODO: TUNE
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(63, -25),
                            new Point2d(52, -41),
                            // INTAKE FIRST SET
                            new Point2d(40, -45) // 35,36 when measured with loco test
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
                            new Point2d(42, -40),
                            // SHOOT FIRST SET
                            new Point2d(60, -22)
                    }, 2000)
                    .addTurnTo(-90, 1000)
                    .waitMilliseconds(300)

                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })
                    .waitMilliseconds(1300)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(700)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(60, -22),
                            // INTAKE SECOND SET
                            new Point2d(63, pickupWallY)
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
                            new Point2d(63, pickupWallY),
                            // SHOOT SECOND SET
                            new Point2d(61, -22)
                    }, 3000)
                    .waitMilliseconds(1600)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })

                    .waitMilliseconds(1300)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(61, -22),
                            // INTAKE THIRD SET
                            new Point2d(62, pickupWallY)
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
                            new Point2d(62, pickupWallY),
                            // SHOOT THIRD SET
                            new Point2d(61, -22)
                    }, 3000)
                    .waitMilliseconds(1600)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })
                    .waitMilliseconds(1300)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(2500)

                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(61, -22),
                            // INTAKE FOURTH SET
                            new Point2d(62, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(600)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutoLongSpitTransferCommand(leftHood, middleHood, rightHood)
//                                new WaitCommand(2000),
//                                new TurnTurretToPosCommand(102)
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(62, pickupWallY),
                            // SHOOT FOURTH SET
                            new Point2d(61, -22)
                    }, 5000)
                    .waitMilliseconds(1000)

                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })
                    .waitMilliseconds(1300)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AllTransferUpCommand(),
                                new WaitCommand(300),
                                new IdleShooterCommand(),
                                new CenterTurretCommand(),
                                new WaitCommand(400),
                                new ParallelizeIntakeCommand(),
                                new ElevatorMiddleCommand(),
                                new AllTransferDownCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(61, -22),
                            new Point2d(60, -50)
                    }, 5000)
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
        shooter.shootWithVelocityIndependent(1490, 1480, 1500);
        turret.setAngle(-107);
//        turret.setFieldCentricPosition(turretAngle, Math.toDegrees(Robot.getInstance().sixWheelDrivetrain.getPos().getH()), true);
        sixWheel.setPosition(new Pose2d(63, -24, Math.toRadians(-90)));
        currentPath = new TestingPath().build().start();
        Globals.setAlliance(Alliance.BLUE);
    }

    public void periodic() {
        llTagDetector.read();
        currentPath.run();
    }
}
