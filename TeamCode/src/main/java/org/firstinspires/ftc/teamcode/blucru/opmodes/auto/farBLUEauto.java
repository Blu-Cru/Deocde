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
public class farBLUEauto extends BluLinearOpMode {
    double turretAnglePreload = 102; //ROBOT CENTRIC: 102  FIELD CENTRIC: 168
    double turretAngleRest = 156; //Field centric angle increase = towards obelisk decrease = towards gate
    double shootVeloLeft = 1440;
    double shootVeloMiddle = 1440;
    double shootVeloRight = 1440;
    Point2d shootingPoint = new Point2d(45, -9);

    double leftHood = 49;
    double middleHood = 45;
    double rightHood = 49;

    // shifted +3 in Y to keep the same path relative to new start pose
    double pickupWallY = -62;

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();
            this.addPurePursuitPath(new Point2d[]{
                            new Point2d(63, -7),
                            new Point2d(63, -8)
                    }, 100)
                    .waitMilliseconds(1500) //TODO: TUNE
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(63, -8),
                            // INTAKE FIRST SET
                            new Point2d(39, -47)
                    }, 2000)
                    .waitMilliseconds(200)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new IntakeStopCommand(),
                                new WaitCommand(100),
                                new IntakeSpitCommand(),
                                new WaitCommand(300),
                                new ElevatorUpCommand(),
                                new WaitCommand(300),
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
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(40, -40),
                            new Point2d(45, -25),
                            // SHOOT FIRST SET
                            shootingPoint
                    }, 2000)
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
                    .waitMilliseconds(300)

                    .addPurePursuitPath(new Point2d[]{
                            shootingPoint,
                            // INTAKE SECOND SET
                            new Point2d(61, -45),
                            new Point2d(62.5,-55),

                            new Point2d(63, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(1000)
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
                            shootingPoint
                    }, 3000)
                    .waitMilliseconds(1100)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })

                    .waitMilliseconds(700)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)

                    .addPurePursuitPath(new Point2d[]{
                            shootingPoint,
                            // INTAKE THIRD SET
                            new Point2d(61, -45),
                            new Point2d(62.5,-55),

                            new Point2d(62, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
//                                new WaitCommand(300),

                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutoLongSpitTransferCommand(leftHood, middleHood, rightHood)
//                                new WaitCommand(4000),
//                                new TurnTurretToPosCommand(102)
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(62, pickupWallY),
                            // SHOOT THIRD SET
                            shootingPoint
                    }, 3000)
                    .waitMilliseconds(1100)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })
                    .waitMilliseconds(700)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)

                    .addPurePursuitPath(new Point2d[]{
                            shootingPoint,
                            // INTAKE FOURTH SET
                            new Point2d(61, -45),
                            new Point2d(62.5,-55),
                            new Point2d(62, pickupWallY)
                    }, 1200)
                    .waitMilliseconds(1000)
                    .callback(() -> {
                        new SequentialCommandGroup(
//                                new WaitCommand(300),
                                new SetShooterVelocityIndependentCommand(shootVeloLeft, shootVeloMiddle, shootVeloRight),
                                new AutoLongSpitTransferCommand(leftHood, middleHood, rightHood)
//                                new WaitCommand(2000),
//                                new TurnTurretToPosCommand(102)
                        ).schedule();
                    })
                    .addPurePursuitPath(new Point2d[]{
                            new Point2d(62, pickupWallY),
                            // SHOOT FOURTH SET
                            shootingPoint
                    }, 3000)
                    .waitMilliseconds(1100)
                    .callback(()->{
                        new TurnTurretToPosFieldCentricCommand(turretAngleRest).schedule();
                    })
                    .waitMilliseconds(700)
                    .callback(() -> {
                        new SequentialCommandGroup(
                                new AutonomousShootCommand()
                        ).schedule();
                    })
                    .waitMilliseconds(300)
//                    .waitMilliseconds(300)
                    .addPurePursuitPath(new Point2d[]{
                            shootingPoint,
                            new Point2d(58, -50)
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
        shooter.shootWithVelocityIndependent(1510, 1520, 1490);
        turret.setAngle(-117);
        currentPath = new TestingPath().build().start();
        sixWheel.setPosition(new Pose2d(63, -7, Math.toRadians(-90)));
        Globals.setAlliance(Alliance.BLUE);
    }

    public void periodic() {
//        llTagDetector.read();
        currentPath.run();
    }
}
