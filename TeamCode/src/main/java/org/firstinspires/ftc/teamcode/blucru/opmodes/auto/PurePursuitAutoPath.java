package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import android.graphics.Point;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousShootCloseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.autonomousCommands.AutonomousTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.PIDPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.SixWheelPIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands.CenterTurretCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands.LockOnGoalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Point2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@Autonomous
public class PurePursuitAutoPath extends BluLinearOpMode {
    double closeTurretAngle = 30;

    public class TestingPath extends SixWheelPIDPathBuilder {

        public TestingPath() {
            super();
            this.addPurePursuitPath(new Point2d[] {
                    new Point2d(-45, 52),
                    new Point2d(-10, 35)
            }, 8000)
                    .waitMilliseconds(500)
                    // SHOOT PRELOAD
                    .addTurnTo(90, 4000)
                    // INTAKE FIRST SET
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-10, 35),
                            new Point2d(-10, 46)
                    }, 8000)
                    .waitMilliseconds(500)
                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-10, 46),
                            new Point2d(-10, 35)
                    }, 8000)
                    .waitMilliseconds(1000)
                    // SHOOT FIRST SET

                    // INTAKE SECOND SET
                    .addTurnTo(45, 4000)

                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(-10, 35),
                            new Point2d(12.5, 46)
                    }, 8000)
                    .waitMilliseconds(1000)

                    // HEAD BACK
                    .addPurePursuitPath(new Point2d[] {
                            new Point2d(12.5, 46),
                            new Point2d(-10, 35)
                    }, 8000)
                    .waitMilliseconds(500)
                    // SHOOT SECOND SET
                    .build();
        }
    }

    Path currentPath;

    public void initialize() {
        addSixWheel();
    }

    public void onStart() {
        sixWheel.setPosition(new Pose2d(-45, 52, Math.toRadians(127 + 180)));
        currentPath = new TestingPath().build().start();
    }

    public void periodic() {
        currentPath.run();
        if (currentPath instanceof PIDPath) {
            ((PIDPath) currentPath).telemetry();
        }
    }

}
