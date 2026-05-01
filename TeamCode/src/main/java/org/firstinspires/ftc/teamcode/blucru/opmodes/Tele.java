package org.firstinspires.ftc.teamcode.blucru.opmodes;

import android.provider.Settings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.seattlesolvers.solverslib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ResetForIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ReturnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.UnshootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.RetransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.ShootReverseWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretFrom180To0TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretTo180DegreeTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.TurnTurretToPosCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.tilt.tiltCommands.ResetTiltCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.tilt.tiltCommands.TiltCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

import java.util.LinkedList;

@TeleOp(name = "Tele", group = "norm")

public class Tele extends BluLinearOpMode{

    StateMachine sm;
    public boolean turreting = true;

    public static boolean dashfield = true;
    public boolean autoTagUpdating = false;
    private boolean usingBrushlands = false;
    public int rumbleDur = 200;
    public int shot = 0;
    public boolean targetHit = false;
    boolean tiltActive = false;

    public static double ELEVATOR_FULL_DELAY_MS = 50;
    public static int INTAKE_REVERSE_MS = 300;
    public static double RELOC_MAX_DIST_IN = 36.0;
    public static double RELOC_BLEND_ALPHA = 0.15;
    public static double RELOC_DEADBAND_IN = 0.5;
    private final ElapsedTime elevatorFullTimer = new ElapsedTime();

    private static final int MAX_TRAIL_SIZE = 200;
    private final LinkedList<double[]> poseTrail = new LinkedList<>();

    public enum State{
        IDLE,
        INTAKING,
        INTAKING_ELEVATED,
        DRIVING_TO_SHOOT,
        INTAKING_FROM_ABOVE
    }

    TelemetryPacket packet = new TelemetryPacket();
    Canvas overlay = packet.fieldOverlay();

    @Override
    public void initialize(){
        reportTelemetry = true;
        manageTelemetry = true;
        tiltActive = false;
        robot.clear();
        addSixWheel();
        addTurret();
        robot.addTurretCam();
        //addLLTagDetector();
        addIntake();
        addElevator();
        addTransfer();
        addShooter();
        addTilt();
        CommandScheduler.getInstance().reset();
        enableDash();

        sm = new StateMachineBuilder()

                .state(State.IDLE)
                .transition(() -> driver1.pressedLeftTrigger(), State.INTAKING, () ->{
                    gamepad1.rumble(rumbleDur);
                    new ResetForIntakeCommand().schedule();
                })
                .transition(() -> driver1.pressedRightBumper(), State.DRIVING_TO_SHOOT, () ->{
                    gamepad1.rumble(rumbleDur);
                    new UnshootCommand().schedule();
                })
                .transition(() -> driver1.pressedA(), State.INTAKING_FROM_ABOVE, () ->{
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
                    gamepad2.rumble(rumbleDur);
                    shot = 0;
                    new TransferCommand(turreting).schedule();
                })
                .transition(() -> gamepad1.left_trigger > 0.2 && gamepad1.right_trigger > 0.2, State.INTAKING_ELEVATED, () ->{
                    gamepad1.rumble(rumbleDur);
                    new ElevatorUpCommand().schedule();
                })
                .state(State.INTAKING)
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.2 || gamepad2.right_trigger > 0.2){
                        intake.setIn();
                    } else if (gamepad1.right_trigger > 0.2 || gamepad2.left_trigger > 0.2){
                        intake.setOut();
                    } else {
                        intake.setPID();
                    }
                })
                .transition(() -> gamepad1.left_trigger > 0.2 && gamepad1.right_trigger > 0.2, State.INTAKING_ELEVATED, () ->{
                    gamepad1.rumble(rumbleDur);
                    new ElevatorUpCommand().schedule();
                })
                .transition(() -> driver1.pressedA(), State.INTAKING_FROM_ABOVE, () ->{
                    gamepad1.rumble(rumbleDur);
                    new SequentialCommandGroup(
                            new IntakeStopCommand(),
                            new ElevatorMiddleCommand(),
                            new WaitCommand(200),
                            new AllTransferMiddleCommand(),
                            new SetHoodAngleCommand(26),
                            new ShootReverseWithVelocityCommand(350)
                    ).schedule();
                })
//                .transition(() -> driver1.pressedDpadDown(), State.INTAKING, () ->{
//                    gamepad1.rumble(rumbleDur);
//                    if (Math.abs(turret.getAngle()) < 5){
//                        new MoveTurretTo180DegreeTransferCommand().schedule();
//                    } else {
//                        new MoveTurretFrom180To0TransferCommand().schedule();
//                    }
//                })
                .transition(() -> driver1.pressedLeftBumper() || driver2.pressedRightBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    gamepad2.rumble(rumbleDur);
                    shot = 0;
                    new TransferCommand(turreting).schedule();
                })
                .transition(this::elevatorStableFull, State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot = 0;
                    new SequentialCommandGroup(
                            new InstantCommand(() -> intake.setOut()),
                            new WaitCommand(INTAKE_REVERSE_MS),
                            new InstantCommand(() -> intake.setPID()),
                            new TransferCommand(turreting)
                    ).schedule();
                })

                .state(State.INTAKING_ELEVATED)
                .loop(() -> {
                    intake.setIn();
                })
                .transition(() -> gamepad1.right_trigger < 0.2, State.INTAKING, () ->{
                    new ElevatorDownCommand().schedule();
                })
//                .transition(() -> driver1.pressedDpadDown(), State.INTAKING, () ->{
//                    gamepad1.rumble(rumbleDur);
//                    if (Math.abs(turret.getAngle()) < 5){
//                        new SequentialCommandGroup(
//                                new TurnTurretToPosCommand(-90),
//                                new WaitCommand(200),
//                                new MoveTurretTo180DegreeTransferCommand()
//                        ).schedule();
//                    } else {
//                        new MoveTurretFrom180To0TransferCommand().schedule();
//                    }
//                })
                .transition(() -> driver1.pressedLeftBumper() || driver2.pressedRightBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    gamepad2.rumble(rumbleDur);
                    shot = 0;
                    new TransferCommand(turreting).schedule();
                })
                .transition(this::elevatorStableFull, State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot = 0;
                    new SequentialCommandGroup(
                            new InstantCommand(() -> intake.setOut()),
                            new WaitCommand(INTAKE_REVERSE_MS),
                            new InstantCommand(() -> intake.setPID()),
                            new TransferCommand(turreting)
                    ).schedule();
                })
                .state(State.DRIVING_TO_SHOOT)
                .transition(() -> driver1.pressedRightBumper(), State.INTAKING, () -> {
                    gamepad1.rumble(rumbleDur);
                    targetHit =  false;
                    new ConditionalCommand(
                            new ShootBallsCommand(),
                            new ReturnCommand(),
                            () -> (shot == 0)
                    ).schedule();
                })
                .transition(() -> driver1.pressedDpadLeft(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ConditionalCommand(
                            new ShootBallsCommand(),
                            new LeftTransferUpCommand(),
                            () -> (shot == 2)
                    ).schedule();
                    shot+=1;
                })
                .transition(() -> driver1.pressedDpadUp(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ConditionalCommand(
                            new ShootBallsCommand(),
                            new MiddleTransferUpCommand(),
                            () -> (shot == 2)
                    ).schedule();
                    shot+=1;
                })
                .transition(() -> driver1.pressedDpadRight(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ConditionalCommand(
                            new ShootBallsCommand(),
                            new RightTransferUpCommand(),
                            () -> (shot == 2)
                    ).schedule();
                    shot+=1;
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
                /*.transition(() -> driver1.pressedRightBumper(), State.IDLE, () -> {
                    gamepad1.rumble(rumbleDur);
                    robot.idleRobot();
                    new IdleCommand().schedule();
                })*/
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

    public void initializePeriodic(){
        telemetry.addLine( "TURN ON INTAKE: HOLD LEFT TRIGGER");
        telemetry.addLine( "EJECT: HOLD RIGHT TRIGGER");
        telemetry.addLine("TRANSFER: LEFT BUMPER");
        telemetry.addLine("SHOOT: RIGHT BUMPER");
        telemetry.addLine("INTAKE FROM ABOVE: X");
        telemetry.addLine("DRIVER 2 CONTROLS: TRANSFER: RIGHT BUMPER");


    }

    public void onStart(){
        new ElevatorDownCommand().schedule();
    }

    public void periodic(){
        //llTagDetector.read();
        sm.update();

        //Shooter
        if(driver2.pressedTouchpad()){
            if (Globals.alliance == Alliance.BLUE) {
                gamepad2.rumble(350);
                shooter.redAlliance = true;
                Globals.setAlliance(Alliance.RED);
            } else {
                gamepad2.rumble(350);
                shooter.redAlliance = false;
                Globals.setAlliance(Alliance.BLUE);
            }
        }

        if(shooter.targetHit() == true && targetHit == false){
            gamepad1.rumble(500);
            targetHit = true;
        }

        //Drivetrain
        sixWheel.teleDrive(gamepad1, 0.0001);
//        if (driver2.pressedB() && !driver2.pressedOptions()){
//            sixWheel.setPosition(llTagDetector.getLLBotPose());
//        }

        if (driver2.pressedA() && !driver2.pressedOptions() && Globals.alliance == Alliance.RED) {
            gamepad2.rumble(1000);
            sixWheel.setPosition(new Pose2d(62, -59, Math.toRadians(-90)));
        }
        if (driver2.pressedA() && !driver2.pressedOptions() && Globals.alliance == Alliance.BLUE) {
            gamepad2.rumble(1000);
            sixWheel.setPosition(new Pose2d(62, 59, Math.toRadians(90)));
        }

        /**if (driver1.pressedRightTrigger()){
         if (sixWheel.getDrivePower() == 0.5){
         sixWheel.setDrivePower(1);
         } else {
         sixWheel.setDrivePower(0.5);
         }
         }*/

        //Turret

        //auto-aim
        if (driver2.pressedDpadUp() && turreting) {
            gamepad2.rumble(1000);
            turreting = false;
        } else if (driver2.pressedDpadUp() && !turreting) {
            gamepad2.rumble(1000);
            turreting = true;
        }

        //manual
        if (driver2.pressedShare()){
            gamepad2.rumble(500);
            turret.resetEncoder();
            turret.toggleManual();
        }

        if (turret.isManual()){
            turret.setPower(gamepad2.right_stick_x * 0.3);
        }

        //modify targets
        if (driver2.pressedDpadLeft()){
            gamepad2.rumble(200);
            if(Globals.alliance == Alliance.RED) {
                Globals.turretTargetRedY -= 5;
            }else {
                Globals.turretTargetBlueY -= 5;
            }
        }
        if (driver2.pressedDpadRight()){
            gamepad2.rumble(200);
            if(Globals.alliance == Alliance.RED) {
                Globals.turretTargetRedY += 5;
            }else {
                Globals.turretTargetBlueY += 5;
            }
        }

        if (driver2.pressedX()){
            gamepad2.rumble(200);
            if(Globals.alliance == Alliance.RED) {
                Globals.shootingGoalRPose = new Vector2d(Globals.shootingGoalRPose.getX()+2, Globals.shootingGoalRPose.getY()-2);
            }else {
                Globals.shootingGoalLPose = new Vector2d(Globals.shootingGoalLPose.getX()+2, Globals.shootingGoalLPose.getY()+2);
            }
        }
        if (driver2.pressedB() && !gamepad2.options){
            gamepad2.rumble(200);
            if(Globals.alliance == Alliance.RED) {
                Globals.shootingGoalRPose = new Vector2d(Globals.shootingGoalRPose.getX()-2, Globals.shootingGoalRPose.getY()+2);
            }else {
                Globals.shootingGoalLPose = new Vector2d(Globals.shootingGoalLPose.getX()-2, Globals.shootingGoalLPose.getY()-2);
            }
        }


        if (driver1.pressedY()){
            telemetry.addLine("offset increasing");
            Intake.offset += 100;
            Intake.offset = ((Intake.offset + 1000) % 2000 + 2000) % 2000 - 1000;
        }

        //manual heading update
        if (driver1.pressedA()){
            gamepad2.rumble(200);
            sixWheel.setHeading(0);
            if(Globals.alliance == Alliance.RED) {
                Globals.shootingGoalRPose = new Vector2d(Globals.OGshootingGoalRPose.getX(), Globals.OGshootingGoalRPose.getY());
                Globals.turretTargetRedY = Globals.OGturretTargetRedY;
                Globals.turretTargetRedX = Globals.OGturretTargetRedX;
            }else {
                Globals.shootingGoalLPose = new Vector2d(Globals.OGshootingGoalLPose.getX(), Globals.OGshootingGoalLPose.getY());
                Globals.turretTargetBlueY = Globals.OGturretTargetBlueY;
                Globals.turretTargetBlueX = Globals.OGturretTargetBlueX;
            }
        }
       // <----------- RELOCALIZIATION :))))) ALEX TRIED HIS BEST ----------->
        // Toggle (driver1 Y) gates reloc; defaults off so it can't move the turret
        // aim until you opt in. Heading reloc removed -- only XY is corrected, so
        // the turret continues to lean entirely on odometry/IMU heading.
        Globals.telemetry.addData("Reloc enabled", autoTagUpdating);
        if (!autoTagUpdating) {
            // skip
        } else if (!Robot.getInstance().turretCam.computedBotposeThisLoop()) {
            Globals.telemetry.addData("Reloc", "no botpose (alliance=%s)", Globals.alliance);
        } else {
            Pose2d tagPose = Robot.getInstance().turretCam.getBotPosePoseHistory();
            Pose2d rawBotpose = Robot.getInstance().turretCam.getBotpose();
            if (tagPose != null && rawBotpose != null) {
                Pose2d odoPose = sixWheel.getPos();
                double dx = tagPose.getX() - odoPose.getX();
                double dy = tagPose.getY() - odoPose.getY();
                double relocDist = Math.sqrt(dx * dx + dy * dy);

                Globals.telemetry.addData("Reloc dist (in)", "%.1f", relocDist);

                if (relocDist < RELOC_MAX_DIST_IN) {
                    if (relocDist > RELOC_DEADBAND_IN) {
                        // Blend toward observation instead of snapping. With alpha ~ 0.15
                        // the correction converges in a few frames without fighting auto-aim.
                        double newX = odoPose.getX() + RELOC_BLEND_ALPHA * dx;
                        double newY = odoPose.getY() + RELOC_BLEND_ALPHA * dy;
                        sixWheel.setXY(new Vector2d(newX, newY));
                    }
                    Globals.telemetry.addLine("Re-loc yay!");
                } else {
                    Globals.telemetry.addLine("Bad re-loc :( ");
                }
            }
        }

        if (gamepad2.left_bumper){
            tilt.goDown();
        } else {
            tilt.reset();
        }

        if (driver2.pressedLeftStickButton()){
                usingBrushlands = !usingBrushlands;
        }

        //toggle
        if (driver2.pressedRightStickButton()){
            turret.useShotLineOffset = !turret.useShotLineOffset;
        }
//        if (driver2.pressedDpadLeft()){
//            llTagDetector.switchToPosition();
//        }

        // Add current position to breadcrumb trail
        Pose2d currentPos = Robot.getInstance().sixWheelDrivetrain.getPos();
        poseTrail.addLast(new double[]{currentPos.getX(), currentPos.getY()});
        while (poseTrail.size() > MAX_TRAIL_SIZE) {
            poseTrail.removeFirst();
        }

        if(dashfield) {
        // Draw breadcrumb trail
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
            overlay.strokeLine(currentPos.getX() + cos * 4.5, currentPos.getY() + sin * 4.5, currentPos.getX() + cos * 9, currentPos.getY() + sin * 9);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    public void telemetry(){
        Pose2d pose = sixWheel.getPos();

        telemetry.addLine("======== ROBOT ========");
        telemetry.addData("State", sm.getState());
        telemetry.addData("Alliance", Globals.alliance);
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getH()));
        telemetry.addData("Voltage", "%.2f V", Globals.voltage);
        telemetry.addData("Pose History", robot.positionHistory.size());
        telemetry.addData("Subsystems", robot.getAmountOfSubsystems());
        telemetry.addData("Auto Tag Update", autoTagUpdating);
        telemetry.addData("Brushlands", usingBrushlands);
        telemetry.addData("Turreting", turreting);
        telemetry.addData("Shot", shot);
        telemetry.addData("Target Hit", targetHit);

        section("DRIVETRAIN");
        sixWheel.telemetry(telemetry);

        section("INTAKE");
        intake.telemetry(telemetry);

        section("ELEVATOR");
        elevator.telemetry(telemetry);

        section("TRANSFER");
        transfer.telemetry(telemetry);

        section("SHOOTER");
        shooter.telemetry(telemetry);

        section("TURRET");
        turret.telemetry(telemetry);

        section("TILT");
        tilt.telemetry(telemetry);

        section("TURRET CAM");
        robot.turretCam.telemetry(telemetry);
    }

    private void section(String name) {
        telemetry.addLine("");
        telemetry.addLine("---- " + name + " ----");
    }

    private boolean elevatorStableFull() {
        if (!elevator.isFull()) {
            elevatorFullTimer.reset();
            return false;
        }
        return elevatorFullTimer.milliseconds() > ELEVATOR_FULL_DELAY_MS && usingBrushlands;
    }

}
