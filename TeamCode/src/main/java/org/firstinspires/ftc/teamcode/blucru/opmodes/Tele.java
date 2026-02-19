package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ResetForIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ReturnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.UnshootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.RetransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.shooterCommands.ShootReverseWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;
import org.firstinspires.ftc.teamcode.blucru.common.util.Vector2d;

@TeleOp (group = "a")

public class Tele extends BluLinearOpMode{

    StateMachine sm;
    public boolean turreting = true;
    public boolean autoTagUpdating = true;
    public int rumbleDur = 200;
    public int shot = 0;
    public boolean targetHit = false;

    public enum State{
        IDLE,
        INTAKING,
        DRIVING_TO_SHOOT,
        INTAKING_FROM_ABOVE
    }

    @Override
    public void initialize(){
        reportTelemetry = true;
        robot.clear();
        robot.addTurretCam();
        addLLTagDetector();
        addSixWheel();
        addIntake();
        addElevator();
        addTransfer();
        addShooter();
        addTurret();
        CommandScheduler.getInstance().reset();

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
                        new SetLeftHoodAngleCommand(26),
                        new SetMiddleHoodAngleCommand(26),
                        new SetRightHoodAngleCommand(26),
                        new ShootReverseWithVelocityCommand(350)
                    ).schedule();
                })

                .state(State.INTAKING)
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.2){
                        intake.setIn();
                    } else if (gamepad1.right_trigger > 0.2){
                        intake.setOut();
                    } else {
                        intake.setPID();
                    }
                })
                /*.transition(() -> driver1.pressedRightBumper(), State.IDLE, () -> {
                    gamepad1.rumble(rumbleDur);
                    robot.idleRobot();
                    new IdleCommand().schedule();
                })*/
                .transition(() -> driver1.pressedLeftBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot = 0;
                    new TransferCommand(turreting).schedule();
                })

                .state(State.DRIVING_TO_SHOOT)
                .transition(() -> driver1.pressedLeftBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    targetHit = false;
                    new RetransferCommand(turreting).schedule();
                })
                .transition(() -> driver1.pressedRightBumper(), State.INTAKING, () -> {
                    gamepad1.rumble(rumbleDur);
                    targetHit = false;
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

        sm.setState(State.INTAKING);

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
        telemetry.addLine( "TURN OFF INTAKE: RELEASE LEFT TRIGGER");
        telemetry.addLine( "EJECT: HOLD RIGHT TRIGGER");
        telemetry.addLine("TRANSFER: LEFT BUMPER");
        telemetry.addLine("SHOOT: RIGHT BUMPER");
        telemetry.addLine("INTAKE FROM ABOVE: X");

    }

    public void onStart(){
         new ElevatorDownCommand().schedule();
    }

    public void periodic(){
        llTagDetector.read();
        sm.update();

        //Shooter
        if(driver2.pressedRightBumper()){
            gamepad2.rumble(350);
            shooter.redAlliance = true;
            Globals.setAlliance(Alliance.RED);
        } else if(driver2.pressedRightTrigger()){
            gamepad2.rumble(350);
            shooter.redAlliance = false;
            Globals.setAlliance(Alliance.BLUE);
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

        if (driver2.pressedA() && !driver2.pressedOptions()) {
            gamepad2.rumble(1000);
            sixWheel.setPosition(new Pose2d(0, 0, 0));
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
        if (driver2.pressedLeftBumper() && turreting) {
            gamepad2.rumble(1000);
            turreting = false;
        } else if (driver2.pressedLeftBumper() && !turreting) {
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

        //relocalization
        if (driver1.pressedY()){
            if (llTagDetector.validLLReads()){
                gamepad2.rumble(200);
                sixWheel.setPosition(llTagDetector.getLLBotPose());
            }
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
        /*if (autoTagUpdating && llTagDetector.hasUpdatedPosition()){
            if (llTagDetector.validLLReads()){
                sixWheel.setPosition(llTagDetector.getLLBotPosePoseHistory());

        }*/

        if (driver1.pressedY()){
            gamepad2.rumble(200);
            autoTagUpdating = !autoTagUpdating;
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
        if (driver2.pressedY()){
            transfer.setAllUp();
        }
        if (driver2.pressedTouchpad()){
            transfer.setAllDown();
        }
        if (driver2.pressedLeftTrigger()){
            Intake.offset += 5;
            Intake.offset = ((Intake.offset + 90) % 180 + 180) % 180 - 90;
        }
//        if (driver2.pressedDpadLeft()){
//            llTagDetector.switchToPosition();
//        }
    }

    public void telemetry(){
        telemetry.addData("State", sm.getState());
    }

}
