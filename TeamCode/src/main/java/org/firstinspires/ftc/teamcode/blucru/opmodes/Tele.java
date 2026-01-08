package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commands.IdleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ResetForIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ReturnCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.ShootBallsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.UnshootCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commands.RetransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetLeftHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetMiddleHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.SetRightHoodAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.shooterCommands.ShootReverseWithVelocityCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferMiddleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Pose2d;

@TeleOp (group = "a")

public class Tele extends BluLinearOpMode{

    StateMachine sm;
    public boolean turreting = true;
    public boolean autoTagUpdating = true;
    public int rumbleDur = 200;
    public int shot = 0;

    public enum State{
        IDLE,
        INTAKING,
        DRIVING_TO_SHOOT,
        INTAKING_FROM_ABOVE
    }

    @Override
    public void initialize(){
        robot.clear();
        addLLTagDetector();
        addSixWheel();
        addIntake();
        addElevator();
        addTransfer();
        addShooter();
        addTurret();

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
                        new AllTransferMiddleCommand(),
                        new SetLeftHoodAngleCommand(26),
                        new SetMiddleHoodAngleCommand(26),
                        new SetRightHoodAngleCommand(26),
                        new ShootReverseWithVelocityCommand(200)
                    ).schedule();
                })

                .state(State.INTAKING)
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.2){
                        intake.setOut();
                    } else if (gamepad1.right_trigger > 0.2){
                        intake.setIn();
                    } else {
                        intake.stop();
                    }
                })
                .transition(() -> driver1.pressedRightBumper(), State.IDLE, () -> {
                    gamepad1.rumble(rumbleDur);
                    robot.idleRobot();
                    new IdleCommand().schedule();
                })
                .transition(() -> driver1.pressedLeftBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot = 0;
                    new TransferCommand(turreting).schedule();
                })

                .state(State.DRIVING_TO_SHOOT)
                .transition(() -> driver1.pressedLeftBumper(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    new RetransferCommand(turreting).schedule();
                })
                .transition(() -> driver1.pressedRightBumper(), State.INTAKING, () -> {
                    gamepad1.rumble(rumbleDur);
                    new ConditionalCommand(
                            new ShootBallsCommand(),
                            new ReturnCommand(),
                            () -> (shot == 0)
                    ).schedule();
                })
                .transition(() -> driver1.pressedDpadLeft(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot+=1;
                    new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Robot.getInstance().sixWheelDrivetrain.makeMotorsBeInBrake();
                            }),
                            new LeftTransferUpCommand()
                    ).schedule();
                })
                .transition(() -> driver1.pressedDpadUp(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot+=1;
                    new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Robot.getInstance().sixWheelDrivetrain.makeMotorsBeInBrake();
                            }),
                            new MiddleTransferUpCommand()
                    ).schedule();
                })
                .transition(() -> driver1.pressedDpadRight(), State.DRIVING_TO_SHOOT, () -> {
                    gamepad1.rumble(rumbleDur);
                    shot+=1;
                    new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Robot.getInstance().sixWheelDrivetrain.makeMotorsBeInBrake();
                            }),
                            new RightTransferUpCommand()
                    ).schedule();
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

    public void initializePeriodic(){
        telemetry.addLine( "TURN ON INTAKE: HOLD LEFT TRIGGER");
        telemetry.addLine( "TURN OFF INTAKE: RELEASE LEFT TRIGGER");
        telemetry.addLine( "EJECT: HOLD RIGHT TRIGGER");
        telemetry.addLine("TRANSFER: LEFT BUMPER");
        telemetry.addLine("SHOOT: RIGHT BUMPER");
    }

    public void onStart(){
         new ElevatorDownCommand().schedule();
    }

    public void periodic(){
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

        //Drivetrain
        sixWheel.teleDrive(gamepad1, 0.0001);
//        if (driver2.pressedB() && !driver2.pressedOptions()){
//            sixWheel.setPosition(llTagDetector.getLLBotPose());
//        }

        if (driver2.pressedA()) {
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
            turret.setPower(gamepad2.right_stick_x*0.5);
        }




        //relocalization
        if (driver2.pressedDpadUp()){
            if (llTagDetector.validLLReads()){
                gamepad1.rumble(200);
                sixWheel.setPosition(llTagDetector.getLLBotPose());
            }
        }
        /*if (autoTagUpdating && llTagDetector.hasUpdatedPosition()){
            if (llTagDetector.validLLReads()){
                sixWheel.setPosition(llTagDetector.getLLBotPosePoseHistory());
            }
        }*/

        if (driver2.pressedDpadUp()){
            gamepad2.rumble(200);
            autoTagUpdating = !autoTagUpdating;
        }

        //manual heading update
        if (driver2.pressedDpadDown()){
            sixWheel.setHeading(0);
        }
    }

    public void telemetry(){
        telemetry.addData("State", sm.getState());
    }

}
