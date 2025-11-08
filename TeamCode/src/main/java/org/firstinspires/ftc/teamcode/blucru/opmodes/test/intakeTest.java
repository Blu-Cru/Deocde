package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.Intake;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class intakeTest extends BluLinearOpMode {

    private static final String LEFT_NAME = "intake_left";
    private static final String RIGHT_NAME = "intake_right";

    private Intake leftIntake;
    private Intake rightIntake;

    @Override
    public void initialize(){
        robot.clear();

    leftIntake = new Intake(LEFT_NAME);
    rightIntake = new Intake(RIGHT_NAME);

    // register and initialize them locally
    leftIntake.init();
    rightIntake.init();

    // start stopped
    leftIntake.stop();
    rightIntake.stop();
    }

    @Override
    public void periodic(){
        // direct trigger checks (no separate threshold variable)
        if (gamepad1.left_trigger > 0.2) {
            leftIntake.setIn();
            rightIntake.setIn();
        } else if (gamepad1.right_trigger > 0.2) {
            leftIntake.setOut();
            rightIntake.setOut();
        } else {
            leftIntake.stop();
            rightIntake.stop();
        }

        // run read/write so the subsystem updates sensors and applies motor power immediately
        leftIntake.read();
        rightIntake.read();
        leftIntake.write();
        rightIntake.write();
    }

    @Override
    public void telemetry(){
        // Show intake states and jam status for testing
        if (leftIntake != null) {
            telemetry.addData("Left Intake State", leftIntake.getState());
            telemetry.addData("Left Intake Jammed", leftIntake.jammed);
            // include motor-level telemetry (current, etc.)
            leftIntake.telemetry(telemetry);
        }

        if (rightIntake != null) {
            telemetry.addData("Right Intake State", rightIntake.getState());
            telemetry.addData("Right Intake Jammed", rightIntake.jammed);
            rightIntake.telemetry(telemetry);
        }
    }

}
