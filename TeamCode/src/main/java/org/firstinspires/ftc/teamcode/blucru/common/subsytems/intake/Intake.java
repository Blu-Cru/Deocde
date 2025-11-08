package org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

@Config
public class Intake implements BluSubsystem, Subsystem {
    private BluMotor motor;
    public boolean jammed;
    public static double JAM_CURRENT_THRESHOLD = 9800; // milliamps, adjust as needed
    public static double NOMINAL_VOLTAGE = 12.0;
    // how long to run OUT (spit) when a jam is detected, in seconds
    public static double SPIT_DURATION = 0.5;
    public enum State{
        IN,
        OUT,
        IDlE
    }
    private State state;
    // if a jam is detected we will automatically run OUT for a short time
    private boolean autoSpitting = false;
    private long spitUntilMs = 0;

    public void setIn() {
        state = State.IN;
    }

    public void setOut() {
        state = State.OUT;
    }

    public void stop(){
        state = State.IDlE;
    }

    public void setIdle() {
        state = State.IDlE;
    }

    public State getState() {
        return state;
    }

    public Intake(String motorName) {
        motor = new BluMotor(motorName);
        state = State.IDlE;
        jammed = false;
    }

    @Override
    public void init() {
        motor.init();
    }

    @Override
    public void read() {
        motor.read();
        double currentVoltage = Robot.getInstance().getVoltage();
        double adjustedThreshold = JAM_CURRENT_THRESHOLD * (currentVoltage / NOMINAL_VOLTAGE);

        boolean detected = (state == State.IN && motor.getCurrent() > adjustedThreshold);

        // If we detect a jam while the driver is commanding IN, schedule a short automatic spit
        if (detected && !autoSpitting) {
            autoSpitting = true;
            spitUntilMs = System.currentTimeMillis() + (long)(SPIT_DURATION * 1000);
        }

        jammed = detected;
    }

    @Override
    public void write() {
        long now = System.currentTimeMillis();

        if (autoSpitting) {
            if (now < spitUntilMs) {
                // actively spitting to clear jam
                motor.setPower(1);
            } else {
                // spit period finished, clear auto-spit and jam flag and return to manual control
                autoSpitting = false;
                jammed = false;
                spitUntilMs = 0;
                // do not re-enter IN automatically; leave state as IDlE so driver regains manual control
                state = State.IDlE;
            }
        }

        if (!autoSpitting) {
            switch(state){
                case IN:
                    motor.setPower(-1);
                    break;
                case OUT:
                    motor.setPower(1);
                    break;
                case IDlE:
                    motor.setPower(0);
                    break;
            }
        }

        motor.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        motor.telemetry();
        telemetry.addData("Current", motor.getCurrent());
        telemetry.addData("Jammed", jammed);
        telemetry.addData("AutoSpitting", autoSpitting);
        if (autoSpitting) {
            telemetry.addData("Spit ms remaining", Math.max(0, spitUntilMs - System.currentTimeMillis()));
        }
    }

    @Override
    public void reset() {
        motor.setPower(0);
        motor.write();
    }
}
