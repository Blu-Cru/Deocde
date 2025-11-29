package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;

public class Elevator implements BluSubsystem, Subsystem {
    private BluServo elevatorServo;
    private BluBrushlandLabsColorSensor leftSensor, middleSensor, rightSensor;
    private static final double DOWN_POSITION = 0;//TODO: find positions
    private static final double UP_POSITION = 0.3;

    public Elevator(){
        elevatorServo = new BluServo("elevator");
    }

    public void setUp(){
        elevatorServo.setPos(UP_POSITION);
    }

    public void setDown(){
        elevatorServo.setPos(DOWN_POSITION);
    }
    public boolean ballInElevatorSlot(){
        return leftSensor.ballDetected() || middleSensor.ballDetected() || rightSensor.ballDetected();
    }

    @Override
    public void init() {
        elevatorServo.init();
    }

    @Override
    public void read() {
        elevatorServo.read();
    }

    @Override
    public void write() {
        elevatorServo.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        elevatorServo.telemetry();
    }

    @Override
    public void reset() {
        setDown();
    }
}
