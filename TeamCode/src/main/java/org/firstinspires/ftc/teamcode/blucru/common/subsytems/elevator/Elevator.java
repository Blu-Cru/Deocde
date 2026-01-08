package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;

public class Elevator implements BluSubsystem, Subsystem {
    private BluServo elevatorServo;
    private static final double DOWN_POSITION = 0.01;//TODO: find positions
    private static final double UP_POSITION = 0.3;
    private static final double MIDDLE_POSITION = 0.13;

    public Elevator(){
        elevatorServo = new BluServo("elevator");
        setDown();
        write();
    }

    public void setUp(){
        elevatorServo.setPos(UP_POSITION);
    }

    public void setDown(){
        elevatorServo.setPos(DOWN_POSITION);
    }
    public void turnOffElevatorServo(){
        elevatorServo.disable();
        //always want to write after a disable
        elevatorServo.write();
    }
    public void setMiddle(){
        elevatorServo.setPos(MIDDLE_POSITION);
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
