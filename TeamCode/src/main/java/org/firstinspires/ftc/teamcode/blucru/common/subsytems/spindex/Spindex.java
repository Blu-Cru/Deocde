package org.firstinspires.ftc.teamcode.blucru.common.subsytems.spindex;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluTouchSensor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class Spindex implements BluSubsystem {

    BluMotorWithEncoder spindexMotor;
    private final String spindexMotorName = "spindex";

    BluBrushlandColorSensor slot1sensor;
    BluBrushlandColorSensor slot2sensor;
    BluBrushlandColorSensor slot3sensor;
    BluTouchSensor limitSwitch;
    String[] slotBalls = new String[3];
    int amountOfEmptySpots;
    int greenSlot;
    int encoderRotations;

    public Spindex(){
        spindexMotor = new BluMotorWithEncoder(spindexMotorName);
        slot1sensor = new BluBrushlandColorSensor("slot1 pin0", "slot1 pin1");
        slot2sensor = new BluBrushlandColorSensor("slot2 pin0", "slot2 pin1");
        slot3sensor = new BluBrushlandColorSensor("slot3 pin0", "slot3 pin1");
        limitSwitch = new BluTouchSensor("touch sensor");
    }

    @Override
    public void init() {
        spindexMotor.init();
    }

    @Override
    public void read() {
        amountOfEmptySpots = 0;
        spindexMotor.read();
        slot1sensor.read();
        slot2sensor.read();
        slot3sensor.read();
        limitSwitch.read();
        if (limitSwitch.isPressed(true)){
            spindexMotor.reset();
            spindexMotor.read();
        }


        updateColorArray(slotBalls, slot1sensor.getPin0State(), slot1sensor.getPin1State(),
                slot2sensor.getPin0State(), slot2sensor.getPin1State(),
                slot3sensor.getPin0State(), slot3sensor.getPin1State());
    }

    private void updateColorArray(String[] slotBalls, boolean slot1pin0, boolean slot1pin1, boolean slot2pin0, boolean slot2pin1, boolean slot3pin0, boolean slot3pin1){
        if (slot1pin0){
            slotBalls[0] = "p";
        } else if (slot1pin1){
            greenSlot = 0;
            slotBalls[0] = "g";
        } else {
            slotBalls[0] = "e";
        }

        if (slot2pin0){
            slotBalls[1] = "p";
        } else if (slot2pin1){
            greenSlot = 1;
            slotBalls[1] = "g";
        } else {
            slotBalls[1] = "e";
        }

        if (slot3pin0){
            slotBalls[2] = "p";
        } else if (slot3pin1){
            greenSlot = 2;
            slotBalls[2] = "g";
        } else {
            slotBalls[2] = "e";
        }
    }

    @Override
    public void write() {
        spindexMotor.write();
    }

    public double getAmountOfEmptySlots(){
        return amountOfEmptySpots;
    }

    public void rotateToCorrectLaunchOrientation(int patternIndex){
        spindexMotor.setTargetPosition(Globals.convertMotorAngleToPosition(6000,(patternIndex-greenSlot)* 120));
    }

    public void moveOneSlot(){
        spindexMotor.setTargetPosition(spindexMotor.getCurrentPosition() + Globals.convertMotorAngleToPosition(6000, 120));
    }

    public void fixForTouchSensor(){
        spindexMotor.setTargetPosition(spindexMotor.getTargetPosition() % encoderRotations);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        Globals.telemetry.addData("Spindex Motor Power", spindexMotor.getPower());
        slot1sensor.telemetry();
        slot2sensor.telemetry();
        slot3sensor.telemetry();
    }

    @Override
    public void reset() {
        spindexMotor.reset();
    }
}
