package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class Elevator implements BluSubsystem, Subsystem {
    private BluServo elevatorServoLeft;
    private BluServo elevatorServoRight;
    private static final double DOWN_POSITION_LEFT = 0.47;//TODO: find positions
    private static final double UP_POSITION_LEFT = 0.65;
    private static final double MIDDLE_POSITION_LEFT = 0.51;

    private static final double DOWN_POSITION_RIGHT = 0.47;//TODO: find positions
    private static final double UP_POSITION_RIGHT = 0.65;
    private static final double MIDDLE_POSITION_RIGHT = 0.52;
    private BluColorSensor leftSensorBottom, leftSensorTop, middleSensorRight, middleSensorLeft, rightSensorBottom, rightSensorTop;
    public Elevator(){
        leftSensorBottom = new BluColorSensor("leftColorSensorBottom", new double[][]{{0.01,0.015,0.02}, {0.016,0.025,0.035}, {0,0.02,0.027}, {0.01,0.035,0.05}});
        //leftSensorBottom = new BluColorSensor("leftColorSensorBottom", new double[][]{{0,0,0}, {1,1,1}, {0,0,0}, {1,1,1}});
        leftSensorTop = new BluColorSensor("leftColorSensorTop", new double[][]{{0.025,0.029,0.03}, {0.035,0.045,0.05}, {0.014,0.049,0.035}, {0.02,0.07,0.045}});
        middleSensorRight = new BluColorSensor("middleColorSensorRight", new double[][]{{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}});
        middleSensorLeft = new BluColorSensor("middleColorSensorLeft", new double[][]{{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}});
        rightSensorBottom = new BluColorSensor("rightColorSensorBottom", new double[][]{{0.023,0.033,0.043}, {0.037,0.047,0.067}, {0.006,0.041,0.031}, {0.026,0.064,0.054}});
        rightSensorTop = new BluColorSensor("rightColorSensorTop", new double[][]{{0.017,0.027,0.048}, {0.033,0.048,0.073}, {0.012,0.047,0.033}, {0.022,0.072,0.047}});
        elevatorServoLeft = new BluServo("elevatorLeft");
        elevatorServoRight = new BluServo("elevatorRight");
        setDown();
        write();
    }

    public void setUp(){
        elevatorServoLeft.setPos(UP_POSITION_LEFT);
        elevatorServoRight.setPos(UP_POSITION_RIGHT);
    }

    public void setDown(){
        elevatorServoLeft.setPos(DOWN_POSITION_LEFT);
        elevatorServoRight.setPos(DOWN_POSITION_RIGHT);
    }

    public void updateLeftBallColor(){
        BallColor targetColor = BallColor.UNKNOWN;
        leftSensorBottom.read();
        Globals.telemetry.addData("left red", leftSensorBottom.getRed());
        Globals.telemetry.addData("left blue", leftSensorBottom.getBlue());
        Globals.telemetry.addData("left green", leftSensorBottom.getGreen());
        if (leftSensorBottom.isPurple()){
            targetColor = BallColor.PURPLE;
        } else if (leftSensorBottom.isGreen()){
            targetColor = BallColor.GREEN;
        }

        if (targetColor != BallColor.UNKNOWN){
            ShooterMotifCoordinator.setLeftColor(targetColor);
            //done, exit
            return;
        }

        leftSensorTop.read();
        if (leftSensorTop.isPurple()){
            targetColor = BallColor.PURPLE;
        } else if (leftSensorTop.isGreen()){
            targetColor = BallColor.GREEN;
        }
        ShooterMotifCoordinator.setLeftColor(targetColor);
    }

    public void updateMiddleBallColor(){
        BallColor targetColor = BallColor.UNKNOWN;
        middleSensorLeft.read();
        if (middleSensorLeft.isPurple()){
            targetColor = BallColor.PURPLE;
        } else if (middleSensorLeft.isGreen()){
            targetColor = BallColor.GREEN;
        }

        if (targetColor != BallColor.UNKNOWN){
            ShooterMotifCoordinator.setMiddleColor(targetColor);
            //done, exit
            return;
        }

        middleSensorRight.read();
        if (middleSensorRight.isPurple()){
            targetColor = BallColor.PURPLE;
        } else if (middleSensorRight.isGreen()){
            targetColor = BallColor.GREEN;
        }
        ShooterMotifCoordinator.setMiddleColor(targetColor);
    }

    public void updateRightBallColor(){
        BallColor targetColor = BallColor.UNKNOWN;
        rightSensorBottom.read();
        if (rightSensorBottom.isPurple()){
            targetColor = BallColor.PURPLE;
        } else if (rightSensorBottom.isGreen()){
            targetColor = BallColor.GREEN;
        }

        if (targetColor != BallColor.UNKNOWN){
            ShooterMotifCoordinator.setRightColor(targetColor);
            //done, exit
            return;
        }
        rightSensorTop.read();


        if (rightSensorTop.isPurple()){
            targetColor = BallColor.PURPLE;
        } else if (rightSensorTop.isGreen()){
            targetColor = BallColor.GREEN;
        }
        ShooterMotifCoordinator.setRightColor(targetColor);
    }

    public void turnOffElevatorServo(){
        elevatorServoLeft.disable();
        elevatorServoRight.disable();
        //always want to write after a disable
        elevatorServoLeft.write();
        elevatorServoRight.write();
    }
    public void setMiddle(){
        elevatorServoLeft.setPos(MIDDLE_POSITION_LEFT);
        elevatorServoRight.setPos(MIDDLE_POSITION_RIGHT);
    }

    @Override
    public void init() {
        elevatorServoLeft.init();
        elevatorServoRight.init();
    }

    @Override
    public void read() {
        elevatorServoLeft.read();
        elevatorServoRight.read();
    }

    @Override
    public void write() {
        elevatorServoLeft.write();
        elevatorServoRight.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        elevatorServoLeft.telemetry();
        elevatorServoRight.telemetry();
    }

    @Override
    public void reset() {
        setDown();
    }
}
