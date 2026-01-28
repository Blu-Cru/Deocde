package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;

public class Elevator implements BluSubsystem, Subsystem {
    private BluServo elevatorServo;
    private static final double DOWN_POSITION = 0.01;//TODO: find positions
    private static final double UP_POSITION = 0.3;
    private static final double MIDDLE_POSITION = 0.13;
    private static final double BOTTOM_PURPLE_BLUE = 50;
    private static final double TOP_PURPLE_BLUE = 100;
    private static final double BOTTOM_GREEN_GREEN = 70;
    private static final double TOP_GREEN_GREEN = 140;
    private BluColorSensor leftSensorBottom, leftSensorTop, middleSensorRight, middleSensorLeft, rightSensorBottom, rightSensorTop;
    public Elevator(){
        elevatorServo = new BluServo("elevator");
        leftSensorBottom = new BluColorSensor("leftColorSensorBottom", new double[][]{{0,0,0}, {0,0,0}, {0,0,0}});
        leftSensorTop = new BluColorSensor("leftColorSensorTop", new double[][]{{0,0,0}, {0,0,0}, {0,0,0}});
        middleSensorRight = new BluColorSensor("middleColorSensorRight", new double[][]{{0,0,0}, {0,0,0}, {0,0,0}});
        middleSensorLeft = new BluColorSensor("middleColorSensorLeft", new double[][]{{0,0,0}, {0,0,0}, {0,0,0}});
        rightSensorBottom = new BluColorSensor("rightColorSensorBottom", new double[][]{{0,0,0}, {0,0,0}, {0,0,0}});
        rightSensorTop = new BluColorSensor("rightColorSensorTop", new double[][]{{0,0,0}, {0,0,0}, {0,0,0}});
        setDown();
        write();
    }

    public void setUp(){
        elevatorServo.setPos(UP_POSITION);
    }

    public void setDown(){
        elevatorServo.setPos(DOWN_POSITION);
    }

    public void updateLeftBallColor(){
        BallColor targetColor = BallColor.UNKNOWN;
        leftSensorTop.read();
        if (leftSensorTop.getBlue() < TOP_PURPLE_BLUE && leftSensorTop.getBlue() > BOTTOM_PURPLE_BLUE){
            targetColor = BallColor.PURPLE;
        } else if (leftSensorTop.getGreen() < TOP_GREEN_GREEN && leftSensorTop.getGreen() > BOTTOM_GREEN_GREEN){
            targetColor = BallColor.GREEN;
        }

        if (targetColor != BallColor.UNKNOWN){
            ShooterMotifCoordinator.setLeftColor(targetColor);
            //done, exit
            return;
        }

        leftSensorBottom.read();
        if (leftSensorBottom.getBlue() < TOP_PURPLE_BLUE && leftSensorBottom.getBlue() > BOTTOM_PURPLE_BLUE){
            targetColor = BallColor.PURPLE;
        } else if (leftSensorBottom.getGreen() < TOP_GREEN_GREEN && leftSensorBottom.getGreen() > BOTTOM_GREEN_GREEN){
            targetColor = BallColor.GREEN;
        }
        ShooterMotifCoordinator.setLeftColor(targetColor);
    }

    public void updateMiddleBallColor(){
        BallColor targetColor = BallColor.UNKNOWN;
        middleSensorLeft.read();
        if (middleSensorLeft.getBlue() < TOP_PURPLE_BLUE && middleSensorLeft.getBlue() > BOTTOM_PURPLE_BLUE){
            targetColor = BallColor.PURPLE;
        } else if (middleSensorLeft.getGreen() < TOP_GREEN_GREEN && middleSensorLeft.getGreen() > BOTTOM_GREEN_GREEN){
            targetColor = BallColor.GREEN;
        }

        if (targetColor != BallColor.UNKNOWN){
            ShooterMotifCoordinator.setMiddleColor(targetColor);
            //done, exit
            return;
        }

        middleSensorRight.read();
        if (middleSensorRight.getBlue() < TOP_PURPLE_BLUE && middleSensorRight.getBlue() > BOTTOM_PURPLE_BLUE){
            targetColor = BallColor.PURPLE;
        } else if (middleSensorRight.getGreen() < TOP_GREEN_GREEN && middleSensorRight.getGreen() > BOTTOM_GREEN_GREEN){
            targetColor = BallColor.GREEN;
        }
        ShooterMotifCoordinator.setMiddleColor(targetColor);
    }

    public void updateRightBallColor(){
        BallColor targetColor = BallColor.UNKNOWN;
        rightSensorTop.read();
        if (rightSensorTop.getBlue() < TOP_PURPLE_BLUE && rightSensorTop.getBlue() > BOTTOM_PURPLE_BLUE){
            targetColor = BallColor.PURPLE;
        } else if (rightSensorTop.getGreen() < TOP_GREEN_GREEN && rightSensorTop.getGreen() > BOTTOM_GREEN_GREEN){
            targetColor = BallColor.GREEN;
        }

        if (targetColor != BallColor.UNKNOWN){
            ShooterMotifCoordinator.setRightColor(targetColor);
            //done, exit
            return;
        }

        rightSensorBottom.read();
        if (rightSensorBottom.getBlue() < TOP_PURPLE_BLUE && rightSensorBottom.getBlue() > BOTTOM_PURPLE_BLUE){
            targetColor = BallColor.PURPLE;
        } else if (rightSensorBottom.getGreen() < TOP_GREEN_GREEN && rightSensorBottom.getGreen() > BOTTOM_GREEN_GREEN){
            targetColor = BallColor.GREEN;
        }
        ShooterMotifCoordinator.setRightColor(targetColor);
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
