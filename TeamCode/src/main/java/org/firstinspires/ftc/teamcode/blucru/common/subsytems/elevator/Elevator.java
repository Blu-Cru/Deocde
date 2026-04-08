package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluBrushlandLabsColorRangefinder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;
@Config
public class Elevator implements BluSubsystem, Subsystem {
    private BluServo elevatorServoLeft;
    private BluServo elevatorServoRight;
    public static double DOWN_POSITION_LEFT = 0.39;// TODO: find positions
    public static double UP_POSITION_LEFT = 0.57;
    public static double MIDDLE_POSITION_LEFT = 0.48;
    public static double INTAKE_MIDDLE_POSITION_LEFT = 0.45;

    public static double DOWN_POSITION_RIGHT = 0.39;// TODO: find positions
    public static double UP_POSITION_RIGHT = 0.57;
    public static double MIDDLE_POSITION_RIGHT = 0.48;
    public static double INTAKE_MIDDLE_POSITION_RIGHT = 0.46;
    private BluBrushlandLabsColorRangefinder leftSensorBottom, leftSensorTop, middleSensorRight, middleSensorLeft, rightSensorBottom,
            rightSensorTop;

    public Elevator() {

        leftSensorBottom = new BluBrushlandLabsColorRangefinder("purpleLeftBottom", "greenLeftBottom");
        leftSensorTop = new BluBrushlandLabsColorRangefinder("purpleLeftTop", "greenLeftTop");
        middleSensorRight = new BluBrushlandLabsColorRangefinder("purpleMiddleRight", "greenMiddleRight");
        middleSensorLeft = new BluBrushlandLabsColorRangefinder("purpleMiddleLeft","greenMiddleLeft");
        rightSensorBottom = new BluBrushlandLabsColorRangefinder("purpleRightBottom", "greenRightBottom");
        rightSensorTop = new BluBrushlandLabsColorRangefinder("purpleRightTop", "greenRightTop");
        elevatorServoLeft = new BluServo("elevatorLeft");
        elevatorServoRight = new BluServo("elevatorRight");
        setDown();
        write();
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
        leftSensorTop.read();
        leftSensorBottom.read();
        middleSensorRight.read();
        middleSensorLeft.read();
        rightSensorTop.read();
        rightSensorBottom.read();
    }

    @Override
    public void write() {
        elevatorServoLeft.write();
        elevatorServoRight.write();
    }

    public void setUp() {
        elevatorServoLeft.setPos(UP_POSITION_LEFT);
        elevatorServoRight.setPos(UP_POSITION_RIGHT);
    }

    public void setDown() {
        elevatorServoLeft.setPos(DOWN_POSITION_LEFT);
        elevatorServoRight.setPos(DOWN_POSITION_RIGHT);
    }

    public void setMiddle() {
        elevatorServoLeft.setPos(MIDDLE_POSITION_LEFT);
        elevatorServoRight.setPos(MIDDLE_POSITION_RIGHT);
    }
    public void setMiddleForIntake() {
        elevatorServoLeft.setPos(INTAKE_MIDDLE_POSITION_LEFT);
        elevatorServoRight.setPos(INTAKE_MIDDLE_POSITION_RIGHT);
    }

    public boolean isFull(){
        return (leftSensorBottom.ballDetected() || leftSensorTop.ballDetected())
                && (rightSensorBottom.ballDetected() || rightSensorTop.ballDetected())
                && (middleSensorLeft.ballDetected() || middleSensorRight.ballDetected());
    }

    public void updateLeftBallColor() {
        BallColor targetColor = BallColor.UNKNOWN;
        if (leftSensorTop.greenBall() || leftSensorBottom.greenBall()){
            targetColor = BallColor.GREEN;
        } else if (leftSensorTop.purpleBall() || leftSensorBottom.purpleBall()){
            targetColor = BallColor.PURPLE;
        }

        ShooterMotifCoordinator.setLeftColor(targetColor);
    }

    public void updateMiddleBallColor() {
        BallColor targetColor = BallColor.UNKNOWN;
        if (middleSensorLeft.greenBall() || middleSensorRight.greenBall()){
            targetColor = BallColor.GREEN;
        } else if (middleSensorLeft.purpleBall() || middleSensorRight.purpleBall()){
            targetColor = BallColor.PURPLE;
        }

        ShooterMotifCoordinator.setMiddleColor(targetColor);
    }

    public void updateRightBallColor() {
        BallColor targetColor = BallColor.UNKNOWN;
        if (rightSensorTop.greenBall() || rightSensorBottom.greenBall()){
            targetColor = BallColor.GREEN;
        } else if (rightSensorTop.purpleBall() || rightSensorBottom.purpleBall()){
            targetColor = BallColor.PURPLE;
        }

        ShooterMotifCoordinator.setRightColor(targetColor);
    }

    public void turnOffElevatorServo() {
        elevatorServoLeft.disable();
        elevatorServoRight.disable();
        // always want to write after a disable
        elevatorServoLeft.write();
        elevatorServoRight.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        //elevatorServoLeft.telemetry();
        //elevatorServoRight.telemetry();
    }

    @Override
    public void reset() {
        setDown();
    }
}
