package org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.ShooterMotifCoordinator;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;

public class Elevator implements BluSubsystem, Subsystem {
    private BluServo elevatorServoLeft;
    private BluServo elevatorServoRight;
    private static final double DOWN_POSITION_LEFT = 0.42;// TODO: find positions
    private static final double UP_POSITION_LEFT = 0.65;
    private static final double MIDDLE_POSITION_LEFT = 0.51;

    private static final double DOWN_POSITION_RIGHT = 0.42;// TODO: find positions
    private static final double UP_POSITION_RIGHT = 0.65;
    private static final double MIDDLE_POSITION_RIGHT = 0.52;

    private BluColorSensor leftSensorBottom, leftSensorTop, middleSensorRight, middleSensorLeft, rightSensorBottom,
            rightSensorTop;

    public Elevator() {
        leftSensorBottom = new BluColorSensor("leftColorSensorBottom");
        // leftSensorBottom = new BluColorSensor("leftColorSensorBottom", new
        // double[][]{{0,0,0}, {1,1,1}, {0,0,0}, {1,1,1}});
        leftSensorTop = new BluColorSensor("leftColorSensorTop");
        middleSensorRight = new BluColorSensor("middleColorSensorRight");
        middleSensorLeft = new BluColorSensor("middleColorSensorLeft");
        rightSensorBottom = new BluColorSensor("rightColorSensorBottom");
        rightSensorTop = new BluColorSensor("rightColorSensorTop");
        elevatorServoLeft = new BluServo("elevatorLeft");
        elevatorServoRight = new BluServo("elevatorRight");
        setDown();
        write();
    }

    public void setUp() {
        elevatorServoLeft.setPos(UP_POSITION_LEFT);
        elevatorServoRight.setPos(UP_POSITION_RIGHT);
    }

    public void setDown() {
        elevatorServoLeft.setPos(DOWN_POSITION_LEFT);
        elevatorServoRight.setPos(DOWN_POSITION_RIGHT);
    }

    public void updateLeftBallColor() {
        final double greenRed = 0.01716;
        final double greenBlue = 0.03354;
        final double greenGreen = 0.0519;
        final double purpleRed = 0.03426;
        final double purpleBlue = 0.03934;
        final double purpleGreen = 0.03086;
        BallColor targetColor;
        leftSensorBottom.read();
        leftSensorTop.read();
        double red = Math.max(leftSensorBottom.getRed(), leftSensorTop.getRed());
        double green = Math.max(leftSensorBottom.getGreen(), leftSensorTop.getGreen());
        double blue = Math.max(leftSensorBottom.getBlue(), leftSensorTop.getBlue());
        double mag = Math.hypot(red, Math.hypot(green, blue));
        // Globals.telemetry.addData("Red Left", red);
        // Globals.telemetry.addData("Blue Left", blue);
        // Globals.telemetry.addData("Green Left",green);
        if (mag > 0.0075) {
            double dotGreen = red * greenRed + blue * greenBlue + green * greenGreen;
            double dotPurple = red * purpleRed + blue * purpleBlue + green * purpleGreen;
            double cosPurple = dotPurple / (mag * Math.hypot(purpleRed, Math.hypot(purpleBlue, purpleGreen)));
            double cosGreen = dotGreen / (mag * Math.hypot(greenRed, Math.hypot(greenBlue, greenGreen)));
            double angleBetweenGreen = Math.acos(cosGreen);
            double angleBetweenPurple = Math.acos(cosPurple);
            if (angleBetweenGreen < angleBetweenPurple) {
                // farther away from purple than green
                targetColor = BallColor.GREEN;
            } else {
                targetColor = BallColor.PURPLE;
            }
        } else {
            targetColor = BallColor.UNKNOWN;
        }

        ShooterMotifCoordinator.setLeftColor(targetColor);
    }

    public void updateMiddleBallColor() {
        final double greenRed = 0.00172;
        final double greenBlue = 0.0041;
        final double greenGreen = 0.00462;
        final double purpleRed = 0.0022;
        final double purpleBlue = 0.00428;
        final double purpleGreen = 0.00344;
        BallColor targetColor;
        middleSensorLeft.read();
        middleSensorRight.read();
        double red = Math.max(middleSensorLeft.getRed(), middleSensorRight.getRed());
        double green = Math.max(middleSensorLeft.getGreen(), middleSensorRight.getGreen());
        double blue = Math.max(middleSensorLeft.getBlue(), middleSensorRight.getBlue());
        // Globals.telemetry.addData("Red Middle", red);
        // Globals.telemetry.addData("Blue Middle", blue);
        // Globals.telemetry.addData("Green Middle",green);
        double mag = Math.hypot(red, Math.hypot(green, blue));
        if (mag > 0.003) {
            double dotGreen = red * greenRed + blue * greenBlue + green * greenGreen;
            double dotPurple = red * purpleRed + blue * purpleBlue + green * purpleGreen;
            double cosPurple = dotPurple / (mag * Math.hypot(purpleRed, Math.hypot(purpleBlue, purpleGreen)));
            double cosGreen = dotGreen / (mag * Math.hypot(greenRed, Math.hypot(greenBlue, greenGreen)));
            double angleBetweenGreen = Math.acos(cosGreen);
            double angleBetweenPurple = Math.acos(cosPurple);
            if (angleBetweenGreen < angleBetweenPurple) {
                // farther away from purple than green
                targetColor = BallColor.GREEN;
            } else {
                targetColor = BallColor.PURPLE;
            }
        } else {
            targetColor = BallColor.UNKNOWN;
        }

        ShooterMotifCoordinator.setMiddleColor(targetColor);
    }

    public void updateRightBallColor() {
        final double greenRed = 0.01398;
        final double greenBlue = 0.03342;
        final double greenGreen = 0.04712;
        final double purpleRed = 0.02484;
        final double purpleBlue = 0.04418;
        final double purpleGreen = 0.0293;
        BallColor targetColor;
        rightSensorTop.read();
        rightSensorBottom.read();
        double red = Math.max(rightSensorTop.getRed(), rightSensorBottom.getRed());
        double green = Math.max(rightSensorTop.getGreen(), rightSensorBottom.getGreen());
        double blue = Math.max(rightSensorTop.getBlue(), rightSensorBottom.getBlue());
        // Globals.telemetry.addData("Red Right", red);
        // Globals.telemetry.addData("Blue Right", blue);
        // Globals.telemetry.addData("Green Right",green);
        double mag = Math.hypot(red, Math.hypot(green, blue));
        if (mag > 0.0075) {
            double dotGreen = red * greenRed + blue * greenBlue + green * greenGreen;
            double dotPurple = red * purpleRed + blue * purpleBlue + green * purpleGreen;
            double cosPurple = dotPurple / (mag * Math.hypot(purpleRed, Math.hypot(purpleBlue, purpleGreen)));
            double cosGreen = dotGreen / (mag * Math.hypot(greenRed, Math.hypot(greenBlue, greenGreen)));
            double angleBetweenGreen = Math.acos(cosGreen);
            double angleBetweenPurple = Math.acos(cosPurple);
            if (angleBetweenGreen < angleBetweenPurple) {
                // farther away from purple than green
                targetColor = BallColor.GREEN;
            } else {
                targetColor = BallColor.PURPLE;
            }
        } else {
            targetColor = BallColor.UNKNOWN;
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

    public void setMiddle() {
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
        // elevatorServoLeft.telemetry();
        // elevatorServoRight.telemetry();
    }

    @Override
    public void reset() {
        setDown();
    }
}
