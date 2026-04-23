package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "test")
public class ElevatorIsFullTest extends BluLinearOpMode {

    @Override
    public void initialize() {
        robot.clear();
        addElevator();
    }

    @Override
    public void periodic() {
        telemetry.addData("isFull", elevator.isFull());
        telemetry.addLine();
        telemetry.addData("LEFT slot",   label(elevator.getLeftBallColor()));
        telemetry.addData("MIDDLE slot", label(elevator.getMiddleBallColor()));
        telemetry.addData("RIGHT slot",  label(elevator.getRightBallColor()));
    }

    private String label(BallColor c) {
        if (c == BallColor.GREEN) return "GREEN";
        if (c == BallColor.PURPLE) return "PURPLE";
        return "NONE";
    }
}
