package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public class ElevatorTest extends BluLinearOpMode {

    public void initialize(){
        addElevator();
        elevator.setDown();
        elevator.write();
    }

    public void periodic(){
        if (gamepad1.y){
            elevator.setUp();
        }
        if (gamepad1.a){
            elevator.setDown();
        }
        if (gamepad1.x || gamepad1.b){
            elevator.setMiddle();
        }
    }

}
