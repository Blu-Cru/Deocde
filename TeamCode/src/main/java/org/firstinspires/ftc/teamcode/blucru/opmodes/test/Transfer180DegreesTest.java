package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.ElevatorUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretFrom180To0TransferCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.turretCommands.MoveTurretTo180DegreeTransferCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;
//@TeleOp
public class Transfer180DegreesTest extends BluLinearOpMode {

    public void initialize(){
        robot.clear();
        addTurret();
        addElevator();
        turret.resetEncoder();
    }

    public void periodic(){
        if (driver1.pressedA()){
            new MoveTurretTo180DegreeTransferCommand().schedule();
        }
        if (driver1.pressedY()){
            new MoveTurretFrom180To0TransferCommand().schedule();
        }
        if (driver1.pressedDpadUp()){
            new ElevatorUpCommand().schedule();
        }
        if (driver1.pressedDpadDown()){
            new ElevatorDownCommand().schedule();
        }
    }
}
