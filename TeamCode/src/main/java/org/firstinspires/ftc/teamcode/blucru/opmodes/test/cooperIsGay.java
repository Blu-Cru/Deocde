package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blucru.common.commands.ParallelizeIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

@TeleOp(group = "cooper is gay")
public class cooperIsGay extends BluLinearOpMode {

    public enum State{
    }

    State state;
    DigitalChannel channel;

    public void initialize(){
        robot.clear();
        addDrivetrain();
        addIntake();
        addTurret();
        drivetrain.reset();
        turret.resetEncoder();
        intake.resetEncoder();

    }


    public void periodic(){

    }
}
