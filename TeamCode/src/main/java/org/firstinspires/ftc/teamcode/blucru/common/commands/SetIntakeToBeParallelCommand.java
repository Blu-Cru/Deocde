package org.firstinspires.ftc.teamcode.blucru.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStartCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.SetCustomIntakePowerCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class SetIntakeToBeParallelCommand extends InstantCommand {

    public SetIntakeToBeParallelCommand(){
        new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Robot.getInstance().intake.setParallelingArms(true);
                }),
                new SetCustomIntakePowerCommand(0.5),
                new WaitCommand((long) (20)),
                new IntakeStopCommand(),
                new WaitCommand(300) ,
                new InstantCommand(() -> {
                    Robot.getInstance().intake.setParallelingArms(false);
                })
        ).schedule();
    }


}
