package org.firstinspires.ftc.teamcode.blucru.common.pathing;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;

public class SixWheelDrivetrainSetSimulatedBatteryVoltageCommand extends InstantCommand {
    public SixWheelDrivetrainSetSimulatedBatteryVoltageCommand(Double simulatedBatteryVoltage) {
        super(() -> Robot.getInstance().sixWheelDrivetrain.setSimulatedBatteryVoltage(simulatedBatteryVoltage));

        addRequirements(Robot.getInstance().sixWheelDrivetrain);
    }
}
