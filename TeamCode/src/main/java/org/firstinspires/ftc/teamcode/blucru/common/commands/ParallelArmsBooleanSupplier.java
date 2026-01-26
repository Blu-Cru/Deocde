package org.firstinspires.ftc.teamcode.blucru.common.commands;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

import java.util.function.BooleanSupplier;

public class ParallelArmsBooleanSupplier implements BooleanSupplier {
    @Override
    public boolean getAsBoolean() {
        return Robot.getInstance().intake.armsParallel();
    }
}
